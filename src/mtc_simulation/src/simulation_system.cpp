#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <mtc_interfaces/srv/latch.hpp>
#include <future>
#include <mutex>
#include <thread>
#include <map>
#include <set>
#include <nlohmann/json.hpp>

namespace mtc {
class SimulationSystem : public gz::sim::System, public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  using Entity=gz::sim::Entity;
  struct Command {std::string object;bool close;std::promise<std::pair<bool,std::string>> result;};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr poses_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr latch_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_pub_;
  bool contacts_enabled_=false;
  uint64_t contact_steps_=0;
  int64_t contact_publish_ns_=-1000000000;
  nlohmann::json contacts_=nlohmann::json::object();
  rclcpp::Service<mtc_interfaces::srv::Latch>::SharedPtr service_;
  std::mutex mutex_;
  std::shared_ptr<Command> pending_;
  Entity robot_=gz::sim::kNullEntity,tcp_=gz::sim::kNullEntity,joint_=gz::sim::kNullEntity;
  std::string attached_;
  std::map<std::string,Entity> bodies_;
  int64_t last_publish_=-1000000000;
  bool locate(gz::sim::EntityComponentManager& ecm) {
    if(robot_!=gz::sim::kNullEntity && tcp_!=gz::sim::kNullEntity && bodies_.size()==4)return true;
    robot_=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("mtc_s3"));
    if(robot_==gz::sim::kNullEntity)return false;
    tcp_=gz::sim::Model(robot_).LinkByName(ecm,"tcp");
    for(char letter='A';letter<='D';++letter){std::string name="battery_"+std::string(1,letter);auto m=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(name));if(m!=gz::sim::kNullEntity)bodies_[name]=gz::sim::Model(m).LinkByName(ecm,"body");}
    return tcp_!=gz::sim::kNullEntity&&bodies_.size()==4;
  }
 public:
  ~SimulationSystem() override {if(executor_)executor_->cancel();if(thread_.joinable())thread_.join();}
  void Configure(const Entity&,const std::shared_ptr<const sdf::Element>&,gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
    if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}
    executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node_=std::make_shared<rclcpp::Node>("mtc_simulation_ground_truth");
    poses_=node_->create_publisher<tf2_msgs::msg::TFMessage>("/simulation/ground_truth",rclcpp::QoS(5).reliable());
    latch_state_=node_->create_publisher<std_msgs::msg::String>("/simulation/latch_state",rclcpp::QoS(1).reliable().transient_local());
    contact_pub_=node_->create_publisher<std_msgs::msg::String>("/simulation/contact_report",rclcpp::QoS(1).reliable().transient_local());
    service_=node_->create_service<mtc_interfaces::srv::Latch>("/simulation/latch",[this](const std::shared_ptr<mtc_interfaces::srv::Latch::Request> req,std::shared_ptr<mtc_interfaces::srv::Latch::Response> res){
      auto cmd=std::make_shared<Command>();cmd->object=req->object_id;cmd->close=req->close;auto future=cmd->result.get_future();
      {std::lock_guard<std::mutex> lock(mutex_);if(pending_){res->success=false;res->message="A latch command is already pending";return;}pending_=cmd;}
      if(future.wait_for(std::chrono::seconds(3))!=std::future_status::ready){std::lock_guard<std::mutex> lock(mutex_);if(pending_==cmd)pending_.reset();res->success=false;res->message="Simulator paused or latch update timed out";return;}
      auto result=future.get();res->success=result.first;res->message=result.second;
    });
    executor_->add_node(node_);thread_=std::thread([this](){executor_->spin();});
    RCLCPP_INFO(node_->get_logger(),"Ground truth and position-gated split latch enabled; no pose teleportation");
  }
  void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
    if(info.paused)return;
    locate(ecm);
    if(!contacts_enabled_&&tcp_!=gz::sim::kNullEntity){std::vector<Entity> collisions;ecm.Each<gz::sim::components::Collision>([&](const Entity& e,const gz::sim::components::Collision*){collisions.push_back(e);return true;});for(auto e:collisions)if(!ecm.Component<gz::sim::components::ContactSensorData>(e))ecm.CreateComponent(e,gz::sim::components::ContactSensorData());contacts_enabled_=true;}
    std::shared_ptr<Command> cmd;{std::lock_guard<std::mutex> lock(mutex_);cmd=pending_;pending_.reset();}
    if(!cmd)return;
    bool ok=false;std::string message;
    if(!locate(ecm)){message="Robot TCP or battery entities unavailable";}
    else if(cmd->close){
      if(!attached_.empty())message="Latch already engaged";
      else if(!bodies_.count(cmd->object))message="Unknown battery";
      else {
        auto tcp=gz::sim::worldPose(tcp_,ecm);auto body=gz::sim::worldPose(bodies_.at(cmd->object),ecm);
        auto handle=body.Pos()+body.Rot().RotateVector(gz::math::Vector3d(0,0,.074));
        double error=(tcp.Pos()-handle).Length();double alignment=std::abs(tcp.Rot().RotateVector(gz::math::Vector3d::UnitX).Dot(body.Rot().RotateVector(gz::math::Vector3d::UnitX)));
        bool closed=true;for(const auto& name:{"latch_left_joint","latch_right_joint"}){auto j=gz::sim::Model(robot_).JointByName(ecm,name);auto q=ecm.Component<gz::sim::components::JointPosition>(j);closed=closed&&q&&!q->Data().empty()&&std::abs(q->Data()[0])<.002;}
        if(!closed)message="Close both latch halves before engagement";
        else if(error>.0025||alignment<.995)message="Handle alignment gate rejected: error_m="+std::to_string(error)+", axis_dot="+std::to_string(alignment);
        else {
          joint_=ecm.CreateEntity();ecm.CreateComponent(joint_,gz::sim::components::DetachableJoint({tcp_,bodies_.at(cmd->object),"fixed"}));attached_=cmd->object;ok=true;message="Mechanical lock engaged at measured relative pose";
        }
      }
    } else {
      if(attached_.empty()){ok=true;message="Latch already released";}
      else {ecm.RequestRemoveEntity(joint_);joint_=gz::sim::kNullEntity;attached_.clear();ok=true;message="Lock released; object remains under gravity and contact dynamics";}
    }
    cmd->result.set_value({ok,message});
    if(ok)RCLCPP_INFO(node_->get_logger(),"%s",message.c_str());else RCLCPP_WARN(node_->get_logger(),"%s",message.c_str());
  }
  void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& ecm) override {
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if(!info.paused&&contacts_enabled_){++contact_steps_;std::set<std::pair<Entity,Entity>> seen;
      ecm.Each<gz::sim::components::Collision,gz::sim::components::ContactSensorData>([&](const Entity&,const gz::sim::components::Collision*,const gz::sim::components::ContactSensorData* data){
        for(const auto& contact:data->Data().contact()){
          Entity a=contact.collision1().id(),b=contact.collision2().id();if(a>b)std::swap(a,b);if(!seen.insert({a,b}).second)continue;
          auto name=[&](Entity e){auto parent=ecm.Component<gz::sim::components::ParentEntity>(e);return parent?gz::sim::scopedName(parent->Data(),ecm,"::",false):std::to_string(e);};
          std::string x=name(a),y=name(b),key=x+" / "+y;
          auto has=[](const std::string& s,const std::string& part){return s.find(part)!=std::string::npos;};
          bool battery_x=has(x,"battery_"),battery_y=has(y,"battery_"),table_x=has(x,"::table::"),table_y=has(y,"::table::"),finger_x=has(x,"latch_left")||has(x,"latch_right"),finger_y=has(y,"latch_left")||has(y,"latch_right");
          std::string kind=((battery_x&&table_y)||(battery_y&&table_x))?"battery_table_support":((has(x,"base_link")&&table_y)||(has(y,"base_link")&&table_x))?"base_mount":((finger_x&&battery_y)||(finger_y&&battery_x))?"latch_handle_contact":(finger_x&&finger_y)?"latch_closure":"unexpected";
          const auto* collision_a=ecm.Component<gz::sim::components::Name>(a);const auto* collision_b=ecm.Component<gz::sim::components::Name>(b);
          if(kind=="latch_handle_contact"&&((battery_x&&collision_a&&collision_a->Data()=="part_0")||(battery_y&&collision_b&&collision_b->Data()=="part_0")))kind="unexpected";
          double depth=0;for(double d:contact.depth())depth=std::max(depth,d);
          if(!contacts_.contains(key))contacts_[key]={{"kind",kind},{"steps",0},{"first_sim_time",ns*1e-9},{"max_depth_m",0.}};
          auto& row=contacts_[key];row["steps"]=row["steps"].get<uint64_t>()+1;row["last_sim_time"]=ns*1e-9;row["max_depth_m"]=std::max(depth,row["max_depth_m"].get<double>());
          row["example_collision_pair"]=gz::sim::scopedName(a,ecm,"::",false)+" / "+gz::sim::scopedName(b,ecm,"::",false);if(kind=="unexpected")row["kind"]="unexpected";
        }return true;});
    }
    if(info.paused||ns-last_publish_<10000000||tcp_==gz::sim::kNullEntity)return;last_publish_=ns;
    tf2_msgs::msg::TFMessage message;
    auto append=[&](const std::string& name,Entity entity){if(entity==gz::sim::kNullEntity)return;auto pose=gz::sim::worldPose(entity,ecm);geometry_msgs::msg::TransformStamped t;t.header.stamp=rclcpp::Time(ns);t.header.frame_id="world";t.child_frame_id=name;t.transform.translation.x=pose.Pos().X();t.transform.translation.y=pose.Pos().Y();t.transform.translation.z=pose.Pos().Z();t.transform.rotation.x=pose.Rot().X();t.transform.rotation.y=pose.Rot().Y();t.transform.rotation.z=pose.Rot().Z();t.transform.rotation.w=pose.Rot().W();message.transforms.push_back(t);};
    append("tcp",tcp_);append("base_link",gz::sim::Model(robot_).LinkByName(ecm,"base_link"));for(const auto& [name,entity]:bodies_)append(name,entity);poses_->publish(message);
    std_msgs::msg::String state;state.data=attached_;latch_state_->publish(state);
    if(ns-contact_publish_ns_>=100000000){contact_publish_ns_=ns;std_msgs::msg::String report;report.data=nlohmann::json({{"physics_steps_observed",contact_steps_},{"pairs",contacts_}}).dump();contact_pub_->publish(report);}
  }
};
}
GZ_ADD_PLUGIN(mtc::SimulationSystem,gz::sim::System,mtc::SimulationSystem::ISystemConfigure,mtc::SimulationSystem::ISystemPreUpdate,mtc::SimulationSystem::ISystemPostUpdate)
