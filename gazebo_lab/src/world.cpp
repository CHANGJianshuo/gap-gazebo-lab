#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <future>
namespace gaplab {
class World : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 using Entity=gz::sim::Entity;
 struct Command {bool close; std::promise<std::pair<bool,std::string>> result;};
 rclcpp::Node::SharedPtr node;
 std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
 std::thread thread;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
 rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv;
 std::mutex mutex;
 std::shared_ptr<Command> pending;
 Entity robot=0,hand=0,cube=0,attachment=0;
 bool contacts=false; int64_t last=-1000000000;
 public:
 ~World() override {if(exec)exec->cancel();if(thread.joinable())thread.join();}
 void Configure(const Entity&,const std::shared_ptr<const sdf::Element>&,gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);}
  exec=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  node=std::make_shared<rclcpp::Node>("gap_ground_truth");pub=node->create_publisher<std_msgs::msg::String>("/gap/world",10);
  srv=node->create_service<std_srvs::srv::SetBool>("/gap/attach",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res){auto c=std::make_shared<Command>();c->close=req->data;auto f=c->result.get_future();{std::lock_guard<std::mutex> l(mutex);if(pending){res->success=false;res->message="busy";return;}pending=c;}
   if(f.wait_for(std::chrono::seconds(3))!=std::future_status::ready){std::lock_guard<std::mutex> l(mutex);if(pending==c)pending.reset();res->success=false;res->message="paused/timeout";return;}auto v=f.get();res->success=v.first;res->message=v.second;});
  exec->add_node(node);thread=std::thread([this]{exec->spin();});
 }
 void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
  if(info.paused)return;
  if(!hand){robot=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("panda"));if(robot)hand=gz::sim::Model(robot).LinkByName(ecm,"panda_hand");}
  if(!cube){auto m=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("cube"));if(m)cube=gz::sim::Model(m).LinkByName(ecm,"body");}
  if(hand&&!contacts){std::vector<Entity> es;ecm.Each<gz::sim::components::Collision>([&](Entity e,const auto*){es.push_back(e);return true;});for(auto e:es)if(!ecm.Component<gz::sim::components::ContactSensorData>(e))ecm.CreateComponent(e,gz::sim::components::ContactSensorData());contacts=true;}
  std::shared_ptr<Command> c;{std::lock_guard<std::mutex> l(mutex);c=pending;pending.reset();}if(!c)return;
  bool ok=false;std::string reason;
  if(!hand||!cube)reason="entities unavailable";
  else if(!c->close){if(attachment)ecm.RequestRemoveEntity(attachment);attachment=0;ok=true;reason="released";}
  else if(attachment){ok=true;reason="already attached";}
  else {auto hp=gz::sim::worldPose(hand,ecm);auto cp=gz::sim::worldPose(cube,ecm);auto tcp=hp.Pos()+hp.Rot().RotateVector(gz::math::Vector3d(0,0,.1034));
   auto fj=gz::sim::Model(robot).JointByName(ecm,"panda_finger_joint1");auto q=ecm.Component<gz::sim::components::JointPosition>(fj);
   if(tcp.Distance(cp.Pos())>.016)reason="TCP must be within 16 mm of cube center";
   else if(!q||q->Data().empty()||q->Data()[0]>.026)reason="gripper must be closed";
   else{attachment=ecm.CreateEntity();ecm.CreateComponent(attachment,gz::sim::components::DetachableJoint({hand,cube,"fixed"}));ok=true;reason="position-gated fixed grasp constraint; measured relative pose retained";}}
  c->result.set_value({ok,reason});
 }
 void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& ecm) override {
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();if(info.paused||!hand||!cube||ns-last<50000000)return;last=ns;
  auto hp=gz::sim::worldPose(hand,ecm);auto cp=gz::sim::worldPose(cube,ecm);auto tp=hp.Pos()+hp.Rot().RotateVector(gz::math::Vector3d(0,0,.1034));
  nlohmann::json j={{"sim_time",ns/1e9},{"tcp",{tp.X(),tp.Y(),tp.Z()}},{"cube",{cp.Pos().X(),cp.Pos().Y(),cp.Pos().Z()}},{"attached",attachment!=0},{"target",{.5,.24,.17}},{"contacts",nlohmann::json::array()}};
  ecm.Each<gz::sim::components::ContactSensorData>([&](Entity,const auto* cs){for(int i=0;i<cs->Data().contact_size();++i){auto& c=cs->Data().contact(i);std::string a=c.collision1().name(),b=c.collision2().name();if(a.empty())a=gz::sim::scopedName(c.collision1().id(),ecm,"/");if(b.empty())b=gz::sim::scopedName(c.collision2().id(),ecm,"/");double d=0;for(int k=0;k<c.depth_size();++k)d=std::max(d,c.depth(k));if(d>.001)j["contacts"].push_back({{"a",a},{"b",b},{"depth",d}});}return true;});
  std_msgs::msg::String msg;msg.data=j.dump();pub->publish(msg);
 }
};}
GZ_ADD_PLUGIN(gaplab::World,gz::sim::System,gaplab::World::ISystemConfigure,gaplab::World::ISystemPreUpdate,gaplab::World::ISystemPostUpdate)
