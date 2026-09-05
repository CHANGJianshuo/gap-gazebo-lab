#include <mtc_motion_planning/trajectory.hpp>
#include <mtc_motion_planning/dynamics.hpp>
#include <mtc_interfaces/action/plan_motion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <thread>

namespace mtc {
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
using Action=mtc_interfaces::action::PlanMotion;
using Handle=rclcpp_action::ServerGoalHandle<Action>;
const std::array<std::string,6> JOINTS={"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& p){
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if(!q.coeffs().allFinite()||q.norm()<.5)throw std::runtime_error("Invalid target quaternion");
  Eigen::Isometry3d T=Eigen::Isometry3d::Identity();T.linear()=q.normalized().toRotationMatrix();T.translation()<<p.position.x,p.position.y,p.position.z;return T;
}
geometry_msgs::msg::Pose pose(const Eigen::Isometry3d& T){geometry_msgs::msg::Pose p;p.position.x=T.translation().x();p.position.y=T.translation().y();p.position.z=T.translation().z();Eigen::Quaterniond q(T.linear());p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;}
Vec joints(const moveit::core::RobotState& state,const moveit::core::JointModelGroup* group){std::vector<double> q;state.copyJointGroupPositions(group,q);if(q.size()!=6)throw std::runtime_error("Expected six S3 joints");return Eigen::Map<Vec>(q.data());}

class Planner : public rclcpp::Node {
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup* group_=nullptr;
  std::shared_ptr<planning_pipeline::PlanningPipeline> pipeline_;
  std::unique_ptr<Dynamics> dynamics_;
  shape_msgs::msg::Mesh battery_mesh_;
  Json config_;
  Limits limits_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr truth_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr latch_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  std::mutex input_mutex_;
  sensor_msgs::msg::JointState joint_state_;
  std::map<std::string,Eigen::Isometry3d> object_poses_;
  std::string attached_;
  Clock::time_point joint_received_=Clock::time_point::min(),truth_received_=Clock::time_point::min();
  std::atomic<bool> busy_{false},stopping_{false};
  std::thread worker_;
  struct Context {
    planning_scene::PlanningScenePtr scene;
    moveit::core::RobotState start;
    Eigen::Isometry3d goal=Eigen::Isometry3d::Identity(),start_tcp=Eigen::Isometry3d::Identity(),payload_transform=Eigen::Isometry3d::Identity();
    Payload payload;
    Limits limits;
    std::string attached,mode,touching;
    double max_line_error=0;
    Context(const moveit::core::RobotModelPtr& model):scene(std::make_shared<planning_scene::PlanningScene>(model)),start(model){}
  };
  struct Validation {bool valid=true;std::string reason;double clearance=std::numeric_limits<double>::infinity(),torque_ratio=0,line_error=0,representation_error=0;size_t samples=0;};

  void setState(moveit::core::RobotState& state,const sensor_msgs::msg::JointState& message){
    if(message.name.size()!=message.position.size())throw std::runtime_error("Joint names/positions length mismatch");
    for(size_t i=0;i<message.name.size();++i){if(!std::isfinite(message.position[i]))throw std::runtime_error("Non-finite joint state");if(model_->hasJointModel(message.name[i]))state.setVariablePosition(message.name[i],message.position[i]);}state.update();
  }
  moveit_msgs::msg::CollisionObject box(const std::string& id,const std::vector<double>& size,const Eigen::Vector3d& center){
    moveit_msgs::msg::CollisionObject o;o.id=id;o.header.frame_id="world";o.operation=o.ADD;shape_msgs::msg::SolidPrimitive b;b.type=b.BOX;b.dimensions.assign(size.begin(),size.end());o.primitives.push_back(b);geometry_msgs::msg::Pose p;p.orientation.w=1;p.position.x=center.x();p.position.y=center.y();p.position.z=center.z();o.primitive_poses.push_back(p);return o;
  }
  moveit_msgs::msg::CollisionObject battery(const std::string& id,const Eigen::Isometry3d& T){
    moveit_msgs::msg::CollisionObject o;o.id=id;o.header.frame_id="world";o.operation=o.ADD;o.meshes.push_back(battery_mesh_);o.mesh_poses.push_back(pose(T));
    for(const auto& shape:config_["battery"]["collision"]){if(shape["kind"]!="box")continue;shape_msgs::msg::SolidPrimitive b;b.type=b.BOX;auto dimensions=shape["size"].get<std::vector<double>>();b.dimensions.assign(dimensions.begin(),dimensions.end());o.primitives.push_back(b);Eigen::Isometry3d offset=Eigen::Isometry3d::Identity();auto p=shape["xyz"].get<std::vector<double>>();offset.translation()<<p[0],p[1],p[2];o.primitive_poses.push_back(pose(T*offset));}return o;
  }
  Context context(const Action::Goal& goal){
    Context c(model_);sensor_msgs::msg::JointState current;std::map<std::string,Eigen::Isometry3d> objects;
    {std::lock_guard<std::mutex> lock(input_mutex_);if(joint_state_.position.empty()||Clock::now()-joint_received_>std::chrono::seconds(2))throw std::runtime_error("STALE_JOINT_STATE");if(object_poses_.size()<4||Clock::now()-truth_received_>std::chrono::seconds(2))throw std::runtime_error("STALE_GROUND_TRUTH");current=joint_state_;objects=object_poses_;c.attached=attached_;}
    c.start.setToDefaultValues();setState(c.start,current);if(!goal.start_state.position.empty())setState(c.start,goal.start_state);c.scene->setCurrentState(c.start);
    c.start_tcp=c.start.getGlobalLinkTransform("tcp");c.mode=goal.mode;c.touching=goal.touching_object;c.limits=limits_;
    double vs=goal.velocity_scaling>0?std::min(1.,goal.velocity_scaling):1.;double as=goal.acceleration_scaling>0?std::min(1.,goal.acceleration_scaling):1.;c.limits.v*=vs;c.limits.a*=as;c.limits.j*=as;
    c.scene->processCollisionObjectMsg(box("table",{.68,.86,.05},{.17,.043,.625}));c.scene->processCollisionObjectMsg(box("floor",{10.,10.,.05},{0,0,-.05}));auto& acm=c.scene->getAllowedCollisionMatrixNonConst();acm.setEntry("base_link","table",true);
    for(const auto& [name,T]:objects){if(name.rfind("battery_",0)!=0)continue;auto object=battery(name,T);c.scene->processCollisionObjectMsg(object);
      if(name==c.attached){moveit_msgs::msg::AttachedCollisionObject attached;attached.link_name="tcp";attached.object=object;attached.touch_links={"latch_left","latch_right"};if(!c.scene->processAttachedCollisionObjectMsg(attached))throw std::runtime_error("Could not attach planning payload");c.payload_transform=c.start_tcp.inverse()*T;c.payload.mass=config_["battery"]["mass"].get<double>();auto com=config_["battery"]["center_of_mass"].get<std::vector<double>>();c.payload.com=c.payload_transform*Eigen::Vector3d(com[0],com[1],com[2]);Eigen::Matrix3d I;for(int i=0;i<3;++i)for(int j=0;j<3;++j)I(i,j)=config_["battery"]["inertia"][i][j];c.payload.inertia=c.payload_transform.linear()*I*c.payload_transform.linear().transpose();}
      if(name==goal.touching_object){acm.setEntry("latch_left",name,true);acm.setEntry("latch_right",name,true);}
    }
    if(!c.attached.empty()&&c.mode=="cartesian"&&goal.touching_object==c.attached)acm.setEntry(c.attached,"table",true);
    c.start=c.scene->getCurrentState();
    if(c.mode=="joint"){auto end=c.start;if(goal.goal_state.position.empty())throw std::runtime_error("Joint goal is empty");setState(end,goal.goal_state);c.goal=end.getGlobalLinkTransform("tcp");}
    else {c.goal=transform(goal.target_pose.pose);if(goal.target_pose.header.frame_id=="base_link")c.goal=c.start.getGlobalLinkTransform("base_link")*c.goal;else if(goal.target_pose.header.frame_id!="world")throw std::runtime_error("Target frame must be world or base_link");}
    return c;
  }
  bool collisionFree(Context& c,moveit::core::RobotState& state,std::string* reason=nullptr,double* distance=nullptr){
    if(!state.satisfiesBounds(group_,1e-7)){if(reason)*reason="JOINT_LIMIT";return false;}
    collision_detection::CollisionRequest request;collision_detection::CollisionResult result;request.contacts=reason!=nullptr;request.max_contacts=2;request.distance=distance!=nullptr;c.scene->checkCollision(request,result,state);
    if(distance)*distance=result.distance;
    if(result.collision){if(reason){*reason="COLLISION";for(const auto& pair:result.contacts){*reason+=" "+pair.first.first+" / "+pair.first.second;break;}}return false;}
    if(c.payload.mass>0){auto T=state.getGlobalLinkTransform("tcp")*c.payload_transform;double low=1e6;for(double x:{-.035,.035})for(double y:{-.040,.040})low=std::min(low,(T*Eigen::Vector3d(x,y,0)).z());if(low<.648){if(reason)*reason="PAYLOAD_BELOW_TABLE";return false;}if(T.linear().col(2).z()<std::cos(.35)){if(reason)*reason="PAYLOAD_TILT";return false;}}
    return true;
  }
  bool taskConstraints(Context& c,const moveit::core::RobotState& state,double& error){
    if(c.mode!="cartesian")return true;const auto& T=state.getGlobalLinkTransform("tcp");Eigen::Vector3d d=c.goal.translation()-c.start_tcp.translation();double u=d.squaredNorm()>1e-12?(T.translation()-c.start_tcp.translation()).dot(d)/d.squaredNorm():0;u=std::clamp(u,0.,1.);error=(T.translation()-(c.start_tcp.translation()+u*d)).norm();Eigen::Quaterniond expected=Eigen::Quaterniond(c.start_tcp.linear()).slerp(u,Eigen::Quaterniond(c.goal.linear()));double angular=expected.angularDistance(Eigen::Quaterniond(T.linear()));return error<=.0015&&angular<=.018;
  }
  Validation validate(Context& c,const Curve& curve,bool final,const std::shared_ptr<Handle>& handle){
    Validation out;auto state=c.start;double t=0;Vec previous=curve.sample(0).q;
    while(true){if(stopping_||handle->is_canceling()){out.valid=false;out.reason="CANCELED";return out;}auto sample=curve.sample(t);if(!sample.q.allFinite()||!sample.v.allFinite()||!sample.a.allFinite()){out.valid=false;out.reason="NONFINITE_TRAJECTORY";return out;}state.setJointGroupPositions(group_,sample.q.data());state.update();double distance=0;
      if(!collisionFree(c,state,&out.reason,final?&distance:nullptr)){out.valid=false;return out;}if(final)out.clearance=std::min(out.clearance,distance);
      double error=0;if(!taskConstraints(c,state,error)){out.valid=false;out.reason="CARTESIAN_PATH_TOLERANCE";return out;}out.line_error=std::max(out.line_error,error);
      if((sample.v.cwiseAbs().array()>c.limits.v.array()*1.0001).any()||(sample.a.cwiseAbs().array()>c.limits.a.array()*1.0001).any()||(sample.j.cwiseAbs().array()>c.limits.j.array()*1.0001).any()){out.valid=false;out.reason="DERIVATIVE_LIMIT";return out;}
      Vec tau=dynamics_->torque(state,sample.v,sample.a,c.payload);out.torque_ratio=std::max(out.torque_ratio,(tau.cwiseAbs().array()/limits_.effort.array()).maxCoeff());++out.samples;
      if(t>=curve.duration-1e-10)break;double dt=final?.004:.014;double speed=sample.v.cwiseAbs().maxCoeff();if(final)dt=std::min(dt,.003/std::max(.1,speed));t=std::min(curve.duration,t+dt);previous=sample.q;
    }
    if(final){auto times=sampleTimes(curve);for(size_t i=1;i<times.size();++i){double dt=times[i]-times[i-1];if(dt<1e-6)continue;auto a=curve.sample(times[i-1]),b=curve.sample(times[i]);for(double u:{.25,.5,.75}){auto represented=quinticInterpolate(a,b,dt,u*dt),truth=curve.sample(times[i-1]+u*dt);out.representation_error=std::max(out.representation_error,(truth.q-represented.q).cwiseAbs().maxCoeff());if((represented.v.cwiseAbs().array()>c.limits.v.array()*1.002).any()||(represented.a.cwiseAbs().array()>c.limits.a.array()*1.002).any()||(represented.j.cwiseAbs().array()>c.limits.j.array()*1.04+.05).any()){out.valid=false;out.reason="CONTROLLER_INTERPOLATION_LIMIT";return out;}}}if(out.representation_error>1e-7){out.valid=false;out.reason="CONTROLLER_REPRESENTATION_ERROR";}}
    return out;
  }
  std::vector<Vec> inverseGoals(Context& c,const Action::Goal& goal,const Clock::time_point& deadline){
    if(c.mode=="joint"){auto end=c.start;setState(end,goal.goal_state);return {joints(end,group_)};}
    std::vector<Vec> out;std::mt19937 rng(goal.seed?goal.seed:20260905);std::normal_distribution<double> near(0,1.2);std::uniform_real_distribution<double> global(-3.0,3.0);Vec initial=joints(c.start,group_);
    for(int attempt=0;attempt<24&&Clock::now()<deadline&&out.size()<6;++attempt){auto state=c.start;Vec seed=initial;if(attempt>0)for(int i=0;i<6;++i)seed[i]=attempt<12?initial[i]+near(rng):global(rng);state.setJointGroupPositions(group_,seed.data());state.enforceBounds(group_);state.update();
      auto valid=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup* g,const double* q){s->setJointGroupPositions(g,q);s->update();return collisionFree(c,*s);};
      if(!state.setFromIK(group_,c.goal,"tcp",.025,valid))continue;Vec q=joints(state,group_);
      for(int i=0;i<6;++i){const auto& bound=model_->getVariableBounds(JOINTS[i]);while(q[i]-initial[i]>M_PI&&q[i]-2*M_PI>=bound.min_position_)q[i]-=2*M_PI;while(q[i]-initial[i]<-M_PI&&q[i]+2*M_PI<=bound.max_position_)q[i]+=2*M_PI;}
      state.setJointGroupPositions(group_,q.data());state.update();if(!collisionFree(c,state))continue;bool duplicate=false;for(const auto& old:out)if((q-old).norm()<.03)duplicate=true;if(!duplicate)out.push_back(q);
    }
    std::sort(out.begin(),out.end(),[&](const Vec& a,const Vec& b){return ((a-initial).array()/c.limits.v.array()).matrix().norm()<((b-initial).array()/c.limits.v.array()).matrix().norm();});return out;
  }
  std::vector<Vec> cartesianPath(Context& c,const std::shared_ptr<Handle>& handle){
    auto state=c.start;std::vector<Vec> path{joints(state,group_)};double length=(c.goal.translation()-c.start_tcp.translation()).norm();double angle=Eigen::Quaterniond(c.start_tcp.linear()).angularDistance(Eigen::Quaterniond(c.goal.linear()));int count=std::max(1,int(std::ceil(std::max(length/.004,angle/.02))));
    for(int i=1;i<=count;++i){if(handle->is_canceling()||stopping_)throw std::runtime_error("CANCELED");double u=double(i)/count;Eigen::Isometry3d target=Eigen::Isometry3d::Identity();target.translation()=(1-u)*c.start_tcp.translation()+u*c.goal.translation();target.linear()=Eigen::Quaterniond(c.start_tcp.linear()).slerp(u,Eigen::Quaterniond(c.goal.linear())).toRotationMatrix();auto valid=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup* g,const double* q){s->setJointGroupPositions(g,q);s->update();return collisionFree(c,*s);};if(!state.setFromIK(group_,target,"tcp",.035,valid))throw std::runtime_error("CARTESIAN_IK_FAILED at "+std::to_string(i)+"/"+std::to_string(count));Vec q=joints(state,group_);if((q-path.back()).cwiseAbs().maxCoeff()>.22)throw std::runtime_error("CARTESIAN_JOINT_JUMP");path.push_back(q);}
    return path;
  }
  std::vector<Vec> rrtPath(Context& c,const Vec& goal,double seconds){
    moveit_msgs::msg::MotionPlanRequest request;request.group_name="arm";request.planner_id="RRTConnectkConfigDefault";request.allowed_planning_time=seconds;request.num_planning_attempts=1;moveit::core::robotStateToRobotStateMsg(c.start,request.start_state);auto end=c.start;end.setJointGroupPositions(group_,goal.data());end.update();request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(end,group_,1e-5));planning_interface::MotionPlanResponse response;std::vector<Vec> path;if(pipeline_->generatePlan(c.scene,request,response)&&response.trajectory_)for(size_t i=0;i<response.trajectory_->getWayPointCount();++i)path.push_back(joints(response.trajectory_->getWayPoint(i),group_));return path;
  }
  void execute(const std::shared_ptr<Handle>& handle){
    auto result=std::make_shared<Action::Result>();const auto started=Clock::now();Json report;report["candidates"]=Json::array();report["scope"]="fastest validated candidate; no global obstacle-constrained optimum claim";
    try {
      const auto& goal=*handle->get_goal();auto c=context(goal);std::string start_error;if(!collisionFree(c,c.start,&start_error))throw std::runtime_error("START_"+start_error);
      double budget=goal.planning_timeout>0?std::clamp(goal.planning_timeout,.2,30.):8.;auto deadline=started+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget));Vec from=joints(c.start,group_);std::vector<Curve> accepted;
      auto feedback=[&](const std::string& stage){auto f=std::make_shared<Action::Feedback>();f->stage=stage;f->candidates_evaluated=report["candidates"].size();handle->publish_feedback(f);};
      auto evaluate=[&](Curve curve){Json row;row["algorithm"]=curve.name;double original=curve.duration;Validation check;
        for(int scaling=0;scaling<9;++scaling){if(!conditionForController(curve,c.limits)){check.valid=false;check.reason="CONTROLLER_CONDITIONING_FAILED";break;}check=validate(c,curve,false,handle);if(!check.valid||check.torque_ratio<=.82)break;curve.scaleTime(1.15);}
        row["duration"]=curve.duration;row["time_scale"]=curve.duration/std::max(original,1e-9);row["torque_ratio"]=check.torque_ratio;row["valid"]=check.valid&&check.torque_ratio<=.82;row["reason"]=check.valid?(check.torque_ratio<=.82?"ok":"TORQUE_LIMIT"):check.reason;report["candidates"].push_back(row);if(check.valid&&check.torque_ratio<=.82)accepted.push_back(std::move(curve));feedback("candidate_validation");};
      feedback("kinematics_and_geometry");
      if(c.mode=="cartesian"){
        auto path=cartesianPath(c,handle);report["cartesian_waypoints"]=path.size();for(int controls:{10,16,24}){if(Clock::now()>deadline&&!accepted.empty())break;evaluate(splineCurve(path,controls,c.limits));}
      }else{
        auto goals=inverseGoals(c,goal,deadline);report["ik_solutions"]=goals.size();if(goals.empty())throw std::runtime_error("NO_VALID_IK_SOLUTION");
        for(const auto& q:goals){if(Clock::now()>deadline&&!accepted.empty())break;try{evaluate(ruckigCurve(from,q,c.limits));}catch(const std::exception& e){report["candidates"].push_back({{"algorithm","ruckig_direct"},{"valid",false},{"reason",e.what()}});}}
        // A Cartesian transfer is also a useful low-tilt candidate with a rigidly captured battery.
        if(c.payload.mass>0||accepted.empty())try{auto path=cartesianPath(c,handle);for(int controls:{10,16})evaluate(splineCurve(path,controls,c.limits));}catch(const std::exception& e){report["cartesian_alternative"]=e.what();}
        if(accepted.empty())for(size_t i=0;i<std::min<size_t>(goals.size(),3);++i){if(Clock::now()>deadline)break;feedback("rrt_connect");double remaining=std::chrono::duration<double>(deadline-Clock::now()).count();auto path=rrtPath(c,goals[i],std::min(1.2,remaining));if(path.size()<2)continue;for(int controls:{12,20,32})evaluate(splineCurve(path,controls,c.limits));}
      }
      if(accepted.empty())throw std::runtime_error("NO_VALID_TRAJECTORY");std::sort(accepted.begin(),accepted.end(),[](const Curve& a,const Curve& b){return a.duration<b.duration;});
      feedback("final_collision_dynamics_and_controller_check");Curve winner;Validation final;bool found=false;
      for(auto& candidate:accepted){final=validate(c,candidate,true,handle);if(final.valid&&final.torque_ratio<=.84){winner=std::move(candidate);found=true;break;}report["final_rejections"].push_back({{"algorithm",candidate.name},{"reason",final.reason}});}
      if(!found)throw std::runtime_error("FINAL_VALIDATION_FAILED");
      auto state=c.start;auto& trajectory=result->trajectory.joint_trajectory;trajectory.joint_names.assign(JOINTS.begin(),JOINTS.end());trajectory.header.frame_id="world";
      for(double t:sampleTimes(winner)){auto sample=winner.sample(t);state.setJointGroupPositions(group_,sample.q.data());state.update();Vec effort=dynamics_->torque(state,sample.v,sample.a,c.payload);trajectory_msgs::msg::JointTrajectoryPoint point;point.positions.assign(sample.q.data(),sample.q.data()+6);point.velocities.assign(sample.v.data(),sample.v.data()+6);point.accelerations.assign(sample.a.data(),sample.a.data()+6);point.effort.assign(effort.data(),effort.data()+6);point.time_from_start=rclcpp::Duration::from_seconds(t);trajectory.points.push_back(point);}
      if(trajectory.points.size()<2)throw std::runtime_error("DEGENERATE_TRAJECTORY");
      report["selected"]=winner.name;report["duration"]=winner.duration;report["validation_samples"]=final.samples;report["collision_step_s_max"]=.004;report["collision_joint_step_rad_target"]=.003;report["collision_validation"]="adaptive discrete FCL, whole robot and payload; finite resolution, not a formal continuous proof";report["max_cartesian_line_error_m"]=final.line_error;report["controller_representation_error_rad"]=final.representation_error;report["payload_mass_kg"]=c.payload.mass;report["maximum_torque_ratio"]=final.torque_ratio;report["minimum_clearance_m"]=final.clearance;
      result->success=true;result->reason="VALIDATED";result->duration=winner.duration;result->minimum_clearance=std::isfinite(final.clearance)?final.clearance:0;result->maximum_torque_ratio=final.torque_ratio;
      moveit_msgs::msg::DisplayTrajectory display;display.model_id=model_->getName();moveit::core::robotStateToRobotStateMsg(c.start,display.trajectory_start);display.trajectory.push_back(result->trajectory);display_->publish(display);
    }catch(const std::exception& e){result->success=false;result->reason=e.what();}
    result->planning_time=std::chrono::duration<double>(Clock::now()-started).count();report["planning_time"]=result->planning_time;report["success"]=result->success;report["reason"]=result->reason;result->diagnostics_json=report.dump();std_msgs::msg::String message;message.data=result->diagnostics_json;diagnostics_->publish(message);
    if(handle->is_canceling())handle->canceled(result);else if(result->success)handle->succeed(result);else handle->abort(result);
    RCLCPP_INFO(get_logger(),"Plan %s: %s, compute %.3fs, execution %.3fs",result->success?"PASS":"FAIL",result->reason.c_str(),result->planning_time,result->duration);busy_=false;
  }
 public:
  explicit Planner(const rclcpp::NodeOptions& options):Node("mtc_motion_planner",options){}
  ~Planner()override{stopping_=true;if(worker_.joinable())worker_.join();}
  void initialize(){
    std::ifstream file(get_parameter("scene_config").as_string());file>>config_;
    loader_=std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(),"robot_description",true);model_=loader_->getModel();if(!model_)throw std::runtime_error("Robot model unavailable");group_=model_->getJointModelGroup("arm");if(!group_||group_->getVariableCount()!=6)throw std::runtime_error("Invalid S3 arm group");
    for(const auto& name:{"base_link","shoulder_Link","upperArm_Link","foreArm_Link","wrist1_Link","wrist2_Link","wrist3_Link","latch_left","latch_right"})if(model_->getLinkModel(name)->getShapes().empty())throw std::runtime_error(std::string("Missing collision geometry: ")+name);
    for(int i=0;i<6;++i){std::string prefix="robot_description_planning.joint_limits."+JOINTS[i]+".";limits_.v[i]=get_parameter(prefix+"max_velocity").as_double();limits_.a[i]=get_parameter(prefix+"max_acceleration").as_double();limits_.j[i]=get_parameter(prefix+"max_jerk").as_double();auto hardware=model_->getURDF()->getJoint(JOINTS[i])->limits;limits_.v[i]=std::min(limits_.v[i],hardware->velocity);limits_.effort[i]=hardware->effort;if(!std::isfinite(limits_.v[i]+limits_.a[i]+limits_.j[i])||limits_.v[i]<=0||limits_.a[i]<=0||limits_.j[i]<=0)throw std::runtime_error("Invalid configured motion limits");}
    dynamics_=std::make_unique<Dynamics>(model_->getURDF(),JOINTS);
    pipeline_=std::make_shared<planning_pipeline::PlanningPipeline>(model_,shared_from_this(),"ompl","ompl_interface/OMPLPlanner",std::vector<std::string>{});pipeline_->displayComputedMotionPlans(false);
    auto mesh=std::unique_ptr<shapes::Mesh>(shapes::createMeshFromResource("file://"+config_["battery"]["collision"][0]["file"].get<std::string>()));if(!mesh)throw std::runtime_error("Battery collision mesh missing");shapes::ShapeMsg shape;shapes::constructMsgFromShape(mesh.get(),shape);battery_mesh_=boost::get<shape_msgs::msg::Mesh>(shape);
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(input_mutex_);joint_state_=*msg;joint_received_=Clock::now();});
    truth_sub_=create_subscription<tf2_msgs::msg::TFMessage>("/simulation/ground_truth",rclcpp::QoS(5).reliable(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(input_mutex_);for(const auto& t:msg->transforms){geometry_msgs::msg::Pose p;p.position.x=t.transform.translation.x;p.position.y=t.transform.translation.y;p.position.z=t.transform.translation.z;p.orientation=t.transform.rotation;object_poses_[t.child_frame_id]=transform(p);}truth_received_=Clock::now();});
    latch_sub_=create_subscription<std_msgs::msg::String>("/simulation/latch_state",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(input_mutex_);attached_=msg->data;});
    diagnostics_=create_publisher<std_msgs::msg::String>("/planning/diagnostics",rclcpp::QoS(5).reliable());display_=create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",rclcpp::QoS(1).reliable().transient_local());
    action_=rclcpp_action::create_server<Action>(shared_from_this(),"/motion_planning/plan",[this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal> goal){if(goal->mode!="free"&&goal->mode!="cartesian"&&goal->mode!="joint")return rclcpp_action::GoalResponse::REJECT;bool expected=false;if(!busy_.compare_exchange_strong(expected,true))return rclcpp_action::GoalResponse::REJECT;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](std::shared_ptr<Handle>){return rclcpp_action::CancelResponse::ACCEPT;},[this](std::shared_ptr<Handle> handle){if(worker_.joinable())worker_.join();worker_=std::thread([this,handle](){execute(handle);});});
    RCLCPP_INFO(get_logger(),"S3 planner ready: multi-seed IK, Ruckig, quintic B-spline, RRT-Connect fallback, full-tree dynamics and FCL");
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto options=rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);auto node=std::make_shared<mtc::Planner>(options);node->initialize();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);executor.add_node(node);executor.spin();}catch(const std::exception& e){std::cerr<<"Planner startup failed: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
