#pragma once
#include "trajectory.hpp"
#include <moveit/robot_state/robot_state.h>
#include <urdf_model/model.h>
#include <map>

namespace mtc {
struct Payload {double mass=0;Eigen::Vector3d com=Eigen::Vector3d::Zero();Eigen::Matrix3d inertia=Eigen::Matrix3d::Zero();};
// Newton-Euler over the complete URDF tree: includes the off-axis camera and both latch halves.
class Dynamics {
  urdf::ModelInterfaceSharedPtr urdf_;
  std::array<std::string,6> joints_;
  static Eigen::Vector3d vec(const urdf::Vector3& v){return {v.x,v.y,v.z};}
  static Eigen::Matrix3d rot(const urdf::Rotation& r){return Eigen::Quaterniond(r.w,r.x,r.y,r.z).toRotationMatrix();}
  struct Wrench {Eigen::Vector3d force=Eigen::Vector3d::Zero(),moment=Eigen::Vector3d::Zero();};
  Wrench recurse(const urdf::LinkConstSharedPtr& link,const moveit::core::RobotState& state,const Eigen::Vector3d& w,const Eigen::Vector3d& alpha,const Eigen::Vector3d& acceleration,const Vec& velocity,const Vec& joint_acceleration,const Payload& payload,Vec& torque)const{
    const auto& T=state.getGlobalLinkTransform(link->name);Wrench result;
    auto addMass=[&](double mass,const Eigen::Vector3d& com,const Eigen::Matrix3d& inertia){
      Eigen::Vector3d r=T.linear()*com;Eigen::Vector3d ac=acceleration+alpha.cross(r)+w.cross(w.cross(r));Eigen::Vector3d force=mass*(ac-Eigen::Vector3d(0,0,-9.81));Eigen::Matrix3d I=T.linear()*inertia*T.linear().transpose();result.force+=force;result.moment+=I*alpha+w.cross(I*w)+r.cross(force);
    };
    if(link->inertial){const auto& in=*link->inertial;Eigen::Matrix3d I;I<<in.ixx,in.ixy,in.ixz,in.ixy,in.iyy,in.iyz,in.ixz,in.iyz,in.izz;auto R=rot(in.origin.rotation);addMass(in.mass,vec(in.origin.position),R*I*R.transpose());}
    if(link->name=="tcp"&&payload.mass>0)addMass(payload.mass,payload.com,payload.inertia);
    for(const auto& child:link->child_links){
      const auto& joint=*child->parent_joint;int index=-1;for(int i=0;i<6;++i)if(joint.name==joints_[i])index=i;double qd=index<0?0:velocity[index],qdd=index<0?0:joint_acceleration[index];
      Eigen::Vector3d axis=T.linear()*rot(joint.parent_to_joint_origin_transform.rotation)*vec(joint.axis);Eigen::Vector3d r=state.getGlobalLinkTransform(child->name).translation()-T.translation();
      Eigen::Vector3d wc=w,al=alpha,ac=acceleration+alpha.cross(r)+w.cross(w.cross(r));
      if(joint.type==urdf::Joint::REVOLUTE||joint.type==urdf::Joint::CONTINUOUS){wc+=axis*qd;al+=axis*qdd+w.cross(axis*qd);}
      if(joint.type==urdf::Joint::PRISMATIC)ac+=2*w.cross(axis*qd)+axis*qdd;
      auto child_wrench=recurse(child,state,wc,al,ac,velocity,joint_acceleration,payload,torque);
      if(index>=0)torque[index]=axis.dot(joint.type==urdf::Joint::PRISMATIC?child_wrench.force:child_wrench.moment);
      result.force+=child_wrench.force;result.moment+=child_wrench.moment+r.cross(child_wrench.force);
    }
    return result;
  }
 public:
  Dynamics(urdf::ModelInterfaceSharedPtr urdf,const std::array<std::string,6>& joints):urdf_(std::move(urdf)),joints_(joints){}
  Vec torque(const moveit::core::RobotState& state,const Vec& velocity,const Vec& acceleration,const Payload& payload={})const{Vec out=Vec::Zero();recurse(urdf_->getRoot(),state,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),velocity,acceleration,payload,out);return out;}
};
}
