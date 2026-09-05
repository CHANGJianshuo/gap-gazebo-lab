#include <mtc_motion_planning/trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <iostream>
#include <random>
int main(){
  mtc::Vec from=mtc::Vec::Zero(),to;to<<.9,-.3,.6,.7,-.2,.4;
  std::vector<mtc::Curve> curves={mtc::ruckigCurve(from,to,{}),mtc::splineCurve({from,to},16,{})};
  std::mt19937 random(20260905);std::uniform_real_distribution<double> position(-2.,2.),delta(-.06,.06);
  for(int i=0;i<200;++i){for(int j=0;j<6;++j){from[j]=position(random);to[j]=from[j]+delta(random);}auto c=mtc::ruckigCurve(from,to,{});if(!mtc::conditionForController(c,{}))throw std::runtime_error("Controller conditioning failed");curves.push_back(c);}
  joint_trajectory_controller::Trajectory controller;double maximum=0,effort_error=0;size_t checked=0;
  for(const auto& curve:curves){auto times=mtc::sampleTimes(curve);for(size_t i=1;i<times.size();++i){
      auto ta=rclcpp::Time(int64_t(times[i-1]*1e9)),tb=rclcpp::Time(int64_t(times[i]*1e9));
      if((tb-ta).nanoseconds()<1000)continue;
      auto a=curve.sample(times[i-1]),b=curve.sample(times[i]);trajectory_msgs::msg::JointTrajectoryPoint pa,pb,out;
      pa.positions.assign(a.q.data(),a.q.data()+6);pa.velocities.assign(a.v.data(),a.v.data()+6);pa.accelerations.assign(a.a.data(),a.a.data()+6);pa.effort=std::vector<double>(6,2.);
      pb.positions.assign(b.q.data(),b.q.data()+6);pb.velocities.assign(b.v.data(),b.v.data()+6);pb.accelerations.assign(b.a.data(),b.a.data()+6);pb.effort=std::vector<double>(6,6.);
      for(double u:{.25,.5,.75}){auto stamp=ta+rclcpp::Duration::from_nanoseconds(int64_t((tb-ta).nanoseconds()*u));controller.interpolate_between_points(ta,pa,tb,pb,stamp,out);auto truth=curve.sample(stamp.seconds());double blend=double((stamp-ta).nanoseconds())/(tb-ta).nanoseconds();
        if(out.effort.size()!=6)throw std::runtime_error("Patched JTC is not loaded");
        for(int j=0;j<6;++j){maximum=std::max(maximum,std::abs(out.positions[j]-truth.q[j]));effort_error=std::max(effort_error,std::abs(out.effort[j]-(2+4*blend)));}++checked;
      }
      pa.effort.clear();pb.effort.clear();controller.interpolate_between_points(ta,pa,tb,pb,ta,out);if(!out.effort.empty())throw std::runtime_error("Stale effort retained");
    }
  }
  std::cout<<"{\"pass\":"<<(maximum<1e-7&&effort_error<1e-10?"true":"false")<<",\"samples\":"<<checked<<",\"max_q_error_rad\":"<<maximum<<",\"max_effort_interpolation_error_Nm\":"<<effort_error<<"}"<<std::endl;
  return maximum<1e-7&&effort_error<1e-10?0:1;
}
