#include <mtc_motion_planning/trajectory.hpp>
#include <iostream>
#include <cassert>
int main(){
  mtc::Limits limits;mtc::Vec a=mtc::Vec::Zero(),b; b<<.8,-1.,.4,.6,-.3,1.2;
  std::vector<mtc::Vec> path={a,(a+b)*.3,(a+b)*.7,b};
  for(auto curve:{mtc::splineCurve(path,14,limits),mtc::ruckigCurve(a,b,limits)}){
    auto start=curve.sample(0),end=curve.sample(curve.duration);
    if((start.q-a).norm()>1e-9||(end.q-b).norm()>1e-9||start.v.norm()>1e-8||end.v.norm()>1e-8||start.a.norm()>1e-8||end.a.norm()>1e-8)throw std::runtime_error("Endpoint continuity failed");
    auto times=mtc::sampleTimes(curve,.013);double worst=0;
    for(size_t i=1;i<times.size();++i){double dt=times[i]-times[i-1];auto x=curve.sample(times[i-1]),y=curve.sample(times[i]);for(double u:{.17,.43,.79}){auto truth=curve.sample(times[i-1]+u*dt);auto interp=mtc::quinticInterpolate(x,y,dt,u*dt);worst=std::max(worst,(interp.q-truth.q).norm());if((interp.v.cwiseAbs().array()>limits.v.array()+1e-6).any()||(interp.a.cwiseAbs().array()>limits.a.array()+1e-5).any()||(interp.j.cwiseAbs().array()>limits.j.array()+.01).any())throw std::runtime_error("Interpolated derivative limit failed");}}
    if(worst>1e-9)throw std::runtime_error("Controller interpolation changed the source curve");
    std::cout<<curve.name<<" duration="<<curve.duration<<" max_interpolation_error="<<worst<<" PASS\n";
  }
  return 0;
}
