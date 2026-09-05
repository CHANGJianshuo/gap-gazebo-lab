#include <mtc_motion_planning/trajectory.hpp>
#include <iostream>
#include <random>
int main(){std::mt19937 random(20260905);std::uniform_real_distribution<double> position(-2,2),delta(-.06,.06);mtc::Limits limits;double max_error=0,max_ratio=0,min_dt=10;int bad=0;
  for(int k=0;k<1000;++k){mtc::Vec from,to;for(int j=0;j<6;++j){from[j]=position(random);to[j]=from[j]+delta(random);}auto curve=mtc::ruckigCurve(from,to,limits);if(!mtc::conditionForController(curve,limits))return 2;auto times=mtc::sampleTimes(curve);
    for(size_t i=1;i<times.size();++i){double dt=times[i]-times[i-1];if(dt<1e-6)continue;min_dt=std::min(min_dt,dt);auto a=curve.sample(times[i-1]),b=curve.sample(times[i]);for(double u:{.25,.5,.75}){auto s=mtc::quinticInterpolate(a,b,dt,u*dt),truth=curve.sample(times[i-1]+u*dt);double ratio=(s.j.cwiseAbs().array()/limits.j.array()).maxCoeff();max_ratio=std::max(max_ratio,ratio);max_error=std::max(max_error,(s.q-truth.q).cwiseAbs().maxCoeff());if(ratio>1.042){++bad;if(bad==1)std::cout<<"first_bad dt="<<dt<<" ratio="<<ratio<<"\n";}}}
  }
  std::cout<<"{\"cases\":1000,\"bad_samples\":"<<bad<<",\"max_jerk_ratio\":"<<max_ratio<<",\"max_q_error_rad\":"<<max_error<<",\"min_dt\":"<<min_dt<<"}"<<std::endl;return bad?1:0;
}
