#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mtc {
using Vec=Eigen::Matrix<double,6,1>;
struct Sample {Vec q=Vec::Zero(),v=Vec::Zero(),a=Vec::Zero(),j=Vec::Zero();};
struct Limits {
  Vec v=(Vec()<<2.3,2.3,2.3,2.8,2.8,2.8).finished();
  Vec a=(Vec()<<4,4,4,6,6,6).finished();
  Vec j=(Vec()<<28,28,28,45,45,45).finished();
  Vec effort=(Vec()<<49,49,39,9.8,9.8,9.8).finished();
};
struct Curve {
  std::string name;
  double duration=0;
  std::vector<double> breaks;
  std::function<Sample(double)> sample;
  void scaleTime(double factor){
    if(factor<1||!std::isfinite(factor))throw std::invalid_argument("Invalid time scale");
    auto old=sample;sample=[old,factor](double t){auto s=old(t/factor);s.v/=factor;s.a/=(factor*factor);s.j/=(factor*factor*factor);return s;};
    duration*=factor;for(auto& t:breaks)t*=factor;
  }
};

// Clamped quintic B-spline. Repeated endpoint controls impose zero endpoint v/a.
class BSpline {
  struct Level {int degree;std::vector<Vec> controls;std::vector<double> knots;};
  std::array<Level,4> levels_;
  static Vec evaluate(const Level& level,double u){
    const auto& c=level.controls;const auto& k=level.knots;const int p=level.degree,n=int(c.size())-1;
    if(u<=0)return c.front();if(u>=1)return c.back();
    int span=int(std::upper_bound(k.begin(),k.end(),u)-k.begin())-1;span=std::clamp(span,p,n);
    std::array<Vec,6> d;for(int j=0;j<=p;++j)d[j]=c[span-p+j];
    for(int r=1;r<=p;++r)for(int j=p;j>=r;--j){int i=span-p+j;double den=k[i+p-r+1]-k[i];double alpha=den>1e-14?(u-k[i])/den:0;d[j]=(1-alpha)*d[j-1]+alpha*d[j];}
    return d[p];
  }
 public:
  explicit BSpline(const std::vector<Vec>& controls){
    if(controls.size()<6)throw std::invalid_argument("Quintic spline needs at least six controls");
    Level first;first.degree=5;first.controls=controls;int m=int(controls.size());
    for(int i=0;i<m+6;++i)first.knots.push_back(i<=5?0:i>=m?1:double(i-5)/(m-5));levels_[0]=first;
    for(int order=1;order<=3;++order){const auto& prev=levels_[order-1];auto& next=levels_[order];next.degree=prev.degree-1;next.knots=std::vector<double>(prev.knots.begin()+1,prev.knots.end()-1);for(size_t i=0;i+1<prev.controls.size();++i){double den=prev.knots[i+prev.degree+1]-prev.knots[i+1];next.controls.push_back(den>1e-14?Vec(prev.degree*(prev.controls[i+1]-prev.controls[i])/den):Vec::Zero());}}
  }
  Vec at(double u,int derivative=0)const{return evaluate(levels_.at(derivative),std::clamp(u,0.,1.));}
  std::vector<double> knots()const{auto k=levels_[0].knots;k.erase(std::unique(k.begin(),k.end()),k.end());return k;}
  Vec derivativeControlBound(int derivative)const{Vec bound=Vec::Zero();for(const auto& q:levels_.at(derivative).controls)bound=bound.cwiseMax(q.cwiseAbs());return bound;}
};

inline std::vector<Vec> resamplePath(const std::vector<Vec>& path,int count){
  if(path.empty())throw std::invalid_argument("Empty path");if(path.size()==1)return std::vector<Vec>(count,path.front());
  std::vector<double> arc{0};for(size_t i=1;i<path.size();++i)arc.push_back(arc.back()+(path[i]-path[i-1]).norm());
  std::vector<Vec> out;for(int i=0;i<count;++i){double s=arc.back()*i/(count-1);auto it=std::lower_bound(arc.begin(),arc.end(),s);size_t b=std::clamp<size_t>(it-arc.begin(),1,path.size()-1);double d=arc[b]-arc[b-1];double u=d>1e-12?(s-arc[b-1])/d:0;out.push_back((1-u)*path[b-1]+u*path[b]);}return out;
}

inline Curve splineCurve(const std::vector<Vec>& path,int control_count,const Limits& limits){
  control_count=std::max(8,control_count);std::vector<Vec> controls(control_count,Vec::Zero());
  for(int i=0;i<3;++i){controls[i]=path.front();controls[control_count-1-i]=path.back();}
  const int unknown=control_count-6;BSpline fixed(controls);std::vector<double> u(unknown);std::vector<double> arc{0};
  for(size_t i=1;i<path.size();++i)arc.push_back(arc.back()+(path[i]-path[i-1]).norm());
  for(int i=0;i<unknown;++i){double sum=0;for(int j=1;j<=5;++j)sum+=std::clamp(double(i+3+j-5)/(control_count-5),0.,1.);u[i]=sum/5.;}
  Eigen::MatrixXd matrix(unknown,unknown),rhs(unknown,6);
  for(int row=0;row<unknown;++row){double s=u[row],progress=10*s*s*s-15*std::pow(s,4)+6*std::pow(s,5),distance=progress*arc.back();auto it=std::lower_bound(arc.begin(),arc.end(),distance);size_t k=std::clamp<size_t>(it-arc.begin(),1,path.size()-1);double span=arc[k]-arc[k-1];double f=span>1e-14?(distance-arc[k-1])/span:0;rhs.row(row)=((1-f)*path[k-1]+f*path[k]-fixed.at(s)).transpose();}
  for(int col=0;col<unknown;++col){std::vector<Vec> basis(control_count,Vec::Zero());basis[col+3][0]=1;BSpline spline(basis);for(int row=0;row<unknown;++row)matrix(row,col)=spline.at(u[row])[0];}
  Eigen::MatrixXd solution=matrix.colPivHouseholderQr().solve(rhs);if(!solution.allFinite())throw std::runtime_error("Spline fit is singular");
  for(int i=0;i<unknown;++i)controls[i+3]=solution.row(i).transpose();
  BSpline spline(controls);double T=.05;
  // Convex-hull bounds on derivative controls hold for all times, not only samples.
  for(int i=0;i<6;++i){T=std::max(T,spline.derivativeControlBound(1)[i]/limits.v[i]);T=std::max(T,std::sqrt(spline.derivativeControlBound(2)[i]/limits.a[i]));T=std::max(T,std::cbrt(spline.derivativeControlBound(3)[i]/limits.j[i]));}
  T*=1.005;Curve curve;curve.name="quintic_bspline_"+std::to_string(controls.size());curve.duration=T;for(double s:spline.knots())curve.breaks.push_back(s*T);
  curve.sample=[spline,T](double t){double u=std::clamp(t/T,0.,1.);Sample s;s.q=spline.at(u);s.v=spline.at(u,1)/T;s.a=spline.at(u,2)/(T*T);s.j=spline.at(u,3)/(T*T*T);return s;};return curve;
}

inline Curve ruckigCurve(const Vec& from,const Vec& to,const Limits& limits,const Vec& initial_velocity=Vec::Zero()){
  if((from-to).cwiseAbs().maxCoeff()<1e-8&&initial_velocity.cwiseAbs().maxCoeff()<1e-8){Curve hold;hold.name="hold";hold.duration=.05;hold.breaks={0,.05};hold.sample=[from](double){Sample s;s.q=from;return s;};return hold;}
  ruckig::Ruckig<6> otg;ruckig::InputParameter<6> input;ruckig::Trajectory<6> trajectory;
  for(int i=0;i<6;++i){input.current_position[i]=from[i];input.current_velocity[i]=initial_velocity[i];input.current_acceleration[i]=0;input.target_position[i]=to[i];input.target_velocity[i]=0;input.target_acceleration[i]=0;input.max_velocity[i]=limits.v[i];input.max_acceleration[i]=limits.a[i];input.max_jerk[i]=limits.j[i];}
  auto status=otg.calculate(input,trajectory);if(int(status)<0)throw std::runtime_error("Ruckig error "+std::to_string(int(status)));
  Curve curve;curve.name="ruckig_direct";curve.duration=trajectory.get_duration();curve.breaks={0,curve.duration};
  for(const auto& section:trajectory.get_profiles())for(const auto& profile:section){for(double t:profile.t_sum)curve.breaks.push_back(t+profile.brake.duration);for(double t:profile.brake.t)curve.breaks.push_back(t);}
  curve.sample=[trajectory](double t){std::array<double,6> q,v,a;trajectory.at_time(std::clamp(t,0.,trajectory.get_duration()),q,v,a);Sample s;for(int i=0;i<6;++i){s.q[i]=q[i];s.v[i]=v[i];s.a[i]=a[i];}auto profiles=trajectory.get_profiles();for(int i=0;i<6;++i){const auto& p=profiles[0][i];double u=t-p.brake.duration;auto it=std::upper_bound(p.t_sum.begin(),p.t_sum.end(),u);int k=it-p.t_sum.begin();s.j[i]=(u>=0&&k<7)?p.j[k]:0;}return s;};return curve;
}

// Exact position/velocity/acceleration Hermite interpolation used by the controller.
inline Sample quinticInterpolate(const Sample& a,const Sample& b,double duration,double elapsed){
  if(duration<=0)return a;double u=std::clamp(elapsed/duration,0.,1.);Vec c0=a.q,c1=a.v*duration,c2=.5*a.a*duration*duration;Vec x=b.q-c0-c1-c2,y=b.v*duration-c1-2*c2,z=b.a*duration*duration-2*c2;
  Vec c3=10*x-4*y+.5*z,c4=-15*x+7*y-z,c5=6*x-3*y+.5*z;Sample s;
  s.q=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));s.v=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/duration;s.a=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(duration*duration);s.j=(6*c3+u*(24*c4+u*60*c5))/(duration*duration*duration);return s;
}
inline std::vector<double> sampleTimes(const Curve& curve,double step=.008){
  // Nearly coincident phase switches from different joints can be microseconds
  // apart. Quintic coefficients then divide floating-point cancellation by dt^3.
  // Keep meaningful switches, coalesce only sub-0.5-ms neighbours, and check the
  // actual interpolant afterwards instead of emitting numerically ill-conditioned points.
  constexpr double minimum=.0005;auto native=curve.breaks;native.push_back(0);native.push_back(curve.duration);std::sort(native.begin(),native.end());std::vector<double> times{0};
  for(double t:native)if(t>=minimum&&t<curve.duration-minimum&&t-times.back()>=minimum)times.push_back(t);
  if(curve.duration>0){if(times.size()>1&&curve.duration-times.back()<minimum)times.pop_back();times.push_back(curve.duration);}
  for(double t=step;t<curve.duration;t+=step){auto next=std::lower_bound(times.begin(),times.end(),t);if(next!=times.end()&&*next-t<minimum)continue;if(next!=times.begin()&&t-*std::prev(next)<minimum)continue;times.insert(next,t);}
  return times;
}

struct DerivativeBounds {Vec v=Vec::Zero(),a=Vec::Zero(),j=Vec::Zero();};
inline DerivativeBounds interpolationBounds(const Sample& a,const Sample& b,double dt){
  Vec c1=a.v*dt,c2=.5*a.a*dt*dt,x=b.q-a.q-c1-c2,y=b.v*dt-c1-2*c2,z=b.a*dt*dt-2*c2;
  Vec c3=10*x-4*y+.5*z,c4=-15*x+7*y-z,c5=6*x-3*y+.5*z;DerivativeBounds bounds;
  // Velocity: a degree-four Bernstein convex-hull bound, independent of position offset.
  std::array<Vec,5> power={c1/dt,2*c2/dt,3*c3/dt,4*c4/dt,5*c5/dt};
  auto choose=[](int n,int k){double v=1;for(int i=1;i<=k;++i)v*=double(n-i+1)/i;return v;};
  for(int i=0;i<5;++i){Vec control=Vec::Zero();for(int k=0;k<=i;++k)control+=choose(i,k)/choose(4,k)*power[k];bounds.v=bounds.v.cwiseMax(control.cwiseAbs());}
  // Jerk is quadratic. Check its extrema and the roots where acceleration peaks.
  for(int j=0;j<6;++j){double A=6*c3[j]/(dt*dt*dt),B=24*c4[j]/(dt*dt*dt),C=60*c5[j]/(dt*dt*dt);
    auto jerk=[&](double u){return A+u*(B+u*C);};
    auto accel=[&](double u){return a.a[j]+dt*(A*u+.5*B*u*u+C*u*u*u/3);};
    bounds.j[j]=std::max(std::abs(jerk(0)),std::abs(jerk(1)));bounds.a[j]=std::max(std::abs(accel(0)),std::abs(accel(1)));
    if(std::abs(C)>1e-12){double critical=-B/(2*C);if(critical>0&&critical<1)bounds.j[j]=std::max(bounds.j[j],std::abs(jerk(critical)));double discriminant=B*B-4*C*A;if(discriminant>=0)for(double u:{(-B-std::sqrt(discriminant))/(2*C),(-B+std::sqrt(discriminant))/(2*C)})if(u>0&&u<1)bounds.a[j]=std::max(bounds.a[j],std::abs(accel(u)));}
    else if(std::abs(B)>1e-12){double u=-A/B;if(u>0&&u<1)bounds.a[j]=std::max(bounds.a[j],std::abs(accel(u)));}
  }return bounds;
}
inline bool conditionForController(Curve& curve,const Limits& limits){
  for(int iteration=0;iteration<8;++iteration){double scale=1.;auto times=sampleTimes(curve);
    for(size_t i=1;i<times.size();++i){double dt=times[i]-times[i-1];if(dt<=0)continue;auto bounds=interpolationBounds(curve.sample(times[i-1]),curve.sample(times[i]),dt);
      for(int j=0;j<6;++j){scale=std::max(scale,bounds.v[j]/limits.v[j]);scale=std::max(scale,std::sqrt(bounds.a[j]/limits.a[j]));scale=std::max(scale,std::cbrt(bounds.j[j]/limits.j[j]));}}
    if(!std::isfinite(scale))return false;if(scale<=1.00001)return true;curve.scaleTime(scale*1.005);
  }return false;
}
}
