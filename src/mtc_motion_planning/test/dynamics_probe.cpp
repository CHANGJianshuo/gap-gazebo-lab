#include <mtc_motion_planning/dynamics.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
int main(int argc,char** argv){
  if(argc!=4)return 2;
  std::ifstream file(argv[1]);std::string xml((std::istreambuf_iterator<char>(file)),{});
  auto urdf=urdf::parseURDF(xml);auto srdf=std::make_shared<srdf::Model>();srdf->initFile(*urdf,argv[2]);
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);moveit::core::RobotState state(model);state.setToDefaultValues();
  const std::array<std::string,6> names={"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};mtc::Dynamics dynamics(urdf,names);
  nlohmann::json input;std::ifstream request(argv[3]);request>>input;auto output=nlohmann::json::array();
  for(const auto& row:input){mtc::Vec v,a;for(int i=0;i<6;++i){state.setVariablePosition(names[i],row["q"][i]);v[i]=row["v"][i];a[i]=row["a"][i];}state.setVariablePosition("latch_left_joint",.022);state.setVariablePosition("latch_right_joint",.022);state.update();mtc::Payload p;
    if(row.contains("payload")){p.mass=row["payload"]["mass"];for(int i=0;i<3;++i){p.com[i]=row["payload"]["com"][i];for(int j=0;j<3;++j)p.inertia(i,j)=row["payload"]["inertia"][i][j];}}
    auto t=dynamics.torque(state,v,a,p);auto T=state.getGlobalLinkTransform("tcp");std::vector<double> q(t.data(),t.data()+6);output.push_back({{"tau",q},{"tcp",{T.translation().x(),T.translation().y(),T.translation().z()}}});
  }
  std::cout<<output.dump()<<std::endl;
}
