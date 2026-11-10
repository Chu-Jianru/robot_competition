// 只用于验收：读取 Gazebo 接触真值，绝不向控制器提供位姿或速度。
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <atomic>
#include <csignal>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <array>
#include <mutex>

static std::atomic<bool> running{true};
static std::map<std::string,double> last;
static std::map<std::string,std::array<double,6>> model_poses;
static std::ofstream pose_log;
static double last_pose=-1;
void stop(int) { running=false; }
void contacts(ConstContactsPtr &message) {
  const double t=message->time().sec()+message->time().nsec()*1e-9;
  for (int i=0;i<message->contact_size();++i) {
    const auto &contact=message->contact(i);
    std::string a=contact.collision1(), b=contact.collision2();
    if (a.find("competition_robot::")==std::string::npos && b.find("competition_robot::")==std::string::npos) continue;
    if (a.find("block_")==std::string::npos && b.find("block_")==std::string::npos &&
        a.find("test_opponent::")==std::string::npos && b.find("test_opponent::")==std::string::npos) continue;
    if (b<a) std::swap(a,b);
    const std::string key=a+"|"+b;
    if (last.count(key) && t-last[key]<.10) continue;
    last[key]=t;
    double depth=0;
    for (int j=0;j<contact.depth_size();++j) depth=std::max(depth,contact.depth(j));
    std::cout<<"{\"t\":"<<t<<",\"a\":\""<<a<<"\",\"b\":\""<<b<<"\",\"depth\":"<<depth<<"}"<<std::endl;
  }
}
void poses(ConstPosesStampedPtr &message) {
  const double t=message->time().sec()+message->time().nsec()*1e-9;
  for (int i=0;i<message->pose_size();++i) {
    const auto &pose=message->pose(i);const auto &name=pose.name();
    if (name!="competition_robot" && !(name.size()==8 && name.substr(0,6)=="block_")) continue;
    const auto &p=pose.position();const auto &q=pose.orientation();
    const double roll=std::atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(q.x()*q.x()+q.y()*q.y()));
    const double pitch=std::asin(std::max(-1.,std::min(1.,2*(q.w()*q.y()-q.z()*q.x()))));
    const double yaw=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
    model_poses[name]={p.x(),p.y(),p.z(),roll,pitch,yaw};
  }
  if (!pose_log.is_open() || t-last_pose<.10) return;
  last_pose=t;pose_log<<std::setprecision(12)<<"{\"t\":"<<t<<",\"poses\":{";bool first=true;
  for (const auto &item:model_poses) {
    if (!first) pose_log<<",";first=false;
    pose_log<<"\""<<item.first<<"\":[";
    for (size_t i=0;i<6;++i) {if (i) pose_log<<",";pose_log<<item.second[i];}
    pose_log<<"]";
  }
  pose_log<<"}}"<<std::endl;
}
int main(int argc,char **argv) {
  if (argc>1) pose_log.open(argv[1]);
  gazebo::client::setup();
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());node->Init("competition");
  auto sub=node->Subscribe("~/physics/contacts",contacts);
  auto pose_sub=node->Subscribe("~/pose/info",poses);
  while(running) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  sub.reset();pose_sub.reset();node->Fini();gazebo::client::shutdown();
}
