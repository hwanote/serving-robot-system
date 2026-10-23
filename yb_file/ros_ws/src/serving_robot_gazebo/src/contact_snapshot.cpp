// Physics contacts: immediate transitions plus 20Hz empty/continuous snapshots.
// Original robot collision geometry is read, never changed. Ground contacts are
// excluded, including normal wheel/caster support; contact with objects is not.
#include <array>
#include <chrono>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
namespace team {
class ContactSnapshot : public gz::sim::System,
 public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
 public:
 ContactSnapshot(){
  for(const auto &r:{std::string("robot_01"),std::string("robot_02")})
   for(int i=0;i<3;++i)publishers_[r][i]=transport_.Advertise<gz::msgs::Contacts>("/model/"+r+"/contacts/sensor_"+std::to_string(i));
 }
 void PreUpdate(const gz::sim::UpdateInfo &,gz::sim::EntityComponentManager &ecm) override {
  bool complete=entities_.size()==2;
  for(const auto &r:entities_)for(const auto &bucket:r.second){complete=complete&&!bucket.empty();for(auto e:bucket)complete=complete&&ecm.HasEntity(e);}
  if(complete)return;
  ecm.Each<gz::sim::components::Collision,gz::sim::components::Name,gz::sim::components::ParentEntity>(
   [&](const gz::sim::Entity &e,const gz::sim::components::Collision *,const gz::sim::components::Name *name,const gz::sim::components::ParentEntity *parent)->bool {
    auto *p=ecm.Component<gz::sim::components::ParentEntity>(parent->Data());if(!p)return true;
    auto *model=ecm.Component<gz::sim::components::Name>(p->Data());if(!model||!publishers_.count(model->Data()))return true;
    int i=name->Data().find("front_bumper_collision")!=std::string::npos?1:name->Data().find("rear_bumper_collision")!=std::string::npos?2:0;
    auto &bucket=entities_[model->Data()][i];bool found=false;for(auto old:bucket)found=found||old==e;if(!found)bucket.push_back(e);
    if(!ecm.Component<gz::sim::components::ContactSensorData>(e))ecm.CreateComponent(e,gz::sim::components::ContactSensorData());
    return true;
   });
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
  if(info.paused||info.dt<=std::chrono::steady_clock::duration::zero())return;
  bool heartbeat=info.simTime-last_>=std::chrono::milliseconds(50);if(heartbeat)last_=info.simTime;
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  for(const auto &r:entities_)for(int i=0;i<3;++i){
   if(r.second[i].empty())continue;
   gz::msgs::Contacts snapshot;bool available=true;std::set<std::pair<uint64_t,uint64_t>> seen;
   for(auto e:r.second[i]){
    auto *data=ecm.Component<gz::sim::components::ContactSensorData>(e);if(!data){available=false;break;}
    for(const auto &contact:data->Data().contact()){
     bool ground=false;
     for(auto id:{contact.collision1().id(),contact.collision2().id()}){auto *name=ecm.Component<gz::sim::components::Name>(id);ground=ground||(name&&name->Data().find("ground")!=std::string::npos);}
     if(ground)continue;
     auto a=contact.collision1().id(),b=contact.collision2().id();if(a>b)std::swap(a,b);
     if(seen.emplace(a,b).second)*snapshot.add_contact()=contact;
    }
   }
   if(!available)continue;
   bool active=snapshot.contact_size()>0;if(!heartbeat&&active==previous_[r.first][i])continue;previous_[r.first][i]=active;
   snapshot.mutable_header()->mutable_stamp()->set_sec(ns/1000000000LL);snapshot.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000LL);
   auto *source=snapshot.mutable_header()->add_data();source->set_key("source");source->add_value("physics_contact_sensor_data");publishers_[r.first][i].Publish(snapshot);
  }
 }
 private:
 gz::transport::Node transport_;
 std::map<std::string,std::array<std::vector<gz::sim::Entity>,3>> entities_;
 std::map<std::string,std::array<gz::transport::Node::Publisher,3>> publishers_;
 std::map<std::string,std::array<bool,3>> previous_;
 std::chrono::steady_clock::duration last_{};
};
}
GZ_ADD_PLUGIN(team::ContactSnapshot,gz::sim::System,team::ContactSnapshot::ISystemPreUpdate,team::ContactSnapshot::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(team::ContactSnapshot,"team::ContactSnapshot")
