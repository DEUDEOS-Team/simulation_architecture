#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Color.hh>
#include <sdf/sdf.hh>

#include <gz/msgs/visual.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/msgs/color.pb.h>

#include <chrono>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

// 4. Faz olan RED_YELLOW eklendi
enum class Phase { GREEN, YELLOW, RED, RED_YELLOW };

class TrafficLightPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->model = Model(_entity);
    
    // Süreler tam istediğin gibi ayarlandı
    this->greenDuration     = std::chrono::seconds(15);
    this->yellowDuration    = std::chrono::seconds(3);
    this->redDuration       = std::chrono::seconds(10);
    this->redYellowDuration = std::chrono::seconds(3);
    
    this->phase = Phase::GREEN;

    this->node = std::make_unique<transport::Node>();
    this->pub  = this->node->Advertise<msgs::StringMsg>("/traffic_light/state");

    gzmsg << "\n[TrafficLightPlugin] 4 Fazli Gercekci Mod Devrede!\n";
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override
  {
    if (_info.paused) return;

    if (this->linkEntity == kNullEntity) {
      this->linkEntity = this->model.LinkByName(_ecm, "link");
      if (this->linkEntity == kNullEntity) return;
    }

    if (!this->visualsFound) {
        FindVisuals(_ecm);
        if (this->redVis == kNullEntity || this->yellowVis == kNullEntity || this->greenVis == kNullEntity) {
            return; 
        }
        
        this->visualsFound = true;
        this->phaseStartSimTime = _info.simTime;
        this->ApplyColors(_ecm);
        this->PublishState();
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime - this->phaseStartSimTime);
    bool advance = false;

    // Faz geçiş mantığı yeni sıraya göre ayarlandı
    switch (this->phase)
    {
      case Phase::GREEN:
        if (elapsed >= this->greenDuration)     { advance = true; this->phase = Phase::YELLOW; }
        break;
      case Phase::YELLOW:
        if (elapsed >= this->yellowDuration)    { advance = true; this->phase = Phase::RED; }
        break;
      case Phase::RED:
        if (elapsed >= this->redDuration)       { advance = true; this->phase = Phase::RED_YELLOW; }
        break;
      case Phase::RED_YELLOW:
        if (elapsed >= this->redYellowDuration) { advance = true; this->phase = Phase::GREEN; }
        break;
    }

    if (advance)
    {
      this->phaseStartSimTime = _info.simTime;
      this->ApplyColors(_ecm);
      this->PublishState();
    }
  }

private:
  void FindVisuals(EntityComponentManager &_ecm) {
      _ecm.Each<components::Visual, components::Name, components::ParentEntity>(
          [&](const Entity &_entity, const components::Visual *, const components::Name *_name, const components::ParentEntity *_parent) -> bool
          {
              if (_parent->Data() == this->linkEntity) {
                  std::string nameStr = _name->Data();
                  if (nameStr.find("red") != std::string::npos) this->redVis = _entity;
                  if (nameStr.find("yellow") != std::string::npos) this->yellowVis = _entity;
                  if (nameStr.find("green") != std::string::npos) this->greenVis = _entity;
              }
              return true;
          });
  }

  void UpdateVisualMaterial(EntityComponentManager &_ecm, Entity _visEntity, math::Color _color)
  {
     if (_visEntity == kNullEntity) return;

     gz::msgs::Visual visMsg;
     visMsg.set_id(_visEntity);
     auto *matMsg = visMsg.mutable_material();

     if (_color.R() < 0.1 && _color.G() < 0.1 && _color.B() < 0.1) {
         matMsg->mutable_ambient()->set_r(0.1f); matMsg->mutable_ambient()->set_g(0.1f); matMsg->mutable_ambient()->set_b(0.1f); matMsg->mutable_ambient()->set_a(1.0f);
         matMsg->mutable_diffuse()->set_r(0.1f); matMsg->mutable_diffuse()->set_g(0.1f); matMsg->mutable_diffuse()->set_b(0.1f); matMsg->mutable_diffuse()->set_a(1.0f);
         matMsg->mutable_emissive()->set_r(0.0f); matMsg->mutable_emissive()->set_g(0.0f); matMsg->mutable_emissive()->set_b(0.0f); matMsg->mutable_emissive()->set_a(1.0f);
     } else {
         matMsg->mutable_ambient()->set_r(_color.R()); matMsg->mutable_ambient()->set_g(_color.G()); matMsg->mutable_ambient()->set_b(_color.B()); matMsg->mutable_ambient()->set_a(1.0f);
         matMsg->mutable_diffuse()->set_r(_color.R()); matMsg->mutable_diffuse()->set_g(_color.G()); matMsg->mutable_diffuse()->set_b(_color.B()); matMsg->mutable_diffuse()->set_a(1.0f);
         matMsg->mutable_emissive()->set_r(_color.R() * 20.0f); matMsg->mutable_emissive()->set_g(_color.G() * 20.0f); matMsg->mutable_emissive()->set_b(_color.B() * 20.0f); matMsg->mutable_emissive()->set_a(1.0f);
     }

     auto *visCmd = _ecm.Component<components::VisualCmd>(_visEntity);
     if (!visCmd) {
         _ecm.CreateComponent(_visEntity, components::VisualCmd(visMsg));
     } else {
         *visCmd = components::VisualCmd(visMsg);
         _ecm.SetChanged(_visEntity, components::VisualCmd::typeId, ComponentState::OneTimeChange);
     }
  }

  void ApplyColors(EntityComponentManager &_ecm)
  {
    math::Color off(0.0f, 0.0f, 0.0f, 1.0f);
    math::Color red(1.0f, 0.0f, 0.0f, 1.0f);
    math::Color yellow(1.0f, 1.0f, 0.0f, 1.0f);
    math::Color green(0.0f, 1.0f, 0.0f, 1.0f);

    switch (this->phase)
    {
      case Phase::GREEN:
        UpdateVisualMaterial(_ecm, this->redVis, off);
        UpdateVisualMaterial(_ecm, this->yellowVis, off);
        UpdateVisualMaterial(_ecm, this->greenVis, green);
        break;
      case Phase::YELLOW:
        UpdateVisualMaterial(_ecm, this->redVis, off);
        UpdateVisualMaterial(_ecm, this->yellowVis, yellow);
        UpdateVisualMaterial(_ecm, this->greenVis, off);
        break;
      case Phase::RED:
        UpdateVisualMaterial(_ecm, this->redVis, red);
        UpdateVisualMaterial(_ecm, this->yellowVis, off);
        UpdateVisualMaterial(_ecm, this->greenVis, off);
        break;
      case Phase::RED_YELLOW: // Yeni ikili yanma fazı
        UpdateVisualMaterial(_ecm, this->redVis, red);
        UpdateVisualMaterial(_ecm, this->yellowVis, yellow);
        UpdateVisualMaterial(_ecm, this->greenVis, off);
        break;
    }
  }

  void PublishState()
  {
    msgs::StringMsg msg;
    switch (this->phase)
    {
      case Phase::GREEN:      msg.set_data("GREEN");      break;
      case Phase::YELLOW:     msg.set_data("YELLOW");     break;
      case Phase::RED:        msg.set_data("RED");        break;
      case Phase::RED_YELLOW: msg.set_data("RED_YELLOW"); break; // ROS 2 tarafı için yeni topic verisi
    }
    this->pub.Publish(msg);
    gzmsg << "[TrafficLight] Faz degisti -> " << msg.data() << "\n";
  }

  Model model{kNullEntity};
  Entity linkEntity{kNullEntity};
  Entity redVis{kNullEntity};
  Entity yellowVis{kNullEntity};
  Entity greenVis{kNullEntity};
  bool visualsFound{false};
  
  Phase phase{Phase::GREEN};
  std::chrono::steady_clock::duration phaseStartSimTime{0};

  // Süre değişkenleri
  std::chrono::seconds greenDuration{15};
  std::chrono::seconds yellowDuration{3};
  std::chrono::seconds redDuration{10};
  std::chrono::seconds redYellowDuration{3};

  std::unique_ptr<transport::Node> node;
  transport::Node::Publisher pub;
};

GZ_ADD_PLUGIN(TrafficLightPlugin, 
              gz::sim::System, 
              gz::sim::ISystemConfigure, 
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(TrafficLightPlugin, "gz::sim::systems::TrafficLightPlugin")