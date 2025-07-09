#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {

class DynamicObstaclePlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
    this->model = _model;
    auto world = this->model->GetWorld();

    // actor 모델을 찾기
    this->actorModel = world->ModelByName("walking_person");
    if (!this->actorModel) {
      gzerr << "[DynamicObstaclePlugin] Couldn't find actor model 'walking_person'\n";
      return;
    }

    // 업데이트 함수 연결
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DynamicObstaclePlugin::OnUpdate, this));
  }

  void OnUpdate() {
    if (!this->actorModel) return;

    // 사람 위치 받아오기
    auto actorPose = this->actorModel->WorldPose();

    // 위치만 가져오고 회전은 무시
    ignition::math::Vector3d position = actorPose.Pos();
    position.Z() += 0.05;  // 겹침 방지

    // 회전은 고정된 값으로 설정 (Yaw = 0)
    ignition::math::Quaterniond fixedRotation(0, 0, 0);
    ignition::math::Pose3d newPose(position, fixedRotation);

    // 박스 위치 업데이트
    this->model->SetWorldPose(newPose);
  }


private:
  physics::ModelPtr model;
  physics::ModelPtr actorModel;
  event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(DynamicObstaclePlugin)

} // namespace gazebo
