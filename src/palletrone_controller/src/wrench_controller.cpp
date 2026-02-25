#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>

class WrenchController : public rclcpp::Node
{
public:
  WrenchController() : rclcpp::Node("wrench_controller")
  {
    // ===================== Gains =====================
    // Position PID (I-gain 유지!)
    const double KP_POS[3] = {30.0, 30.0, 30.0};
    const double KI_POS[3] = {0.20, 0.20, 0.05};
    const double KD_POS[3] = {5.00, 5.00, 1.50};
    const double I_MIN_POS = -5.0, I_MAX_POS = 5.0, OUT_MIN_POS = -200.0, OUT_MAX_POS = 200.0;

    // Attitude PD (기본 안정적으로 I는 0)
    const double KP_ATT[3] = {50.0, 50.0, 30.0};
    const double KI_ATT[3] = {0.0,  0.0,  0.0};   // <-- I 끔 (원하면 다시 켜줄게)
    const double KD_ATT[3] = {10.0, 10.0, 10.0};
    const double I_MIN_ATT = -1.0, I_MAX_ATT = 1.0, OUT_MIN_ATT = -20.0, OUT_MAX_ATT = 20.0;

    auto init_pid = [](double kp, double ki, double kd,
                       double i_min, double i_max,
                       double out_min, double out_max)
      -> std::function<double(double,double,double,double)>
    {
      double iacc = 0.0;
      return [=](double ref, double cur, double dcur, double dt) mutable
      {
        if (dt <= 0.0) dt = 1e-3;
        const double e  = ref - cur;
        const double de = -dcur;              // ref_dot=0 가정
        iacc += ki * e * dt;                  // iacc는 I-term (ki 포함)
        iacc = std::clamp(iacc, i_min, i_max);
        double u = kp*e + iacc + kd*de;
        return std::clamp(u, out_min, out_max);
      };
    };

    // pos PID
    pid_pos_[0] = init_pid(KP_POS[0], KI_POS[0], KD_POS[0], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);
    pid_pos_[1] = init_pid(KP_POS[1], KI_POS[1], KD_POS[1], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);
    pid_pos_[2] = init_pid(KP_POS[2], KI_POS[2], KD_POS[2], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);

    // att PID (현재는 PD)
    pid_att_[0] = init_pid(KP_ATT[0], KI_ATT[0], KD_ATT[0], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);
    pid_att_[1] = init_pid(KP_ATT[1], KI_ATT[1], KD_ATT[1], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);
    pid_att_[2] = init_pid(KP_ATT[2], KI_ATT[2], KD_ATT[2], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);

    // ===================== ROS I/O =====================
    sub_cmd_ = this->create_subscription<palletrone_interfaces::msg::Cmd>(
      "/cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&WrenchController::onCmd, this, std::placeholders::_1));

    // state는 고주기 → SensorDataQoS
    sub_state_ = this->create_subscription<palletrone_interfaces::msg::PalletroneState>(
      "/palletrone_state", rclcpp::SensorDataQoS(),
      std::bind(&WrenchController::onState, this, std::placeholders::_1));

    // DOB 중간 삽입용: wrench_des로 publish
    pub_wrench_ = this->create_publisher<palletrone_interfaces::msg::Wrench>(
      "/wrench_des", rclcpp::SystemDefaultsQoS());

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "wrench_controller ready. pub=/wrench_des, sub=/cmd,/palletrone_state (SensorDataQoS)");
  }

private:
  static double wrap_err(double ref, double cur)
  {
    return std::atan2(std::sin(ref - cur), std::cos(ref - cur));
  }

  void onCmd(const palletrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    pos_cmd_ << (double)msg->pos_cmd[0], (double)msg->pos_cmd[1], (double)msg->pos_cmd[2];
    rpy_cmd_ << (double)msg->rpy_cmd[0], (double)msg->rpy_cmd[1], (double)msg->rpy_cmd[2];

    // cmd 최초 수신 순간: state 이미 받은 상태면 현재 위치/자세를 latch해서 점프 줄이기
    if (!have_cmd_) {
      if (have_state_) {
        pos_hold_ = pos_;
        have_hold_ = true;

        yaw_hold_ = rpy_.z();
        have_yaw_hold_ = true;
      }
      // state가 아직이면 최초 onState에서 latch됨
    }

    have_cmd_ = true;
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
  {
    // ---------- read state ----------
    pos_    << (double)msg->pos[0],   (double)msg->pos[1],   (double)msg->pos[2];
    vel_    << (double)msg->vel[0],   (double)msg->vel[1],   (double)msg->vel[2];
    rpy_    << (double)msg->rpy[0],   (double)msg->rpy[1],   (double)msg->rpy[2];
    w_body_ << (double)msg->w_rpy[0], (double)msg->w_rpy[1], (double)msg->w_rpy[2];

    const auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0/400.0;

    have_state_ = true;

    // ---------- hold latch (state 최초 1회) ----------
    if (!have_hold_) {
      pos_hold_ = pos_;
      have_hold_ = true;

      yaw_hold_ = rpy_.z();
      have_yaw_hold_ = true;
    }

    // ---------- references ----------
    // cmd 없으면 pos_hold로 hover 유지
    const Eigen::Vector3d pos_ref = have_cmd_ ? pos_cmd_ : pos_hold_;

    // attitude cmd:
    // cmd 있으면 rpy_cmd 사용
    // cmd 없으면 roll/pitch=0, yaw는 (옵션) 현재 yaw hold
    double roll_ref  = 0.0;
    double pitch_ref = 0.0;
    double yaw_ref   = 0.0;

    if (have_cmd_) {
      roll_ref  = rpy_cmd_.x();
      pitch_ref = rpy_cmd_.y();
      yaw_ref   = rpy_cmd_.z();
    } else {
      if (yaw_hold_enabled_ && have_yaw_hold_) yaw_ref = yaw_hold_;
      else yaw_ref = 0.0;
    }

    // ---------- position control (PID) ----------
    Eigen::Vector3d u_pos;
    u_pos.x() = pid_pos_[0](pos_ref.x(), pos_.x(), vel_.x(), dt);
    u_pos.y() = pid_pos_[1](pos_ref.y(), pos_.y(), vel_.y(), dt);
    u_pos.z() = pid_pos_[2](pos_ref.z(), pos_.z(), vel_.z(), dt);

    // world force (gravity comp)
    Eigen::Vector3d F_world(u_pos.x(), u_pos.y(), u_pos.z() + mass_ * grav_);

    // ---------- rotation (world->body) ----------
    const double r = rpy_.x(), p = rpy_.y(), y = rpy_.z();
    const double sr = std::sin(r), cr = std::cos(r);
    const double sp = std::sin(p), cp = std::cos(p);
    const double sy = std::sin(y), cy = std::cos(y);

    Eigen::Matrix3d R_WB;
    R_WB <<  cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
               -sp,              cp*sr,              cp*cr;

    const Eigen::Vector3d F_body = R_WB.transpose() * F_world;

    // ---------- attitude control ----------
    // 각 wrap해서 equivalent reference 만들기
    const double roll_ref_equiv  = rpy_.x() + wrap_err(roll_ref,  rpy_.x());
    const double pitch_ref_equiv = rpy_.y() + wrap_err(pitch_ref, rpy_.y());
    const double yaw_ref_equiv   = rpy_.z() + wrap_err(yaw_ref,   rpy_.z());

    Eigen::Vector3d M_body;
    M_body.x() = pid_att_[0](roll_ref_equiv,  rpy_.x(), w_body_.x(), dt);
    M_body.y() = pid_att_[1](pitch_ref_equiv, rpy_.y(), w_body_.y(), dt);
    M_body.z() = pid_att_[2](yaw_ref_equiv,   rpy_.z(), w_body_.z(), dt);

    // ---------- publish wrench_des ----------
    palletrone_interfaces::msg::Wrench w;
    w.moment[0] = (float)M_body.x();
    w.moment[1] = (float)M_body.y();
    w.moment[2] = (float)M_body.z();
    w.force[0]  = (float)F_body.x();
    w.force[1]  = (float)F_body.y();
    w.force[2]  = (float)F_body.z();

    pub_wrench_->publish(w);
  }

  // ROS
  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Time last_time_;

  // command/state
  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_cmd_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};

  // hold latch
  Eigen::Vector3d pos_hold_{Eigen::Vector3d::Zero()};
  bool have_hold_{false};

  bool yaw_hold_enabled_{true};
  double yaw_hold_{0.0};
  bool have_yaw_hold_{false};

  // PID
  std::function<double(double,double,double,double)> pid_pos_[3];
  std::function<double(double,double,double,double)> pid_att_[3];

  // params
  double mass_{5.718};
  double grav_{9.81};

  bool have_state_{false};
  bool have_cmd_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}