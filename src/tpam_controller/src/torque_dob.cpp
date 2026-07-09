#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tpam_interfaces/msg/wrench.hpp>
#include <tpam_interfaces/msg/tpam_state.hpp>

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <optional>
#include <algorithm>

struct TorqueDhat {
  Eigen::Vector3f xyz{Eigen::Vector3f::Zero()};
};

class TorqueDOBCore {
public:
  struct Params {
    float wc = 10.0f;                 // [rad/s] cutoff
    float Jxx = 0.360091f, Jyy = 0.360702f, Jzz = 0.660702f; // [kg*m^2]
    float limit = 10.0f;              // [N*m] saturation for dhat
  };

  explicit TorqueDOBCore(const Params& p) : p_(p) { reset(); }

  void setParams(const Params& p) { p_ = p; }

  void reset() {
    xQ_.setZero();
    xM_.setZero();
    dhat_.setZero();
  }

  void update(float dt,
              const Eigen::Vector3f& tau_for_q,
              const Eigen::Vector3f& omega_rpy,      // must be SAME frame as tau (body r/p/y axis)
              TorqueDhat& torque_dhat)
  {
    const float wc  = std::max(0.0f, p_.wc);
    const float wc2 = wc * wc;
    const float root2 = std::sqrt(2.0f);

    // Common A,B
    Eigen::Matrix2f A;
    A << -root2*wc, -wc2,
          1.0f,     0.0f;
    Eigen::Vector2f B(1.0f, 0.0f);

    // Q-filter output: yQ = [0, wc^2] xQ
    Eigen::RowVector2f Cq(0.0f, wc2);

    // MinvQ output: yM = [J*wc^2, 0] xM  (axis-dependent J)
    const float J[3] = {p_.Jxx, p_.Jyy, p_.Jzz};

    for (int i = 0; i < 3; ++i) {
      // ---- MinvQ path (input = omega) ----
      Eigen::Vector2f xM_dot = A * xM_.col(i) + B * omega_rpy(i);
      xM_.col(i) += xM_dot * dt;
      Eigen::RowVector2f Cm(J[i]*wc2, 0.0f);
      const float yM = (Cm * xM_.col(i))(0);

      // ---- Q path (input = actual torque command sent to plant) ----
      Eigen::Vector2f xQ_dot = A * xQ_.col(i) + B * tau_for_q(i);
      xQ_.col(i) += xQ_dot * dt;
      const float yQ = (Cq * xQ_.col(i))(0);

      dhat_(i) = saturateFinite(yM - yQ, p_.limit);
    }

    torque_dhat.xyz = dhat_;
  }

private:
  static float saturateFinite(float v, float limit) {
    if (!std::isfinite(v)) return 0.0f;
    if (std::fabs(v) > limit) return std::copysign(limit, v);
    return v;
  }

  Params p_;
  Eigen::Matrix<float, 2, 3> xQ_;   // each col: Q state for axis i
  Eigen::Matrix<float, 2, 3> xM_;   // each col: MinvQ state for axis i
  Eigen::Vector3f dhat_;
};

class TorqueDobNode : public rclcpp::Node {
public:
  TorqueDobNode() : Node("torque_dob")
  {
    // ---- topics ----
    in_wrench_topic_  = declare_parameter<std::string>("in_wrench_topic",  "/wrench_des");
    out_wrench_topic_ = declare_parameter<std::string>("out_wrench_topic", "/wrench_cmd");
    state_topic_      = declare_parameter<std::string>("state_topic",      "/Tpam_state");
    enable_topic_     = declare_parameter<std::string>("enable_topic",     "/dob_enable");
    dhat_topic_       = declare_parameter<std::string>("dhat_topic",       "/dob_dhat");

    // ---- timing ----
    default_dt_ = declare_parameter<double>("default_dt", 1.0/400.0);
    dt_min_     = declare_parameter<double>("dt_min",     1e-4);
    dt_max_     = declare_parameter<double>("dt_max",     0.05);

    // ---- behavior ----
    enabled_.store(declare_parameter<bool>("enable_on_start", false));
    reset_on_enable_ = declare_parameter<bool>("reset_on_enable", false);
    dhat_rate_limit_ = (float)declare_parameter<double>("dhat_rate_limit", 5.0);

    // ---- DOB params ----
    TorqueDOBCore::Params p;
    p.wc    = (float)declare_parameter<double>("wc", 10.0);  // rad/s
    p.Jxx   = (float)declare_parameter<double>("Jxx", 0.360091);
    p.Jyy   = (float)declare_parameter<double>("Jyy", 0.360702);
    p.Jzz   = (float)declare_parameter<double>("Jzz", 0.660702);
    p.limit = (float)declare_parameter<double>("limit", 10.0);
    core_.setParams(p);

    using std::placeholders::_1;

    // state: gyro (w_rpy) is high-rate → SensorDataQoS recommended
    sub_state_ = create_subscription<tpam_interfaces::msg::TpamState>(
      state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TorqueDobNode::onState, this, _1));

    sub_wrench_ = create_subscription<tpam_interfaces::msg::Wrench>(
      in_wrench_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TorqueDobNode::onWrench, this, _1));

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TorqueDobNode::onEnable, this, _1));

    pub_wrench_ = create_publisher<tpam_interfaces::msg::Wrench>(
      out_wrench_topic_, rclcpp::SystemDefaultsQoS());

    pub_dhat_ = create_publisher<tpam_interfaces::msg::Wrench>(
      dhat_topic_, rclcpp::SystemDefaultsQoS());

    pub_dhat_used_ = create_publisher<tpam_interfaces::msg::Wrench>(
      "/dob_dhat_used", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(get_logger(),
      "TorqueDobNode ready.\n  in:  %s\n  out: %s\n  state: %s\n  enable: %s",
      in_wrench_topic_.c_str(), out_wrench_topic_.c_str(),
      state_topic_.c_str(), enable_topic_.c_str());
  }

private:
  void onEnable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = enabled_.load();
    const bool next = msg->data;
    if (prev == next) return;

    enabled_.store(next);

    if (next) {
      dhat_used_.setZero();
      if (reset_on_enable_) {
        core_.reset();
        last_time_.reset();
      }
      RCLCPP_WARN(get_logger(), "DOB ENABLED%s", reset_on_enable_ ? " (reset)" : "");
    } else {
      dhat_used_.setZero();
      RCLCPP_WARN(get_logger(), "DOB DISABLED (bypass)");
    }
  }

  void onState(const tpam_interfaces::msg::TpamState::SharedPtr msg)
  {
    // Using w_rpy exactly as your wrench_controller does:
    // w_rpy = [roll_rate, pitch_rate, yaw_rate] (same axis order)
    omega_(0) = (float)msg->w_rpy[0];
    omega_(1) = (float)msg->w_rpy[1];
    omega_(2) = (float)msg->w_rpy[2];
    have_state_.store(true);
  }

  void publishDhat(const Eigen::Vector3f& dhat)
  {
    tpam_interfaces::msg::Wrench w;
    w.moment[0] = dhat(0);
    w.moment[1] = dhat(1);
    w.moment[2] = dhat(2);
    w.force[0] = 0.0f;
    w.force[1] = 0.0f;
    w.force[2] = 0.0f;
    pub_dhat_->publish(w);
  }

  void publishDhatUsed(const Eigen::Vector3f& dhat_used)
  {
    tpam_interfaces::msg::Wrench w;
    w.moment[0] = dhat_used(0);
    w.moment[1] = dhat_used(1);
    w.moment[2] = dhat_used(2);
    w.force[0] = 0.0f;
    w.force[1] = 0.0f;
    w.force[2] = 0.0f;
    pub_dhat_used_->publish(w);
  }

  void updateDhatUsed(float dt, const Eigen::Vector3f& dhat_est)
  {
    if (!enabled_.load()) {
      dhat_used_.setZero();
      return;
    }

    const float max_step = std::max(0.0f, dhat_rate_limit_) * dt;
    for (int i = 0; i < 3; ++i) {
      const float error = dhat_est(i) - dhat_used_(i);
      dhat_used_(i) += std::clamp(error, -max_step, max_step);
    }
  }

  void onWrench(const tpam_interfaces::msg::Wrench::SharedPtr msg)
  {
    // If no gyro yet -> bypass
    if (!have_state_.load()) {
      pub_wrench_->publish(*msg);
      publishDhat(Eigen::Vector3f::Zero());
      publishDhatUsed(Eigen::Vector3f::Zero());
      return;
    }

    // dt from node clock (msg has no header)
    const auto now = this->now();
    double dt = default_dt_;
    if (last_time_.has_value()) {
      dt = (now - last_time_.value()).seconds();
      if (!(dt >= dt_min_ && dt <= dt_max_)) dt = default_dt_;
    }
    last_time_ = now;

    // Desired torque from wrench input
    Eigen::Vector3f tau_des;
    tau_des << (float)msg->moment[0], (float)msg->moment[1], (float)msg->moment[2];

    // Output wrench initially = input (force passthrough always)
    tpam_interfaces::msg::Wrench out = *msg;

    // Use the compensation value from the previous callback for both plant
    // output and the Q-filter input. Then update dhat_used_ for the next
    // callback. This avoids an algebraic loop and prevents enable-step jumps.
    const bool enabled = enabled_.load();
    const Eigen::Vector3f dhat_used_now = enabled ? dhat_used_ : Eigen::Vector3f::Zero();
    const Eigen::Vector3f tau_cmd = tau_des - dhat_used_now;
    const Eigen::Vector3f tau_for_q = enabled ? tau_cmd : tau_des;

    TorqueDhat dhat;
    core_.update((float)dt, tau_for_q, omega_, dhat);

    out.moment[0] = tau_cmd(0);
    out.moment[1] = tau_cmd(1);
    out.moment[2] = tau_cmd(2);

    publishDhat(dhat.xyz);
    publishDhatUsed(dhat_used_now);

    updateDhatUsed((float)dt, dhat.xyz);

    pub_wrench_->publish(out);
  }

  // topics
  std::string in_wrench_topic_, out_wrench_topic_, state_topic_, enable_topic_, dhat_topic_;

  // timing
  double default_dt_{1.0/400.0}, dt_min_{1e-4}, dt_max_{0.05};
  std::optional<rclcpp::Time> last_time_;

  // core
  TorqueDOBCore core_{TorqueDOBCore::Params{}};
  bool reset_on_enable_{false};
  float dhat_rate_limit_{5.0f};
  Eigen::Vector3f dhat_used_{Eigen::Vector3f::Zero()};

  // state
  std::atomic<bool> have_state_{false};
  Eigen::Vector3f omega_{Eigen::Vector3f::Zero()};

  // enable
  std::atomic<bool> enabled_{false};

  // ROS
  rclcpp::Subscription<tpam_interfaces::msg::TpamState>::SharedPtr sub_state_;
  rclcpp::Subscription<tpam_interfaces::msg::Wrench>::SharedPtr sub_wrench_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<tpam_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Publisher<tpam_interfaces::msg::Wrench>::SharedPtr pub_dhat_;
  rclcpp::Publisher<tpam_interfaces::msg::Wrench>::SharedPtr pub_dhat_used_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueDobNode>());
  rclcpp::shutdown();
  return 0;
}
