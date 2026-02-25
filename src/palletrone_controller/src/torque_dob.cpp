#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <palletrone_interfaces/msg/wrench.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>

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
    float Jxx = 0.0768f, Jyy = 0.0871f, Jzz = 0.113f; // [kg*m^2]
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
              const Eigen::Vector3f& tau_rpy_des,
              const Eigen::Vector3f& omega_rpy,      // must be SAME frame as tau (body r/p/y axis)
              Eigen::Vector3f& tau_rpy_tilde,
              TorqueDhat& torque_dhat,
              bool compensate_flag)
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

    Eigen::Vector3f tau_in = tau_rpy_des;
    if (compensate_flag) tau_in -= dhat_; // feedback-in-Q path (as in your code)

    for (int i = 0; i < 3; ++i) {
      // ---- MinvQ path (input = omega) ----
      Eigen::Vector2f xM_dot = A * xM_.col(i) + B * omega_rpy(i);
      xM_.col(i) += xM_dot * dt;
      Eigen::RowVector2f Cm(J[i]*wc2, 0.0f);
      const float yM = (Cm * xM_.col(i))(0);

      // ---- Q path (input = tau_in) ----
      Eigen::Vector2f xQ_dot = A * xQ_.col(i) + B * tau_in(i);
      xQ_.col(i) += xQ_dot * dt;
      const float yQ = (Cq * xQ_.col(i))(0);

      dhat_(i) = saturateFinite(yM - yQ, p_.limit);
    }

    tau_rpy_tilde = tau_rpy_des - dhat_;
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
    state_topic_      = declare_parameter<std::string>("state_topic",      "/palletrone_state");
    enable_topic_     = declare_parameter<std::string>("enable_topic",     "/dob_enable");
    dhat_topic_       = declare_parameter<std::string>("dhat_topic",       "/dob_dhat");

    // ---- timing ----
    default_dt_ = declare_parameter<double>("default_dt", 1.0/400.0);
    dt_min_     = declare_parameter<double>("dt_min",     1e-4);
    dt_max_     = declare_parameter<double>("dt_max",     0.05);

    // ---- behavior ----
    enabled_.store(declare_parameter<bool>("enable_on_start", false));
    reset_on_enable_ = declare_parameter<bool>("reset_on_enable", true);
    compensate_flag_ = declare_parameter<bool>("compensate_flag", true);

    // ---- DOB params ----
    TorqueDOBCore::Params p;
    p.wc    = (float)declare_parameter<double>("wc", 10.0);  // rad/s
    p.Jxx   = (float)declare_parameter<double>("Jxx", 0.0768);
    p.Jyy   = (float)declare_parameter<double>("Jyy", 0.0871);
    p.Jzz   = (float)declare_parameter<double>("Jzz", 0.113);
    p.limit = (float)declare_parameter<double>("limit", 10.0);
    core_.setParams(p);

    using std::placeholders::_1;

    // state: gyro (w_rpy) is high-rate → SensorDataQoS recommended
    sub_state_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
      state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TorqueDobNode::onState, this, _1));

    sub_wrench_ = create_subscription<palletrone_interfaces::msg::Wrench>(
      in_wrench_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TorqueDobNode::onWrench, this, _1));

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TorqueDobNode::onEnable, this, _1));

    pub_wrench_ = create_publisher<palletrone_interfaces::msg::Wrench>(
      out_wrench_topic_, rclcpp::SystemDefaultsQoS());

    pub_dhat_ = create_publisher<palletrone_interfaces::msg::Wrench>(
      dhat_topic_, rclcpp::SystemDefaultsQoS());

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
      if (reset_on_enable_) {
        core_.reset();
        last_time_.reset();
      }
      RCLCPP_WARN(get_logger(), "DOB ENABLED%s", reset_on_enable_ ? " (reset)" : "");
    } else {
      RCLCPP_WARN(get_logger(), "DOB DISABLED (bypass)");
    }
  }

  void onState(const palletrone_interfaces::msg::PalletroneState::SharedPtr msg)
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
    palletrone_interfaces::msg::Wrench w;
    w.moment[0] = dhat(0);
    w.moment[1] = dhat(1);
    w.moment[2] = dhat(2);
    w.force[0] = 0.0f;
    w.force[1] = 0.0f;
    w.force[2] = 0.0f;
    pub_dhat_->publish(w);
  }

  void onWrench(const palletrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    // If no gyro yet -> bypass
    if (!have_state_.load()) {
      pub_wrench_->publish(*msg);
      publishDhat(Eigen::Vector3f::Zero());
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
    palletrone_interfaces::msg::Wrench out = *msg;

    if (enabled_.load()) {
      Eigen::Vector3f tau_tilde;
      TorqueDhat dhat;
      core_.update((float)dt, tau_des, omega_, tau_tilde, dhat, compensate_flag_);

      out.moment[0] = tau_tilde(0);
      out.moment[1] = tau_tilde(1);
      out.moment[2] = tau_tilde(2);

      publishDhat(dhat.xyz);
    } else {
      publishDhat(Eigen::Vector3f::Zero());
    }

    pub_wrench_->publish(out);
  }

  // topics
  std::string in_wrench_topic_, out_wrench_topic_, state_topic_, enable_topic_, dhat_topic_;

  // timing
  double default_dt_{1.0/400.0}, dt_min_{1e-4}, dt_max_{0.05};
  std::optional<rclcpp::Time> last_time_;

  // core
  TorqueDOBCore core_{TorqueDOBCore::Params{}};
  bool reset_on_enable_{true};
  bool compensate_flag_{true};

  // state
  std::atomic<bool> have_state_{false};
  Eigen::Vector3f omega_{Eigen::Vector3f::Zero()};

  // enable
  std::atomic<bool> enabled_{false};

  // ROS
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<palletrone_interfaces::msg::Wrench>::SharedPtr sub_wrench_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_dhat_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueDobNode>());
  rclcpp::shutdown();
  return 0;
}