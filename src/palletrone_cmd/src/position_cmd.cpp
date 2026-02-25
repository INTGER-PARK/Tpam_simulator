#include <chrono>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>

static constexpr double X_CMD = 0.0;
static constexpr double Y_CMD = 0.0;
static constexpr double Z_CMD = 0.0;
static constexpr int    RATE_HZ = 400;
static constexpr double RAMP_TIME_SEC = 20.0;

class PositionCmd : public rclcpp::Node {
public:
  PositionCmd() : rclcpp::Node("position_cmd")
  {
    using palletrone_interfaces::msg::Cmd;

    pub_cmd_ = this->create_publisher<Cmd>("/cmd", 10);


    z0_ = 0.0;
    t0_ = std::chrono::steady_clock::now();

    const double dt = 1.0 / (RATE_HZ > 0 ? RATE_HZ : 1);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(dt));
    timer_ = this->create_wall_timer(period, std::bind(&PositionCmd::onTick, this));
  }

private:
  void onTick()
  {
    using palletrone_interfaces::msg::Cmd;

    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - t0_).count();

    double s = (RAMP_TIME_SEC > 0.0) ? (t / RAMP_TIME_SEC) : 1.0;
    s = std::clamp(s, 0.0, 1.0);

    double z_ref = z0_ + (Z_CMD - z0_) * s;

    Cmd msg;
    msg.pos_cmd[0] = static_cast<float>(X_CMD);
    msg.pos_cmd[1] = static_cast<float>(Y_CMD);
    msg.pos_cmd[2] = static_cast<float>(z_ref);

    pub_cmd_->publish(msg);
  }

  rclcpp::Publisher<palletrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point t0_;
  double z0_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCmd>());
  rclcpp::shutdown();
  return 0;
}

