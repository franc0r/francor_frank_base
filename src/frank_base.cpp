#include <francor_frank_base/base_controller.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/time.hpp>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

constexpr float DFT_ACCEL_LIMIT = 25.0F;  //!< Default accelleration limit rpm/s

using namespace std::chrono_literals;

using BoolSrv = rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr;

using TwistMsg = geometry_msgs::msg::Twist;
using TwistMsgSub = rclcpp::Subscription<TwistMsg>;
using Float32Msg = std_msgs::msg::Float32;
using Float32MsgSub = rclcpp::Subscription<Float32Msg>;
using Float32MsgPub = rclcpp::Publisher<Float32Msg>;
using OdomMsg = nav_msgs::msg::Odometry;
using OdomMsgPub = rclcpp::Publisher<OdomMsg>;
using Clock = std::chrono::steady_clock;
using ClockTimePoint = std::chrono::time_point<std::chrono::steady_clock>;

class FrankBase : public rclcpp::Node {
 public:
  FrankBase();

 private:
  void resetVariables();
  void declareParams();
  void readParams();
  void createBaseController();
  void createServices();
  void createPublishers();
  void createSubscriber();
  void createTimers();

  void publish();

  void cbReadParams();
  void cbBaseStep();
  void cbEnableDrives(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void cbCmdVelocity(TwistMsg::SharedPtr cmd_vel);

  void setCmdVel();

  std::string _can_name;
  BaseChassisParams _chassis_params;
  double _cmd_vel_max_timeout_s;
  bool _en_cmd_vel_timeout;
  OdomMsg _odom_msg;

  rclcpp::TimerBase::SharedPtr _param_read_timer;
  rclcpp::TimerBase::SharedPtr _base_step_timer;

  BoolSrv _enable_drives_srv;
  std::shared_ptr<BaseController> _base_controller;
  std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _speed_pub_lst;
  std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _torque_pub_lst;
  std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _tempC_pub_lst;
  std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _voltV_pub_lst;
  TwistMsgSub::SharedPtr _speed_subs;
  OdomMsgPub::SharedPtr _odom_pub;

  ClockTimePoint _cmd_vel_timestamp;
  std::atomic<float> _cmd_lin_x;
  std::atomic<float> _cmd_ang_z;

  float _accel_limit;
  bool _auto_enable_on_start;
  bool _auto_enable;
};

FrankBase::FrankBase() : Node("frank_base") {
  resetVariables();
  declareParams();
  readParams();
  createBaseController();
  createServices();
  createPublishers();
  createSubscriber();
  createTimers();
}

void FrankBase::resetVariables() {
  _cmd_lin_x = 0.0F;
  _cmd_ang_z = 0.0F;
}

void FrankBase::declareParams() {
  this->declare_parameter<std::string>("can", "can0");
  this->declare_parameter<double>("cmd_vel_max_timeout_s", 1.0);
  this->declare_parameter<bool>("enable_cmd_vel_timeout", true);
  this->declare_parameter<bool>("auto_enable_on_start", false);
  this->declare_parameter<bool>("auto_enable", false);
  this->declare_parameter<float>("accel_limit", DFT_ACCEL_LIMIT);
  this->declare_parameter<float>("gear_ratio", 1.0);
  this->declare_parameter<float>("wheel_diameter_m", 1.0);
  this->declare_parameter<float>("wheel_separation_x_m", 1.0);
  this->declare_parameter<float>("wheel_separation_y_m", 1.0);
}

void FrankBase::readParams() {
  this->get_parameter("can", _can_name);
  this->get_parameter("cmd_vel_max_timeout_s", _cmd_vel_max_timeout_s);
  this->get_parameter("enable_cmd_vel_timeout", _en_cmd_vel_timeout);
  this->get_parameter("auto_enable_on_start", _auto_enable_on_start);
  this->get_parameter("auto_enable", _auto_enable);
  this->get_parameter("accel_limit", _accel_limit);
  this->get_parameter("gear_ratio", _chassis_params.gear_ratio);
  this->get_parameter("wheel_diameter_m", _chassis_params.wheel_diameter_m);
  this->get_parameter("wheel_separation_x_m", _chassis_params.wheel_separation_x_m);
  this->get_parameter("wheel_separation_y_m", _chassis_params.wheel_separation_y_m);

  RCLCPP_INFO(this->get_logger(), "CAN %s", _can_name.c_str());                                              // NOLINT
  RCLCPP_INFO(this->get_logger(), "Cmd Velocity Timeout %f", _cmd_vel_max_timeout_s);                        // NOLINT
  RCLCPP_INFO(this->get_logger(), "Enable Cmd Velocity Timeout %i", static_cast<int>(_en_cmd_vel_timeout));  // NOLINT
  RCLCPP_INFO(this->get_logger(), "Auto enable on start %i", static_cast<int>(_auto_enable_on_start));       // NOLINT
  RCLCPP_INFO(this->get_logger(), "Auto enable %i", static_cast<int>(_auto_enable));                         // NOLINT
  RCLCPP_INFO(this->get_logger(), "Accelleration Limit %.2f", _accel_limit);                                 // NOLINT
  RCLCPP_INFO(this->get_logger(), "gear_ratio %f", _chassis_params.gear_ratio);                              // NOLINT
  RCLCPP_INFO(this->get_logger(), "wheel_diameter_m %f", _chassis_params.wheel_diameter_m);                  // NOLINT
  RCLCPP_INFO(this->get_logger(), "wheel_separation_x_m %f", _chassis_params.wheel_separation_x_m);          // NOLINT
  RCLCPP_INFO(this->get_logger(), "wheel_separation_y_m %f", _chassis_params.wheel_separation_y_m);          // NOLINT

  _cmd_vel_max_timeout_s = 1.0F;
}

void FrankBase::createBaseController() {
  BaseConfig base_config = BaseConfig(_can_name, _auto_enable_on_start, _auto_enable, 1.0F);
  this->_base_controller = std::make_shared<BaseController>(base_config, _chassis_params);
  this->_base_controller->setAccelLimit(_accel_limit);
}

void FrankBase::createServices() {
  _enable_drives_srv = create_service<std_srvs::srv::SetBool>(
      "enable_drives", std::bind(&FrankBase::cbEnableDrives, this, std::placeholders::_1, std::placeholders::_2));
}

void FrankBase::createPublishers() {
  for (auto idx = 0U; idx < BASE_NUM_DRIVES; idx++) {
    _speed_pub_lst.at(idx) = create_publisher<std_msgs::msg::Float32>(
        getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/speed", rclcpp::QoS(1).best_effort());

    _torque_pub_lst.at(idx) = create_publisher<std_msgs::msg::Float32>(
        getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/torque", rclcpp::QoS(1).best_effort());

    _tempC_pub_lst.at(idx) = create_publisher<std_msgs::msg::Float32>(
        getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/temp", rclcpp::QoS(1).best_effort());

    _voltV_pub_lst.at(idx) = create_publisher<std_msgs::msg::Float32>(
        getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/voltage", rclcpp::QoS(1).best_effort());
  }

  _odom_pub = create_publisher<OdomMsg>("odom", rclcpp::QoS(1).best_effort());
}

void FrankBase::publish() {
  if (_base_controller->allDrivesConnected()) {
    for (auto idx = 0U; idx < BASE_NUM_DRIVES; idx++) {
      Float32Msg msg;

      try {
        msg.data = _base_controller->getDrive(idx)->getCurrentSpeedRPM();
        _speed_pub_lst.at(idx)->publish(msg);

        msg.data = _base_controller->getDrive(idx)->getCurrentTorqueNm();
        _torque_pub_lst.at(idx)->publish(msg);

        msg.data = _base_controller->getDrive(idx)->getTempC();
        _tempC_pub_lst.at(idx)->publish(msg);

        msg.data = _base_controller->getDrive(idx)->getVoltageV();
        _voltV_pub_lst.at(idx)->publish(msg);
      } catch (francor::can::can_exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error publishing drive ['%i'] state", idx);  // NOLINT
      }
    }

    _odom_pub->publish(_odom_msg);
  }
}

void FrankBase::createSubscriber() {
  _speed_subs = this->create_subscription<TwistMsg>("cmd_vel", rclcpp::QoS(1).best_effort(),
                                                    std::bind(&FrankBase::cbCmdVelocity, this, std::placeholders::_1));
}

void FrankBase::createTimers() {
  _base_step_timer = this->create_wall_timer(20ms, std::bind(&FrankBase::cbBaseStep, this));
  _param_read_timer = this->create_wall_timer(1000ms, std::bind(&FrankBase::cbReadParams, this));
}

void FrankBase::cbBaseStep() {
  _base_controller->stepStateMachine();
  setCmdVel();
  _base_controller->setAccelLimit(_accel_limit);

  publish();
}

void FrankBase::cbReadParams() { this->get_parameter("accel_limit", _accel_limit); }

void FrankBase::cbEnableDrives(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  try {
    if (request->data) {
      _base_controller->enableDrives();
    } else {
      _base_controller->disableDrives();
    }
    response->success = true;
  } catch (std::runtime_error& e) {
    response->success = false;
    response->message = e.what();
  }
}

void FrankBase::cbCmdVelocity(const TwistMsg::SharedPtr cmd_vel) {
  _cmd_vel_timestamp = Clock::now();
  _cmd_lin_x = static_cast<float>(cmd_vel->linear.x);
  _cmd_ang_z = static_cast<float>(cmd_vel->angular.z);
}

void FrankBase::setCmdVel() {
  const int delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - _cmd_vel_timestamp).count();
  const int max_timeout_ms = static_cast<int>(_cmd_vel_max_timeout_s) * 1000;

  if (delta_ms < max_timeout_ms || !_en_cmd_vel_timeout) {
    _base_controller->setCmdVel(BaseCmdVel(_cmd_lin_x, _cmd_ang_z));
  } else {
    auto steady_clock = rclcpp::Clock();
    _base_controller->setCmdVel(BaseCmdVel(0.0, 0.0));
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Cmd velocity timeout!");  // NOLINT
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankBase>());
  rclcpp::shutdown();
  return 0;
}
