#include <francor_frank_base/base_controller.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

using BoolSrv = rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr;

using TwistMsg = geometry_msgs::msg::Twist;
using TwistMsgSub = rclcpp::Subscription<TwistMsg>;
using Float32Msg = std_msgs::msg::Float32;
using Float32MsgSub = rclcpp::Subscription<Float32Msg>;
using Float32MsgPub = rclcpp::Publisher<Float32Msg>;

class FrankBase : public rclcpp::Node {
   public:
    FrankBase();

   private:
    void declareParams();
    void readParams();
    void createBaseController();
    void createServices();
    void createPublishers();
    void createSubscriber();

    void publish();

    void cbBaseStep();
    void cbEnableDrives(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void cbCmdVelocity(TwistMsg::SharedPtr cmd_vel);

    std::string _can_name;

    rclcpp::TimerBase::SharedPtr _base_step_timer;
    BoolSrv _enable_drives_srv;
    std::shared_ptr<BaseController> _base_controller;
    std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _speed_pub_lst;
    std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _torque_pub_lst;
    std::array<Float32MsgPub::SharedPtr, BASE_NUM_DRIVES> _tempC_pub_lst;
    TwistMsgSub::SharedPtr _speed_subs;
};

FrankBase::FrankBase() : Node("frank_base") {
    declareParams();
    readParams();
    createBaseController();
    createServices();
    createPublishers();
    createSubscriber();

    _base_step_timer = this->create_wall_timer(20ms, std::bind(&FrankBase::cbBaseStep, this));
}

void FrankBase::declareParams() { this->declare_parameter<std::string>("can", "can0"); }

void FrankBase::readParams() {
    this->get_parameter("can", _can_name);

    RCLCPP_INFO(this->get_logger(), "CAN %s", _can_name.c_str());  // NOLINT
}

void FrankBase::createBaseController() {
    BaseConfig base_config = BaseConfig(_can_name, 1.0F);
    this->_base_controller = std::make_shared<BaseController>(base_config);
}

void FrankBase::createServices() {
    _enable_drives_srv = create_service<std_srvs::srv::SetBool>(
        "enable_drives", std::bind(&FrankBase::cbEnableDrives, this, std::placeholders::_1, std::placeholders::_2));
}

void FrankBase::createPublishers() {
    for (auto idx = 0U; idx < BASE_NUM_DRIVES; idx++) {
        _speed_pub_lst.at(idx) =
            create_publisher<std_msgs::msg::Float32>(getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/speed", 1);

        _torque_pub_lst.at(idx) =
            create_publisher<std_msgs::msg::Float32>(getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/torque", 1);

        _tempC_pub_lst.at(idx) =
            create_publisher<std_msgs::msg::Float32>(getDriveIDStr(static_cast<BaseDriveID>(idx)) + "/temp", 1);
    }
}

void FrankBase::publish() {
    if (_base_controller->allDrivesConnected()) {
        for (auto idx = 0U; idx < BASE_NUM_DRIVES; idx++) {
            Float32Msg msg;

            msg.data = _base_controller->getDrive(idx)->getCurrentSpeedRPM();
            _speed_pub_lst.at(idx)->publish(msg);

            msg.data = _base_controller->getDrive(idx)->getCurrentTorqueNm();
            _torque_pub_lst.at(idx)->publish(msg);

            msg.data = _base_controller->getDrive(idx)->getTempC();
            _tempC_pub_lst.at(idx)->publish(msg);
        }
    }
}

void FrankBase::createSubscriber() {
    _speed_subs = this->create_subscription<TwistMsg>(
        "cmd_vel", 1, std::bind(&FrankBase::cbCmdVelocity, this, std::placeholders::_1));
}

void FrankBase::cbBaseStep() {
    this->_base_controller->stepStateMachine();
    publish();
}

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
    _base_controller->setCmdVel(static_cast<float>(cmd_vel->linear.x), static_cast<float>(cmd_vel->angular.z));
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankBase>());
    rclcpp::shutdown();
    return 0;
}
