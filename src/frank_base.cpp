#include <francor_frank_base/base_controller.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class FrankBase : public rclcpp::Node {
   public:
    FrankBase();

   private:
    void declareParams();
    void readParams();
    void createBaseController();
    void createServices();

    void cbBaseStep();
    void cbEnableDrives(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    std::string _can_name;

    rclcpp::TimerBase::SharedPtr _base_step_timer;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_drives_srv;
    std::shared_ptr<BaseController> _base_controller;
};

FrankBase::FrankBase() : Node("frank_base") {
    declareParams();
    readParams();
    createBaseController();
    createServices();

    _base_step_timer = this->create_wall_timer(10ms, std::bind(&FrankBase::cbBaseStep, this));
}

void FrankBase::declareParams() { this->declare_parameter<std::string>("can", "can0"); }

void FrankBase::readParams() {
    this->get_parameter("can", _can_name);

    RCLCPP_INFO(this->get_logger(), "CAN %s", _can_name.c_str());  // NOLINT
}

void FrankBase::createBaseController() {
    BaseConfig base_config = BaseConfig(_can_name, 5.0F);
    this->_base_controller = std::make_shared<BaseController>(base_config);
}

void FrankBase::createServices() {
    _enable_drives_srv = create_service<std_srvs::srv::SetBool>(
        "enable_drives", std::bind(&FrankBase::cbEnableDrives, this, std::placeholders::_1, std::placeholders::_2));
}

void FrankBase::cbBaseStep() { this->_base_controller->stepStateMachine(); }

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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankBase>());
    rclcpp::shutdown();
    return 0;
}
