#include <francor_frank_base/base_controller.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class FrankBase : public rclcpp::Node {
   public:
    FrankBase();

   private:
    void declareParams();
    void readParams();
    void createBaseController();

    void base_step_callback();

    std::string _can_name;

    rclcpp::TimerBase::SharedPtr base_step_timer_;
    std::shared_ptr<BaseController> _base_controller;
};

FrankBase::FrankBase() : Node("frank_base") {
    declareParams();
    readParams();
    createBaseController();

    base_step_timer_ = this->create_wall_timer(10ms, std::bind(&FrankBase::base_step_callback, this));
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

void FrankBase::base_step_callback() { this->_base_controller->stepStateMachine(); }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankBase>());
    rclcpp::shutdown();
    return 0;
}
