/**
 * @file base_controller.cpp
 * @author Martin Bauernschmitt
 * @brief Main class of robot base controller
 *
 * @version 0.1
 * @date 2022-02-17
 *
 * @copyright Copyright (c) 2022 - BSD-3-clause - FRANCOR e.V.
 *
 */

#include "francor_frank_base/base_controller.h"

#include <sstream>

#include "rclcpp/rclcpp.hpp"

using namespace francor::can;

std::string getBaseStateDesc(const BaseState state) {
  std::string desc;
  switch (state) {
    case BASE_STS_INIT:
      desc = "BASE_STS_INIT";
      break;
    case BASE_STS_OPEN_COM:
      desc = "BASE_STS_OPEN_COM";
      break;
    case BASE_STS_DRIVES_INIT:
      desc = "BASE_STS_DRIVES_INIT";
      break;
    case BASE_STS_DRIVES_IDLE:
      desc = "BASE_STS_DRIVES_IDLE";
      break;
    case BASE_STS_DRIVES_ENABLED:
      desc = "BASE_STS_DRIVES_ENABLED";
      break;
    case BASE_STS_ERROR:
      desc = "BASE_STS_ERROR";
      break;
    default:
      desc = "BASE_STS_UKNOWN";
      break;
  }
  return desc;
}

std::string getDriveIDStr(const BaseDriveID id) {
  std::string desc;
  switch (id) {
    case BASE_DRIVE_FRONT_LEFT:
      desc = "drive_front_left";
      break;
    case BASE_DRIVE_FRONT_RIGHT:
      desc = "drive_front_right";
      break;
    case BASE_DRIVE_REAR_LEFT:
      desc = "drive_rear_left";
      break;
    case BASE_DRIVE_REAR_RIGHT:
      desc = "drive_rear_right";
      break;
  }
  return desc;
}

BaseConfig::BaseConfig() : can("can0"), auto_enable_on_start(false), auto_enable(false), error_heal_time_s(1.0F) {}
BaseConfig::BaseConfig(std::string& can, bool auto_enable_on_start, bool auto_enable, float error_heal_time_s)
    : can(can),
      auto_enable_on_start(auto_enable_on_start),
      auto_enable(auto_enable),
      error_heal_time_s(error_heal_time_s) {}

BaseController::BaseController(BaseConfig& config, BaseChassisParams& chassis_params)
    : _config(config), _chassis_params(chassis_params) {
  this->buildKinematic();
}

BaseController::~BaseController() {
  if (_can_if) {
    for (auto& drive : _drives) {
      try {
        if (drive) {
          drive->disable();
        }
      } catch (...) {
      }
    }
  }
}

void BaseController::enableDrives() {
  if (_active_state == BASE_STS_DRIVES_IDLE) {
    _user_enable_request = true;
    _user_disable_request = false;
  } else {
    std::stringstream desc;
    desc << "Transition to BASE_STS_DRIVES_ENABLED from '" << getBaseStateDesc(_active_state) << "' not possible!";
    throw std::runtime_error(desc.str());
  }
}

void BaseController::disableDrives() {
  if (_active_state == BASE_STS_DRIVES_ENABLED) {
    _user_disable_request = true;
    _user_enable_request = false;
  } else {
    std::stringstream desc;
    desc << "Transition to BASE_STS_DRIVES_IDLE from '" << getBaseStateDesc(_active_state) << "' not possible!";
    throw std::runtime_error(desc.str());
  }
}

void BaseController::resetOdometry() {
  this->_velocity = Eigen::Vector3f();
  this->_pose = Eigen::Vector3f();
}

void BaseController::setAccelLimit(float accel_limit) { _accel_limit_req = accel_limit; }

void BaseController::setCmdVel(BaseCmdVel cmd_vel) { _cmd_vel_req = cmd_vel; }

void BaseController::updateOdometry(float dT) {
  if (_active_state == BASE_STS_DRIVES_ENABLED || _active_state == BASE_STS_DRIVES_IDLE) {
    try {
      Eigen::Vector4f speed_radps;
      speed_radps(0) = _drives.at(BASE_DRIVE_FRONT_LEFT)->getCurrentSpeedRPM();
      speed_radps(1) = _drives.at(BASE_DRIVE_REAR_LEFT)->getCurrentSpeedRPM();
      speed_radps(2) = _drives.at(BASE_DRIVE_FRONT_RIGHT)->getCurrentSpeedRPM();
      speed_radps(3) = _drives.at(BASE_DRIVE_REAR_RIGHT)->getCurrentSpeedRPM();
      speed_radps *= ((2.0F * M_PI) / 60.0F);

      this->_velocity = _chassis_params.kinematic_matrix_inv * speed_radps * _chassis_params.odom_factor;
      this->_velocity(2) *= -1.0F;

      Eigen::Vector3f dPose = this->_velocity * dT;

      this->_pose(2) = this->_pose(2) + dPose.z();

      if (this->_pose.z() > (2.0F * M_PI)) {
        this->_pose(2) = this->_pose(2) - 2.0F * M_PI;
      }

      if (this->_pose.z() < 0.0F) {
        this->_pose(2) = this->_pose(2) + 2.0F * M_PI;
      }

      this->_pose(0) = this->_pose.x() + dPose.x() * cos(this->_pose.z());
      this->_pose(1) = this->_pose.y() + dPose.x() * sin(this->_pose.z());
    } catch (can_exception& e) {
      setErrorState("Error updating odometry!");
    }
  } else {
    this->_velocity = Eigen::Vector3f();
  }
}

void BaseController::stepStateMachine() {
  runDriveStsHandling();

  switch (_active_state) {
    case BASE_STS_INIT:
      runStsInit();
      break;
    case BASE_STS_OPEN_COM:
      runStsOpenCom();
      break;
    case BASE_STS_DRIVES_INIT:
      runStsDrivesInit();
      break;
    case BASE_STS_DRIVES_IDLE:
      runStsDrivesIdle();
      break;
    case BASE_STS_DRIVES_ENABLED:
      runStsDrivesEnabled();
      break;
    case BASE_STS_ERROR:
      runStsError();
      break;
  }
}

std::shared_ptr<francor::drive::Drive> BaseController::getDrive(uint8_t idx) { return _drives.at(idx); }

Eigen::Vector3f BaseController::getPose() const { return _pose; }

Eigen::Vector3f BaseController::getVelocity() const { return _velocity; }

bool BaseController::isCANRunning() {
  bool can_running = {false};

  if (_can_if) {
    try {
      can_running = _can_if->isDeviceUp();
    } catch (const can_exception& e) {
      setErrorState(e.what());
    }
  }

  return can_running;
}

bool BaseController::allDrivesConnected() {
  bool drives_connected = {true};

  try {
    unsigned int drive_id = {0U};
    for (auto& drive : _drives) {
      if (drive) {
        if (!drive->isConnected()) {
          RCLCPP_WARN(rclcpp::get_logger(_logger), "Drive ['%i-%s'] not connected!", drive_id,
                      getDriveIDStr(static_cast<BaseDriveID>(drive_id)).c_str());
          drives_connected = false;
        }
      } else {
        drives_connected = false;
      }
      drive_id++;
    };
  } catch (const can_exception& e) {
    setErrorState("Error checking if all drives are connected!");
  }

  return drives_connected;
}

void BaseController::buildKinematic() {
  const float l_x = _chassis_params.wheel_separation_x_m;
  const float l_y = _chassis_params.wheel_separation_y_m;
  const float wheel_radius = _chassis_params.wheel_diameter_m * 0.5F;
  const float l_squared = l_x * l_x + l_y * l_y;

  _chassis_params.kinematic_matrix.resize(4, 3);
  _chassis_params.kinematic_matrix << 1.0f, 0.0f, l_squared / (2.0f * l_y), 1.0f, 0.0f, l_squared / (2.0f * l_y), -1.0f,
      0.0f, l_squared / (2.0f * l_y), -1.0f, 0.0f, l_squared / (2.0f * l_y);
  _chassis_params.kinematic_matrix *= 1.0f / wheel_radius;

  _chassis_params.kinematic_matrix_inv =
      _chassis_params.kinematic_matrix.completeOrthogonalDecomposition().pseudoInverse();
}

void BaseController::runDriveStsHandling() {
  const bool auto_enable = ((_config.auto_enable_on_start && _initial_startup) || (_config.auto_enable));

  if (_active_state == BASE_STS_DRIVES_IDLE) {
    _en_drives = (auto_enable && !_user_disable_request) || _user_enable_request;
  } else if (_active_state == BASE_STS_DRIVES_ENABLED) {
    _en_drives = (!_user_disable_request);
  }
}

void BaseController::setNewState(const BaseState state) {
  RCLCPP_INFO(rclcpp::get_logger(_logger), "Transition from ['%s'] to ['%s']", getBaseStateDesc(_active_state).c_str(),
              getBaseStateDesc(state).c_str());
  _active_state = state;
}

void BaseController::setErrorState(const std::string desc) {
  RCLCPP_WARN(rclcpp::get_logger(_logger), "Transition from ['%s'] to ['%s']", getBaseStateDesc(_active_state).c_str(),
              getBaseStateDesc(BASE_STS_ERROR).c_str());
  RCLCPP_WARN(rclcpp::get_logger(_logger), "Error description: %s", desc.c_str());
  RCLCPP_WARN(rclcpp::get_logger(_logger), "Retry executing state ['%s'] after %f second(s)",
              getBaseStateDesc(_active_state).c_str(), _config.error_heal_time_s);

  /* Cleanup */
  _can_if.reset();

  for (auto& drive : _drives) {
    drive.reset();
  }

  _pre_error_state = _active_state;
  _error_time_point = std::chrono::steady_clock::now();
  _active_state = BASE_STS_ERROR;
}

void BaseController::runStsInit() {
  /* Transition handling */
  setNewState(BASE_STS_OPEN_COM);
}

void BaseController::runStsOpenCom() {
  bool can_ok = {false};

  try {
    _can_if = std::make_shared<SocketCAN>(_config.can);
    can_ok = true;
  } catch (const can_exception& e) {
    _can_if.reset();
    can_ok = false;
  }

  /* Transition handling */
  if (can_ok) {
    if (_can_if->isDeviceUp()) {
      setNewState(BASE_STS_DRIVES_INIT);
    } else {
      setErrorState("CAN device is not UP!");
    }
  } else {
    setErrorState("Failed to open CAN interface! Check if can adapter is connected and up!");
  }
}

void BaseController::runStsError() {
  bool error_healed = {false};

  std::chrono::duration<float> error_state_time =
      std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::steady_clock::now() - _error_time_point);

  if (error_state_time.count() >= _config.error_heal_time_s) {
    error_healed = true;
  }

  /* Transition handling */
  if (error_healed) {
    setNewState(BASE_STS_OPEN_COM);
  }
}

void BaseController::runStsDrivesInit() {
  std::stringstream error_desc;
  bool drives_ok = {true};

  /* Init drives */
  unsigned int drive_id = {0U};
  for (auto& drive : _drives) {
    try {
      auto rmd_x8_drive = std::make_shared<francor::drive::RMDX8Drive>(drive_id, _can_if);
      drive = rmd_x8_drive;

      if (drive) {
        if (drive->isConnected()) {
          drive->disable();
        } else {
          drives_ok = false;
          error_desc << "\nFailed to communicate with drive ['" << +drive_id << "-"
                     << getDriveIDStr(static_cast<BaseDriveID>(drive_id)) << "']! Drive connected?\n";
        }
      }
    } catch (...) {
      drives_ok = false;
      error_desc << "\nFailed to initialize drive ['" << +drive_id << "-"
                 << getDriveIDStr(static_cast<BaseDriveID>(drive_id)) << "']! Drive connected?\n";
    }

    drive_id++;
  }

  /* Reset variables */
  _en_drives = false;
  _active_accel_limit = 0.0F;

  /* Transition handling */
  const bool can_running = isCANRunning();
  const bool transition_ok = can_running && drives_ok;

  if (transition_ok) {
    setNewState(BASE_STS_DRIVES_IDLE);
  } else {
    if (!can_running) {
      setErrorState("CAN device failure! (Maybe not connected or down!)");
    }
    if (!drives_ok) {
      setErrorState(error_desc.str());
    }
  }
}

void BaseController::runStsDrivesIdle() {
  bool drives_enabled = {false};

  /* Update config */
  updateAccelLimit();

  /* React to requests */
  if (_en_drives) {
    enableAllDrives();
    drives_enabled = true;
  }

  /* Reset variables */
  _cmd_vel_req = BaseCmdVel();

  /* Transition handling */
  const bool can_running = isCANRunning();
  const bool drives_connected = allDrivesConnected();
  const bool transition_ok = can_running && drives_enabled && drives_connected;

  if (transition_ok) {
    setNewState(BASE_STS_DRIVES_ENABLED);
  } else {
    if (!can_running) {
      setErrorState("CAN device failure! (Maybe not connected or down!)");
    }
    if (!drives_connected) {
      setErrorState("One or more drives are not responding or connected!");
    }
  }
}

void BaseController::runStsDrivesEnabled() {
  /* Update config */
  updateAccelLimit();

  /* React to requests */
  if (!_en_drives) {
    disableAllDrives();
  }

  /* Update cmd velocity */
  updateCmdVel();

  /* Transition handling */
  const bool can_running = isCANRunning();
  const bool drives_connected = allDrivesConnected();
  const bool transition_ok = can_running && !_en_drives && drives_connected;

  if (transition_ok) {
    setNewState(BASE_STS_DRIVES_IDLE);
  } else {
    if (!can_running) {
      setErrorState("CAN device failure! (Maybe not connected or down!)");
    }
    if (!drives_connected) {
      setErrorState("One or more drives are not responding or connected!");
    }
    if (drives_connected && can_running && _initial_startup) {
      /* Initial startup done */
      _initial_startup = false;
    }
  }
}

void BaseController::enableAllDrives() {
  try {
    for (auto& drive : _drives) {
      drive->enable();
    }
  } catch (can_exception& e) {
    setErrorState("Error enabling all drives!");
  }
}

void BaseController::disableAllDrives() {
  try {
    for (auto& drive : _drives) {
      drive->disable();
    }
  } catch (can_exception& e) {
    setErrorState("Error disabling all drives!");
  }
}

void BaseController::updateAccelLimit() {
  try {
    if (abs(_active_accel_limit - _accel_limit_req) > 0.01F) {
      for (auto& drive : _drives) {
        drive->setAccelleration(_accel_limit_req);
      }
      _active_accel_limit = _accel_limit_req;
    }
  } catch (can_exception& e) {
    setErrorState("Error setting accelleration limit!");
  }
}

void BaseController::updateCmdVel() {
  Eigen::Vector3f velocity_cmd(_cmd_vel_req.getLinearVel(), 0.0F, -_cmd_vel_req.getAngularVel());
  Eigen::VectorXf radps = _chassis_params.kinematic_matrix * velocity_cmd.matrix();

  const float speed_left = radps(0) * (60.0F / (2.0F * M_PI));
  const float speed_right = radps(2) * (60.0F / (2.0F * M_PI));

  try {
    // if (_cmd_vel_actv != _cmd_vel_req) {
    _drives.at(BASE_DRIVE_FRONT_LEFT)->setSpeedRPM(speed_left);
    _drives.at(BASE_DRIVE_REAR_LEFT)->setSpeedRPM(speed_left);

    _drives.at(BASE_DRIVE_FRONT_RIGHT)->setSpeedRPM(speed_right);
    _drives.at(BASE_DRIVE_REAR_RIGHT)->setSpeedRPM(speed_right);

    //_cmd_vel_actv = _cmd_vel_req;
    //}
  } catch (can_exception& e) {
    setErrorState("Error setting speed!");
  }
}
