/**
 * @file base_controller.h
 * @author Martin Bauernschmitt
 * @brief Main class of robot base controller
 *
 * @version 0.1
 * @date 2022-02-17
 *
 * @copyright Copyright (c) 2022 - BSD-3-clause - FRANCOR e.V.
 *
 */

#pragma once

#include <francor_can/socketcan.h>
#include <francor_drive/rmd_x8_drive.h>

#include <Eigen/Dense>
#include <chrono>
#include <string>

constexpr std::size_t BASE_NUM_DRIVES = {4U};

enum BaseDriveID {
  BASE_DRIVE_FRONT_LEFT = 0,
  BASE_DRIVE_FRONT_RIGHT = 1,
  BASE_DRIVE_REAR_LEFT = 2,
  BASE_DRIVE_REAR_RIGHT = 3,
};

enum BaseState {
  BASE_STS_INIT = 0,
  BASE_STS_OPEN_COM,
  BASE_STS_DRIVES_INIT,
  BASE_STS_DRIVES_IDLE,
  BASE_STS_DRIVES_ENABLED,
  BASE_STS_ERROR,
};

struct BaseConfig {
  BaseConfig();
  BaseConfig(std::string& can, bool auto_enable_on_start, bool auto_enable, float error_heal_time_s);

  std::string can;
  bool auto_enable_on_start;
  bool auto_enable;
  float error_heal_time_s;
};

class BaseCmdVel {
 public:
  BaseCmdVel() = default;
  BaseCmdVel(const BaseCmdVel& cmd_vel) = default;
  explicit BaseCmdVel(const float linear_vel, const float angular_vel)
      : _linear_vel(linear_vel), _angular_vel(angular_vel) {}

  auto getLinearVel() const { return _linear_vel; }
  auto getAngularVel() const { return _angular_vel; }

  auto operator==(BaseCmdVel& rhs) const {
    return ((abs(_linear_vel - rhs._linear_vel) < 0.0001F) && (abs(_angular_vel - rhs._angular_vel) < 0.0001F));
  }

  auto operator!=(BaseCmdVel& rhs) const {
    return !((abs(_linear_vel - rhs._linear_vel) < 0.0001F) && (abs(_angular_vel - rhs._angular_vel) < 0.0001F));
  }

 private:
  float _linear_vel = {0.0F};
  float _angular_vel = {0.0F};
};

struct BaseChassisParams {
  BaseChassisParams() = default;
  BaseChassisParams(double gear_ratio, double wheel_diameter_m, double wheel_separation_x_m,
                    double wheel_separation_y_m)
      : gear_ratio(gear_ratio),
        wheel_diameter_m(wheel_diameter_m),
        wheel_separation_x_m(wheel_separation_x_m),
        wheel_separation_y_m(wheel_separation_y_m) {}

  float gear_ratio = {1.0};
  float wheel_diameter_m = {0.0};
  float wheel_separation_x_m = {0.0};
  float wheel_separation_y_m = {0.0};

  Eigen::MatrixXf kinematic_matrix = {};
  Eigen::MatrixXf kinematic_matrix_inv = {};
};

class BaseController {
 public:
  BaseController() = default;
  explicit BaseController(BaseConfig& config, BaseChassisParams& chassis_params);
  ~BaseController();

  void enableDrives();
  void disableDrives();
  void resetErrors();
  void resetOdometry();

  void setAccelLimit(float accel_limit);
  void setCmdVel(BaseCmdVel cmd_vel);
  void updateOdometry(float dT);

  void stepStateMachine();

  std::shared_ptr<francor::drive::Drive> getDrive(uint8_t idx);

  Eigen::Vector3f getPose() const;
  Eigen::Vector3f getVelocity() const;

  bool isCANRunning();
  bool allDrivesConnected();

 private:
  void buildKinematic();

  void runDriveStsHandling();
  void setNewState(BaseState state);

  void setErrorState(std::string desc);

  void runStsInit();
  void runStsOpenCom();
  void runStsDrivesInit();
  void runStsDrivesIdle();
  void runStsDrivesEnabled();
  void runStsError();

  void enableAllDrives();
  void disableAllDrives();

  void updateAccelLimit();
  void updateCmdVel();

  bool _user_enable_request = {false};
  bool _user_disable_request = {false};

  bool _initial_startup = {true};
  bool _en_drives = {false};
  bool _reset_errors = {false};

  BaseConfig _config = {};
  BaseChassisParams _chassis_params = {};
  BaseState _active_state = {BASE_STS_INIT};

  BaseState _pre_error_state = {BASE_STS_INIT};
  std::chrono::steady_clock::time_point _error_time_point = {};

  std::shared_ptr<francor::can::CAN> _can_if = {};
  std::array<std::shared_ptr<francor::drive::Drive>, BASE_NUM_DRIVES> _drives = {};

  float _accel_limit_req = {0.0F};
  float _active_accel_limit = {0.0F};

  BaseCmdVel _cmd_vel_req = {};
  BaseCmdVel _cmd_vel_actv = {};

  Eigen::Vector3f _velocity = {};
  Eigen::Vector3f _pose = {};

  const std::string _logger = {"FrancorBaseController"};
};

std::string getBaseStateDesc(BaseState state);
std::string getDriveIDStr(BaseDriveID id);