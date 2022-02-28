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

#include <chrono>
#include <string>

constexpr std::size_t BASE_NUM_DRIVES = {1U};

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
    BaseConfig(std::string& can, float error_heal_time_s);

    std::string can;
    float error_heal_time_s;
};

class BaseController {
   public:
    BaseController() = default;
    explicit BaseController(BaseConfig& config);
    ~BaseController();

    void enableDrives();
    void disableDrives();
    void resetErrors();
    void setCmdVel(float linear, float angular);

    void stepStateMachine();

    std::shared_ptr<francor::drive::Drive> getDrive(uint8_t idx);

    bool isCANRunning();
    bool allDrivesConnected();

   private:
    void setNewState(BaseState state);

    void setErrorState(std::string desc);

    void runStsInit();
    void runStsOpenCom();
    void runStsDrivesInit();
    void runStsDrivesIdle();
    void runStsDrivesEnabled();
    void runStsError();

    bool _en_drives = {false};
    bool _reset_errors = {false};

    BaseConfig _config = {};
    BaseState _active_state = {BASE_STS_INIT};

    BaseState _pre_error_state = {BASE_STS_INIT};
    std::chrono::steady_clock::time_point _error_time_point = {};

    std::shared_ptr<francor::can::CAN> _can_if = {};
    std::array<std::shared_ptr<francor::drive::Drive>, BASE_NUM_DRIVES> _drives = {};

    const std::string _logger = {"FrancorBaseController"};
};

std::string getBaseStateDesc(BaseState state);
std::string getDriveIDStr(BaseDriveID id);