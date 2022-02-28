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

BaseConfig::BaseConfig() : can("can0"), error_heal_time_s(1.0F) {}
BaseConfig::BaseConfig(std::string& can, float error_heal_time_s) : can(can), error_heal_time_s(error_heal_time_s) {}

BaseController::BaseController(BaseConfig& config) : _config(config) {}

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
        _en_drives = true;
    } else {
        std::stringstream desc;
        desc << "Transition to BASE_STS_DRIVES_ENABLED from '" << getBaseStateDesc(_active_state) << "' not possible!";
        throw std::runtime_error(desc.str());
    }
}

void BaseController::disableDrives() {
    if (_active_state == BASE_STS_DRIVES_ENABLED) {
        _en_drives = false;
    } else {
        std::stringstream desc;
        desc << "Transition to BASE_STS_DRIVES_IDLE from '" << getBaseStateDesc(_active_state) << "' not possible!";
        throw std::runtime_error(desc.str());
    }
}

void BaseController::setCmdVel(const float linear, const float angular) {
    // TODO implement kinematics
    (void)angular;

    if (_active_state == BASE_STS_DRIVES_ENABLED) {
        for (auto& drive : _drives) {
            if (drive) {
                drive->setSpeedRPM(linear);
            }
        }
    }
}

void BaseController::stepStateMachine() {
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

    unsigned int drive_id = {0U};
    for (auto& drive : _drives) {
        if (drive) {
            if (!drive->isConnected()) {
                RCLCPP_WARN(rclcpp::get_logger(_logger), "Drive %i not connected!", drive_id);
                drives_connected = false;
            }
        } else {
            drives_connected = false;
        }
        drive_id++;
    };

    return drives_connected;
}

void BaseController::setNewState(const BaseState state) {
    RCLCPP_INFO(rclcpp::get_logger(_logger), "Transition from ['%s'] to ['%s']",
                getBaseStateDesc(_active_state).c_str(), getBaseStateDesc(state).c_str());
    _active_state = state;
}

void BaseController::setErrorState(const std::string desc) {
    RCLCPP_WARN(rclcpp::get_logger(_logger), "Transition from ['%s'] to ['%s']",
                getBaseStateDesc(_active_state).c_str(), getBaseStateDesc(BASE_STS_ERROR).c_str());
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
                    error_desc << "\nFailed to communicate with drive ['" << +drive_id << "']! Drive connected?\n";
                }
            }
        } catch (...) {
            drives_ok = false;
            error_desc << "\nFailed to initialize drive ['" << +drive_id << "']! Drive connected?\n";
        }

        drive_id++;
    }

    /* Reset variables */
    _en_drives = false;

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

    if (_en_drives) {
        drives_enabled = true;

        for (auto& drive : _drives) {
            drive->enable();
        };
    }

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
    if (!_en_drives) {
        for (auto& drive : _drives) {
            drive->disable();
        };
    }

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
    }
}