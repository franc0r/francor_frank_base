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
    switch (state) {
        case BASE_STS_INIT:
            return "BASE_STS_INIT";
        case BASE_STS_OPEN_COM:
            return "BASE_STS_OPEN_COM";
        case BASE_STS_DRIVES_INIT:
            return "BASE_STS_DRIVES_INIT";
        case BASE_STS_DRIVES_IDLE:
            return "BASE_STS_DRIVES_IDLE";
        case BASE_STS_DRIVES_ENABLED:
            return "BASE_STS_DRIVES_ENABLED";
        case BASE_STS_ERROR:
            return "BASE_STS_ERROR";
        default:
            return "BASE_STS_UKNOWN";
    }
}

BaseConfig::BaseConfig() : can("can0"), error_heal_time_s(1.0F) {}
BaseConfig::BaseConfig(std::string& can, float error_heal_time_s) : can(can), error_heal_time_s(error_heal_time_s) {}

BaseController::BaseController(BaseConfig& config) : _config(config) {}

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

auto BaseController::isCANRunning() {
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
            drive = std::make_shared<francor::drive::RMDX8Drive>(drive_id, _can_if);

            if (drive) {
                if (!drive->isConnected()) {
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
    bool drives_enabled = false;

    if (_en_drives) {
        drives_enabled = true;

        unsigned int drive_id = {0U};
        for (auto& drive : _drives) {
            (void)drive;
            drive_id++;
        };
    }

    /* Transition handling */
    const bool can_running = isCANRunning();
    const bool transition_ok = can_running && drives_enabled;

    if (transition_ok) {
        setNewState(BASE_STS_DRIVES_ENABLED);
    } else {
        if (!can_running) {
            setErrorState("CAN device failure! (Maybe not connected or down!)");
        }
    }
}

void BaseController::runStsDrivesEnabled() {
    /* Transition handling */
    const bool can_running = isCANRunning();
    const bool transition_ok = can_running && !_en_drives;

    if (transition_ok) {
        setNewState(BASE_STS_DRIVES_IDLE);
    } else {
        if (!can_running) {
            setErrorState("CAN device failure! (Maybe not connected or down!)");
        }
    }
}