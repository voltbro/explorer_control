#include <cmath>

#include <uavcan/node/ExecuteCommand_1_1.h>
#include <voltbro/hmi/beeper_service_1_0.h>
#include <voltbro/hmi/led_service_1_0.h>

#include "handlers.h"
#include "utils.h"

void WheelVelHandler::callback(const std_msgs::Float64::ConstPtr& msg) {
    double speed = msg->data;
    if (is_inverted) speed = -speed;

    //ROS_INFO("Setting speed for [%d]: [%f]", target_id, speed);
    service->send_speed(target_id, speed);

    last_update = timeMillis();
    is_zeroed = false;
}

void WheelVelHandler::zeroing_check(uint64_t current_time) {
    if ((current_time - last_update) < ZEROING_MILLIS || is_zeroed) {
        return;
    }
    service->send_speed(target_id, 0);
    is_zeroed = true;
}

static uint8_t beeper_buf[voltbro_hmi_beeper_service_Request_1_0_EXTENT_BYTES_];
static CanardTransferID beeper_transfer_id = 0;
#define SRV_HMI_BEEPER_PORT 258
void HMIHandler::send_beep(float duration, float frequency) {
    voltbro_hmi_beeper_service_Request_1_0 beeper_msg = {0};
    beeper_msg.frequency.hertz = frequency;
    beeper_msg.duration.second = duration;

    interface->SEND_TRANSFER(
        voltbro_hmi_beeper_service_Request_1_0,
        &beeper_msg,
        beeper_buf,
        SRV_HMI_BEEPER_PORT,
        &beeper_transfer_id,
        CanardPriorityNominal,
        CanardTransferKindRequest,
        target_id
    );
}

void HMIHandler::callback(const cyphal_bridge::HMIBeeper::ConstPtr& msg) {
    send_beep(msg->duration, msg->frequency);
}

static uint8_t led_buf[voltbro_hmi_led_service_Request_1_0_EXTENT_BYTES_];
static CanardTransferID led_transfer_id = 0;
#define SRV_HMI_LED_PORT 172
void HMIHandler::send_color(uint8_t r, uint8_t g, uint8_t b) {
    voltbro_hmi_led_service_Request_1_0 led_msg = {0};
    led_msg.r.value = r;
    led_msg.g.value = g;
    led_msg.b.value = b;

    interface->SEND_TRANSFER(
        voltbro_hmi_led_service_Request_1_0,
        &led_msg,
        led_buf,
        SRV_HMI_LED_PORT,
        &led_transfer_id,
        CanardPriorityNominal,
        CanardTransferKindRequest,
        target_id
    );
}

HMIHandler::~HMIHandler() {
    send_color(255, 0, 0);
    send_beep(1, 1);
}

void HMIHandler::reset_color() {
    send_color(last_color[0], last_color[1], last_color[2]);
}

void HMIHandler::callback(const cyphal_bridge::HMILed::ConstPtr& msg) {
    last_color[0] = msg->r;
    last_color[1] = msg->g;
    last_color[2] = msg->b;
    send_color(msg->r, msg->g, msg->b);
}

bool ResetDriveProvider::callback(
    cyphal_bridge::ResetDrive::Request& request,
    cyphal_bridge::ResetDrive::Response& response
) {
    // TODO: real who responded
    response.has_responded[0] = false;
    response.has_responded[1] = false;
    response.has_responded[2] = false;
    response.has_responded[3] = false;

    switch (request.drive) {
        case 0:
            ROS_INFO("Resetting all drives");
            motor_service->send_state_all(false);
            motor_service->send_speed_all(0);
            motor_service->send_state_all(true);
            response.has_responded[0] = response.has_responded[1] = response.has_responded[2] = response.has_responded[3] = true;
            break;
        case 2:
        case 4:
        case 8:
        case 16:
            ROS_INFO("Resetting drive <%d>", request.drive);
            motor_service->send_state(request.drive, false);
            motor_service->send_speed(request.drive, 0);
            motor_service->send_state(request.drive, true);
            response.has_responded[(int)std::log2(request.drive) - 1] = true;
            break;
        default:
            return false;
    }
    return true;
}

static uint8_t command_buf[uavcan_node_ExecuteCommand_Request_1_1_EXTENT_BYTES_];
static CanardTransferID command_transfer_id = 0;
void PowerServicesProvider::call_reboot() {
    uavcan_node_ExecuteCommand_Request_1_1 command = {0};
    command.command = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART;

    interface->SEND_TRANSFER(
        uavcan_node_ExecuteCommand_Request_1_1,
        &command,
        command_buf,
        uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_,
        &command_transfer_id,
        CanardPriorityNominal,
        CanardTransferKindRequest,
        CID_PWR
    );
}

bool PowerServicesProvider::callback(
    cyphal_bridge::PowerReset::Request& request,
    cyphal_bridge::PowerReset::Response& response
) {
    // TODO: real response
    call_reboot();
    response.ok = true;
    return true;
}
