#include <array>
#include <csignal>
#include <iostream>

#include <ros/ros.h>

#include <cyphal/cyphal.h>
#include <cyphal/providers/LinuxCAN.h>
#include <cyphal/allocators/o1/o1_allocator.h>

#include <boost/stacktrace.hpp>

#include "ros_handlers/handlers.h"
#include "cyphal_subscriptions/subscriptions.h"

#include "utils.h"

CyphalInterface* interface;

PowerServicesProvider* power_services          = nullptr;
MotorService* motor_service                    = nullptr;
BatteryService* battery_service                = nullptr;
OdometryService* odometry_service              = nullptr;
HeartbeatService* hbeat_service                = nullptr;
HMIHandler* hmi_handler                        = nullptr;
ResetDriveProvider* reset_provider             = nullptr;
std::array<WheelVelHandler*, 4> wheel_handlers = {nullptr, nullptr, nullptr, nullptr};

#define DELETE_SAFE(ptr) {delete ptr; ptr = nullptr;}

void error_handler() {std::cout << "Error in libcyhpal: " << std::endl << boost::stacktrace::stacktrace(); exit(1);}

uint32_t uptime = 0;
PREPARE_MESSAGE(uavcan_node_Heartbeat_1_0, hbeat)
void heartbeat() {
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    interface->SEND_MSG(uavcan_node_Heartbeat_1_0, &heartbeat_msg, hbeat_buf, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
    uptime += 1;
}

static bool is_terminating = false;
void signalFinalizer(int signum) {
    if (is_terminating) {
        return;
    }
    is_terminating = true;

    std::cout << "Recieved termination signal" << std::endl;

    std::cout << "Attempting to delete motor_service" << std::endl;
    DELETE_SAFE(motor_service);
    std::cout << "Attempting to delete battery_service" << std::endl;
    DELETE_SAFE(battery_service);
    std::cout << "Attempting to delete odometry_service" << std::endl;
    DELETE_SAFE(odometry_service);
    std::cout << "Attempting to delete hmi_handler" << std::endl;
    DELETE_SAFE(hmi_handler);
    std::cout << "Attempting to delete reset_provider" << std::endl;
    DELETE_SAFE(reset_provider);
    std::cout << "Attempting to delete power_services" << std::endl;
    DELETE_SAFE(power_services);

    std::cout << "Checking if cyphal is running" << std::endl;
    if (interface->is_up()) {
        std::cout << "Processing last CAN TX messages. Deadline: +5s" << std::endl;
        uint64_t start = timeMillis();
        uint64_t now = start;
        while (interface->has_unsent_frames() && (now - start) < 5000) {
            interface->process_tx_once();
            now = timeMillis();
        }
    }

    ros::shutdown();

    exit(signum);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle ros_node;

    signal(SIGINT, signalFinalizer);
    signal(SIGTERM, signalFinalizer);

    interface = new CyphalInterface(99);
    interface->setup<LinuxCAN, O1Allocator>("can0");

    motor_service = new MotorService(interface);
    battery_service = new BatteryService(interface, ros_node);
    odometry_service = new OdometryService(interface, ros_node);
    hbeat_service = new HeartbeatService(interface, hmi_handler);

    hmi_handler = new HMIHandler(interface, ros_node, "/hmi/beeper", "/hmi/led", 9);
    reset_provider = new ResetDriveProvider(ros_node, motor_service);
    power_services = new PowerServicesProvider(interface, ros_node);

    WheelVelHandler handler_bl(ros_node, "wheel_vel/bl", CID_BL, motor_service);
    WheelVelHandler handler_br(ros_node, "wheel_vel/br", CID_BR, motor_service, true);
    WheelVelHandler handler_fr(ros_node, "wheel_vel/fr", CID_FR, motor_service, true);
    WheelVelHandler handler_fl(ros_node, "wheel_vel/fl", CID_FL, motor_service);
    wheel_handlers = {&handler_br, &handler_bl, &handler_fr, &handler_fl};

    uint64_t last_heartbeat_time = 0;
    uint64_t last_odom_time = 0;
    uint64_t last_down_check = 0;

    motor_service->send_speed_all(0);
    motor_service->send_state_all(1);

    hmi_handler->last_color = {0, 255, 0};
    hmi_handler->send_beep(0.5, 10);

    while (ros::ok()) {
        uint64_t tick = timeMillis();
        EACH_N_TICKS(1000, last_heartbeat_time, {
            heartbeat();
        })
        EACH_N_TICKS(500, last_odom_time, {
            odometry_service->publish();
        })
        EACH_N_TICKS(500, last_down_check, {
            if (hbeat_service->is_anyone_down(tick)) {
                hmi_handler->send_color(255, 0, 0);
                hmi_handler->send_beep(1, 1);
                power_services->call_reboot();
            }
            else {
                hmi_handler->reset_color();
            }
        })

        for (WheelVelHandler* handler: wheel_handlers) {
            handler->zeroing_check(tick);
        }

        interface->loop();
        ros::spinOnce();
    }

    return 0;
}
