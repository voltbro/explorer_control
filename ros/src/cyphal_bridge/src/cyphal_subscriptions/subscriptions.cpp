#include "subscriptions.h"

#include <algorithm>
#include <iostream>
#include <string>

#include "utils.h"

PREPARE_MESSAGE(uavcan_register_Access_Request_1_0, reg_access)

#define ACCESS_REG(obj, node_id)            \
    interface->SEND_TRANSFER(               \
        uavcan_register_Access_Request_1_0, \
        &obj,                               \
        reg_access_buf,                     \
        port_id,                            \
        &reg_access_transfer_id,            \
        CanardPriorityNominal,              \
        CanardTransferKindRequest,          \
        node_id                             \
    )

void MotorService::send_state(CanardNodeID node_id, uint8_t state) {
    uavcan_register_Access_Request_1_0 request{0};
    sprintf((char*)request.name.name.elements, "motor.is_on");
    request.name.name.count = strlen((char*)request.name.name.elements);

    uavcan_register_Value_1_0 value = {};
    value._tag_ = 11;

    uavcan_primitive_array_Natural8_1_0 result = {};
    result.value.elements[0] = state;
    result.value.count = 1;

    value.natural8 = result;
    request.value = value;

    ACCESS_REG(request, node_id);
}

void MotorService::send_speed(CanardNodeID node_id, double speed) {
    uavcan_register_Access_Request_1_0 request{0};
    sprintf((char*)request.name.name.elements, "motor.speed");
    request.name.name.count = strlen((char*)request.name.name.elements);

    uavcan_register_Value_1_0 value = {};
    value._tag_ = 12;

    uavcan_primitive_array_Real64_1_0 result = {};
    result.value.elements[0] = speed;
    result.value.count = 1;

    value.real64 = result;
    request.value = value;

    ACCESS_REG(request, node_id);
}

void MotorService::send_speed_all(double speed) {
    for (CanardNodeID node_id : left_nodes) {
        send_speed(node_id, speed);
    }
    for (CanardNodeID node_id : right_nodes) {
        send_speed(node_id, -speed);
    }
}

void MotorService::send_speed_all_raw(double speed) {
    for (CanardNodeID node_id : nodes) {
        send_speed(node_id, speed);
    }
}

void MotorService::send_state_all(uint8_t state) {
    for (CanardNodeID node_id : nodes) {
        send_state(node_id, state);
    }
}

void MotorService::send_off() {
    send_state_all(0);
}

void MotorService::send_on() {
    send_state_all(1);
}

void MotorService::handler(
    const uavcan_register_Access_Response_1_0& reg_response,
    CanardRxTransfer* transfer
) {
    // TODO
}

double WHEEL_RADIUS = 0.18;

OdometryService::OdometryService(CyphalInterface* interface, ros::NodeHandle& node) : AbstractSubscription(
    interface,
    CanardTransferKindMessage,
    6586,
    uavcan_si_unit_angular_velocity_Scalar_1_0_EXTENT_BYTES_
) {
    odom_publisher = node.advertise<nav_msgs::Odometry>("odom", 5);
    imu_subscriber = node.subscribe("imu/data", 1000, &OdometryService::imu_callback, this);
}

void OdometryService::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_q = Eigen::Quaterniond(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    transform.angular_vel_x = msg->angular_velocity.x;
    transform.angular_vel_y = msg->angular_velocity.y;
    transform.angular_vel_z = msg->angular_velocity.z;
}

void OdometryService::handler(
    const uavcan_si_unit_angular_velocity_Scalar_1_0& velocity,
    CanardRxTransfer* transfer
) {
    ROS_INFO("Recieved velocity message from: [%d]", transfer->metadata.remote_node_id);

    float rads = velocity.radian_per_second;
    switch (transfer->metadata.remote_node_id) {
        case 2:
            wheel_speeds.fl = rads;
            break;
        case 4:
            wheel_speeds.fr = rads;
            break;
        case 8:
            wheel_speeds.br = rads;
            break;
        case 16:
            wheel_speeds.bl = rads;
            break;
    }
    transform.linear_speed = std::min({wheel_speeds.fl, wheel_speeds.fr, wheel_speeds.bl, wheel_speeds.br}) * WHEEL_RADIUS;

    uint64_t now = timeMillis();
    uint64_t dt = now - last_coord_calc_time_millis;
    double dl = transform.linear_speed / 1000000 * dt;

    Eigen::Vector3d displacement(dl, 0, 0);
    displacement = imu_q * displacement;

    transform.x == displacement(0);
    transform.y == displacement(1);
    transform.z == displacement(2);
}

void OdometryService::publish() {
    nav_msgs::Odometry odometry;
    odometry.pose.pose.position.x = transform.x;
    odometry.pose.pose.position.y = transform.y;
    odometry.pose.pose.position.z = transform.z;

    odometry.pose.pose.orientation.z = imu_q.z();
    odometry.pose.pose.orientation.x = imu_q.x();
    odometry.pose.pose.orientation.y = imu_q.y();
    odometry.pose.pose.orientation.w = imu_q.w();

    odometry.twist.twist.angular.x = transform.angular_vel_x;
    odometry.twist.twist.angular.y = transform.angular_vel_y;
    odometry.twist.twist.angular.z = transform.angular_vel_z;

    Eigen::Vector3d linear_speed(transform.linear_speed, 0, 0);
    linear_speed = imu_q * linear_speed;

    odometry.twist.twist.linear.x = linear_speed(0);
    odometry.twist.twist.linear.y = linear_speed(1);
    odometry.twist.twist.linear.z = linear_speed(2);

    odom_publisher.publish(odometry);
}


BatteryService::BatteryService(CyphalInterface* interface, ros::NodeHandle& node) : AbstractSubscription(
    interface,
    CanardTransferKindMessage,
    7993,
    voltbro_battery_state_1_0_EXTENT_BYTES_
) {
    bat_publisher = node.advertise<sensor_msgs::BatteryState>("bat", 5);
}

void BatteryService::handler(
    const voltbro_battery_state_1_0& bat_info,
    CanardRxTransfer* transfer
) {
    ROS_INFO("Recieved battery message from: [%d]", transfer->metadata.remote_node_id);

    sensor_msgs::BatteryState battery;
    battery.voltage = bat_info.voltage.volt;
    battery.current = bat_info.current.ampere;
    battery.charge = bat_info.charge.coulomb / 3.6;
    battery.capacity = bat_info.capacity.coulomb / 3.6;
    battery.percentage = battery.charge / battery.capacity;

    battery.design_capacity = bat_info.design_capacity.coulomb / 3.6;

    battery.power_supply_status = bat_info.power_supply_status.value;
    battery.power_supply_health = bat_info.power_supply_health.value;
    battery.power_supply_technology = bat_info.power_supply_technology.value;
    battery.present = bat_info.is_present.value == 1 ? true : false;

    battery.location = std::string(
        (char*)bat_info.location.value.elements,
        bat_info.location.value.count
    );
    battery.serial_number = std::string(
        (char*)bat_info.serial_number.value.elements,
        bat_info.serial_number.value.count
    );

    bat_publisher.publish(battery);
}

void HeartbeatService::handler(
    const uavcan_node_Heartbeat_1_0& hbeat,
    CanardRxTransfer* transfer
) {
    uint8_t node_id = transfer->metadata.remote_node_id;
    ROS_INFO("Recieved heartbeat message from: [%d]", node_id);

    if (last_hbeats.count(node_id) == 0) {
        return;
    }

    last_hbeats[node_id] = timeMillis();
}

bool HeartbeatService::is_anyone_down(uint64_t now) {
    bool found_missing = false;
    for (const std::pair<CanardNodeID, uint64_t>& pair : last_hbeats) {
        uint8_t node_id = pair.first;
        uint64_t last_time = pair.second;

        if (last_time == 0) continue;

        auto dt = now - last_time;

        if (dt > 3000) {
            ROS_INFO("Required node is down: [%d]", node_id);
            found_missing = true;
        }
    }

    return found_missing;
}
