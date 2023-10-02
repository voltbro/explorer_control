#pragma once

#include <array>
#include <map>

#include <ros/ros.h>

#include <cyphal/subscriptions/subscription.h>

#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <voltbro/battery/state_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>

#include "../bases.h"

#define CID_BR 16
#define CID_BL 8
#define CID_FR 4
#define CID_FL 2
#define CID_PWR 9

class MotorService: public AbstractSubscription<uavcan_register_Access_Response_1_0> {
private:
    DESERIALIZE_TYPE(uavcan_register_Access_Response_1_0, interface)

    std::array<CanardNodeID, 4> nodes = {2, 4, 8, 16};
    std::array<CanardNodeID, 2> left_nodes = {2, 8};
    std::array<CanardNodeID, 2> right_nodes = {4, 16};
public:
    MotorService(CyphalInterface* interface)
        : AbstractSubscription(
            interface,
            CanardTransferKindResponse,
            uavcan_register_Access_1_0_FIXED_PORT_ID_,
            uavcan_register_Access_Response_1_0_EXTENT_BYTES_
        ){};
    void handler(
        const uavcan_register_Access_Response_1_0& reg_response,
        CanardRxTransfer* transfer
    ) override;
    void send_state(CanardNodeID node_id, uint8_t state);
    void send_state_all(uint8_t state);
    void send_on();
    void send_off();
    void send_speed(CanardNodeID node_id, double speed);
    void send_speed_all(double speed);
    void send_speed_all_raw(double speed);
};

struct WheelSpeeds {
    float bl = 0;
    float br = 0;
    float fl = 0;
    float fr = 0;
};

struct Transform {
    float angular_vel_x = 0;
    float angular_vel_y = 0;
    float angular_vel_z = 0;

    double x = 0;
    double y = 0;
    double z = 0;

    double linear_speed = 0;
};

class OdometryService : public AbstractSubscription<uavcan_si_unit_angular_velocity_Scalar_1_0> {
private:
    DESERIALIZE_TYPE(uavcan_si_unit_angular_velocity_Scalar_1_0, interface)

    uint64_t last_coord_calc_time_millis = 0;
    // rotation (quaternion) and wheel speeds - basic data to infer pose
    // saved from cyphal (wheels) and ros (imu)
    Eigen::Quaterniond imu_q;
    WheelSpeeds wheel_speeds;
    // calculated from imu_q and wheel_speeds, used to create /odom
    Transform transform;
    // ros integration
    ros::Publisher odom_publisher;
    ros::Subscriber imu_subscriber;
public:
    OdometryService(CyphalInterface* interface, ros::NodeHandle&);
    void handler(
        const uavcan_si_unit_angular_velocity_Scalar_1_0& velocity,
        CanardRxTransfer* transfer
    ) override;
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void publish();
};

class BatteryService : public AbstractSubscription<voltbro_battery_state_1_0> {
private:
    DESERIALIZE_TYPE(voltbro_battery_state_1_0, interface)

    ros::Publisher bat_publisher;
public:
    BatteryService(CyphalInterface* interface, ros::NodeHandle&);
    void handler(
        const voltbro_battery_state_1_0& bat_info,
        CanardRxTransfer* transfer
    ) override;
};

class HeartbeatService : public AbstractSubscription<uavcan_node_Heartbeat_1_0> {
private:
    DESERIALIZE_TYPE(uavcan_node_Heartbeat_1_0, interface)

    const I_HMI* hmi;
    std::map<CanardNodeID, uint64_t> last_hbeats = {};
public:
    HeartbeatService(CyphalInterface* interface, I_HMI* hmi):
        AbstractSubscription(
            interface,
            CanardTransferKindMessage,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            uavcan_node_Heartbeat_1_0_EXTENT_BYTES_
        ),
        hmi(hmi) {
        last_hbeats[CID_PWR] = 0;
        last_hbeats[CID_BL] = 0;
        last_hbeats[CID_BR] = 0;
        last_hbeats[CID_FL] = 0;
        last_hbeats[CID_FR] = 0;
    };
    void handler(
        const uavcan_node_Heartbeat_1_0& hbeat,
        CanardRxTransfer* transfer
    ) override;
    bool is_anyone_down(uint64_t now);
};
