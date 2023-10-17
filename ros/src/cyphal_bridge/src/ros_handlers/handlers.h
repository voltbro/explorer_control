#pragma once

#include <string>
#include <iostream>
#include <std_msgs/Float64.h>
#include <cyphal_bridge/HMIBeeper.h>
#include <cyphal_bridge/HMILed.h>
#include <cyphal_bridge/ResetDrive.h>
#include <cyphal_bridge/PowerReset.h>

#include <ros/ros.h>
#include <cyphal/subscriptions/subscription.h>
#include <libcanard/canard.h>

#include "../cyphal_subscriptions/subscriptions.h"
#include "../consts.h"
#include "../bases.h"

template <typename T>
class ROSHandler {
private:
    ros::Subscriber subscriber;
public:
    ROSHandler(ros::NodeHandle& node, const std::string& topic, size_t queue_size) {
        subscriber = node.subscribe(topic, queue_size, &ROSHandler::callback, this);
        std::cout << "Subscribed to topic <" << topic << ">" << std::endl;
    }
    ROSHandler(ros::NodeHandle& node, const std::string& topic)
        : ROSHandler(node, topic, 1000) {}
    virtual void callback(const typename T::ConstPtr&) = 0;
};

class WheelVelHandler : public ROSHandler<std_msgs::Float64> {
private:
    MotorService* service;
    const CanardNodeID target_id;
    static const uint32_t ZEROING_MILLIS = 350;
    const bool is_inverted;

public:
    bool is_zeroed;
    uint64_t last_update = 0;
    void callback(const std_msgs::Float64::ConstPtr& msg) override;
    void zeroing_check(uint64_t current_time);

    WheelVelHandler(ros::NodeHandle& node, const std::string& topic, CanardNodeID target_id, MotorService* service)
        : WheelVelHandler(node, topic, target_id, service, false){};
    WheelVelHandler(
        ros::NodeHandle& node,
        const std::string& topic,
        CanardNodeID target_id,
        MotorService* service,
        bool is_inverted
    ) : ROSHandler(node, topic),
        target_id(target_id),
        service(service),
        is_zeroed(false),
        is_inverted(is_inverted) {};
};

class HMIHandler : public ROSHandler<cyphal_bridge::HMIBeeper>, public ROSHandler<cyphal_bridge::HMILed>, public I_HMI {
private:
    CanardNodeID target_id;
    CyphalInterface* interface;
public:
    std::array<uint8_t, 3> last_color;

    HMIHandler(
        CyphalInterface* interface, ros::NodeHandle& node, const std::string& topic_beeper, const std::string& topic_led, CanardNodeID target_id
    ) : ROSHandler<cyphal_bridge::HMIBeeper>(node, topic_beeper),
        ROSHandler<cyphal_bridge::HMILed>(node, topic_led),
        target_id(target_id),
        interface(interface) {}
    void send_beep(float duration, float frequency) override;
    void send_color(uint8_t r, uint8_t g, uint8_t b) override;
    void reset_color();
    void callback(const cyphal_bridge::HMIBeeper::ConstPtr& msg) override;
    void callback(const cyphal_bridge::HMILed::ConstPtr& msg) override;
    ~HMIHandler();
};

template <typename T>
class ServiceProvider {
private:
    ros::ServiceServer service;
public:
    virtual bool callback(typename T::Request&, typename T::Response&) = 0;

    ServiceProvider(ros::NodeHandle& node, const std::string& service_name) {
        service = node.advertiseService(service_name, &ServiceProvider::callback, this);
        std::cout << "Providing service <" << service_name << ">" << std::endl;
    }
};

class ResetDriveProvider: ServiceProvider<cyphal_bridge::ResetDrive> {
private:
    MotorService* motor_service;
public:
    ResetDriveProvider(ros::NodeHandle& node, MotorService* motor_service):
        ServiceProvider(node, "/reset_drives"),
        motor_service(motor_service) {};
    bool callback(cyphal_bridge::ResetDrive::Request&, cyphal_bridge::ResetDrive::Response&) override;
};

class PowerServicesProvider: ServiceProvider<cyphal_bridge::PowerReset> {
private:
    CyphalInterface* interface;
public:
    PowerServicesProvider(CyphalInterface* interface, ros::NodeHandle& node):
        ServiceProvider(node, "/power/reset"),
        interface(interface) {}
    bool callback(cyphal_bridge::PowerReset::Request&, cyphal_bridge::PowerReset::Response&) override;
    void call_reboot();
};
