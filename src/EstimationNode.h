#ifndef ESTIMATION_NODE_H
#define ESTIMATION_NODE_H

#include "ExtendedKalmanFilter.h"
#include "Types.h"
#include "rclcpp/rclcpp.hpp"
#include "turtle_common/ECUParams.hpp"
#include "turtle_common/Input.h"
#include "turtle_common/VehicleParams.h"
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <turtle_interfaces/msg/actuator_cmd.hpp>
#include <turtle_interfaces/msg/rpm.hpp>
#include <turtle_interfaces/msg/steering.hpp>
#include <turtle_interfaces/msg/vn_euler.hpp>
#include <turtle_interfaces/msg/vn_gps_velocity.hpp>
#include <turtle_interfaces/msg/vn_imu.hpp>
#include <turtle_interfaces/msg/vn_ins_status.hpp>

struct EstimationParams {
    bool use_sim_time;
    bool hall;
    bool resolver;
    bool imu;
    bool gps1;
    bool gps2;
    bool insAfterInit;
    double lowSpeed;
    double highSpeed;
    bool lockOnPublish;
    double steeringOffset;
    bool asyncPredict;
    std::string processModel;
    std::string mission;
};

class EstimationNode : public rclcpp::Node {
private:
    // state, input and other variables
    stateT state;
    covT P;
    inputT input;

    // Estimator
    ExtendedKalmanFilter estimator;

    // parameters
    VehicleParams vParams;
    ECUParams eParams;
    EstimationParams params;

    // observations
    doubleCovT Rresolver;
    doubleCovT Rhall;
    singleCovT Rimu;
    doubleCovT Rvel1;
    doubleCovT Rvel2;

    // chrono
    rclcpp::Time lastUpdateTime;
    bool firstMessage;

    // mutex
    std::mutex g_calc_mutex;
    void changeStateAndCov(estimationT estimation, rclcpp::Time updateTime);

    // is there a heading, and if so, the heading
    bool headingInitialized;
    double heading;      // in degrees
    bool insInitialized; // is the INS initialized in a good mode

    // subscribers
    rclcpp::Subscription<turtle_interfaces::msg::RPM>::SharedPtr resolverSubscriber;
    rclcpp::Subscription<turtle_interfaces::msg::VnImu>::SharedPtr imuSubscriber;
    rclcpp::Subscription<turtle_interfaces::msg::ActuatorCmd>::SharedPtr throttleSubscriber;
    rclcpp::Subscription<turtle_interfaces::msg::Steering>::SharedPtr steeringSubscriber;
    rclcpp::Subscription<turtle_interfaces::msg::RPM>::SharedPtr hallSubscriber;
    rclcpp::Subscription<turtle_interfaces::msg::VnGpsVelocity>::SharedPtr gpsVel1Subscriber;
    rclcpp::Subscription<turtle_interfaces::msg::VnGpsVelocity>::SharedPtr gpsVel2Subscriber;
    rclcpp::Subscription<turtle_interfaces::msg::VnEuler>::SharedPtr eulerAnglesSubscriber;
    rclcpp::Subscription<turtle_interfaces::msg::VnInsStatus>::SharedPtr insStatusSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr insOdomSubscriber;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimationPublisher;
    rclcpp::TimerBase::SharedPtr publishTimer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::TimerBase::SharedPtr predictTimer;

    // tracker variables
    rclcpp::Time hallLastUpdate;
    doubleT hallLastValue;
    bool firstHallMessage;

    rclcpp::Time resolverLastUpdate;
    doubleT resolverLastValue;
    bool firstResolverMessage;

    rclcpp::Time imuLastUpdate;
    singleT imuLastValue;
    bool firstImuMessage;

    rclcpp::Time vel1LastUpdate;
    doubleT vel1LastValue;
    bool firstGPS1Message;

    rclcpp::Time vel2LastUpdate;
    doubleT vel2LastValue;
    bool firstGPS2Message;

    void initializeParams();

    // ROS2 subscribers
    void initializeSubscribers();

    void throttleCallback(turtle_interfaces::msg::ActuatorCmd::SharedPtr msg);
    void steeringCallback(turtle_interfaces::msg::Steering::SharedPtr msg);
    void eulerAnglesCallback(turtle_interfaces::msg::VnEuler::SharedPtr msg);
    void insStatusCallback(turtle_interfaces::msg::VnInsStatus::SharedPtr msg);

    void resolverCallback(turtle_interfaces::msg::RPM::SharedPtr msg);
    bool checkResolver(doubleT &values, rclcpp::Time time);

    void imuCallback(turtle_interfaces::msg::VnImu::SharedPtr msg);
    bool checkImu(singleT &values, rclcpp::Time time);

    void hallCallback(turtle_interfaces::msg::RPM::SharedPtr msg);
    bool checkHall(doubleT &values, rclcpp::Time time);

    void gpsVelCallback1(turtle_interfaces::msg::VnGpsVelocity::SharedPtr msg);
    void gpsVelCallback2(turtle_interfaces::msg::VnGpsVelocity::SharedPtr msg);

    void insOdomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    // ROS2 publishers
    void initializePublishers();

    void estimationPublisherFunction();

    void asyncPredictFunction();

public:
    // constructor
    EstimationNode();
};

#endif
