#include "EstimationNode.h"
#include "ExtendedKalmanFilter.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"
#include <algorithm>
#include <chrono>
#include <math.h>
#include <time.h>

using namespace std::chrono_literals;
using namespace std::chrono;
constexpr double GRAVITY = 9.81;

EstimationNode::EstimationNode() : Node("EstimationNode") {
    // parameter initialization
    this->initializeParams();

    // publisher initialization
    this->initializePublishers();

    // subscriber initialization
    this->initializeSubscribers();

    // parameter reading
    std::string package_share_path_control = ament_index_cpp::get_package_share_directory("turtle_control");
    std::string package_share_path_common = ament_index_cpp::get_package_share_directory("turtle_common");
    std::string dv_yaml = package_share_path_common + "/config/car_models/ev22_params.yaml";
    std::string dv_ecu_yaml = package_share_path_control + "/config/missions_params/" + params.mission + ".yaml";
    this->vParams = paramsFromFile(dv_yaml);
    this->eParams = paramsFromYaml(dv_ecu_yaml);

    rclcpp::Time startingTime = rclcpp::Time(0, 0);

    this->lastUpdateTime = startingTime;
    this->firstMessage = true;

    this->hallLastUpdate = startingTime;
    this->hallLastValue << 0, 0;
    this->firstHallMessage = true;

    this->resolverLastUpdate = startingTime;
    this->resolverLastValue << 0, 0;
    this->firstResolverMessage = true;

    this->imuLastUpdate = startingTime;
    this->imuLastValue << 0;
    this->firstImuMessage = true;

    this->vel1LastUpdate = startingTime;
    this->vel1LastValue << 0, 0;
    this->firstGPS1Message = true;

    this->vel2LastUpdate = startingTime;
    this->vel2LastValue << 0, 0;
    this->firstGPS2Message = true;

    // initialize estimator
    this->estimator = ExtendedKalmanFilter(paramsFromFile(dv_yaml), paramsFromYaml(dv_ecu_yaml), this->params.processModel, this->params.lowSpeed, this->params.highSpeed);

    // we assume to start from a zeroed state
    this->state << 0, 0, 0, 0, 0, 0;
    this->input << 0, 0;

    // initially no knowledge of heading
    this->headingInitialized = false;
    this->heading = 0;

    // initial estimation covariance
    this->P << 0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0.25, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    // resolver covariance matrix
    // this->Rresolver << 1000.0, 0,
    //     0, 1000.0;
    this->Rresolver << 100.0, 0,
        0, 100.0;

    // hall covariance matrix
    this->Rhall << 52308, 0,
        0, 780480;

    // IMU covariance matrix
    this->Rimu << 1e-5;

    // GPS velocity covariance matrices
    this->Rvel1 << 1, 0,
        0, 1;
    this->Rvel2 << 1, 0,
        0, 1;

    // if we want to use asynchronous prediction
    if (this->params.asyncPredict) {
        this->predictTimer = this->create_wall_timer(10ms, std::bind(&EstimationNode::asyncPredictFunction, this));
    }
}

void EstimationNode::initializeParams() {
    this->declare_parameter("hall", false);
    this->params.hall = this->get_parameter("hall").as_bool();

    this->declare_parameter("resolver", false);
    this->params.resolver = this->get_parameter("resolver").as_bool();

    this->declare_parameter("imu", false);
    this->params.imu = this->get_parameter("imu").as_bool();

    this->declare_parameter("gps1", false);
    this->params.gps1 = this->get_parameter("gps1").as_bool();

    this->declare_parameter("gps2", false);
    this->params.gps2 = this->get_parameter("gps2").as_bool();

    this->declare_parameter("insAfterInit", false);
    this->params.insAfterInit = this->get_parameter("insAfterInit").as_bool();
    this->insInitialized = false;

    this->declare_parameter("processModel", "bicycle");
    this->params.processModel = this->get_parameter("processModel").as_string();

    this->declare_parameter("lowSpeed", 3.0);
    this->params.lowSpeed = this->get_parameter("lowSpeed").as_double();

    this->declare_parameter("highSpeed", 5.0);
    this->params.highSpeed = this->get_parameter("highSpeed").as_double();

    this->declare_parameter("mission", "acceleration");
    this->params.mission = this->get_parameter("mission").as_string();

    this->declare_parameter("lockOnPublish", false);
    this->params.lockOnPublish = this->get_parameter("lockOnPublish").as_bool();

    this->declare_parameter("steeringOffset", 0.51);
    this->params.steeringOffset = this->get_parameter("steeringOffset").as_double();

    this->declare_parameter("asyncPredict", false);
    this->params.asyncPredict = this->get_parameter("asyncPredict").as_bool();

    rclcpp::Parameter time_param("use_sim_time", params.use_sim_time ? true : false);
    this->set_parameter(time_param);
}

void EstimationNode::initializePublishers() {
    rclcpp::SensorDataQoS sensorQos;

    this->estimationPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", sensorQos);
    this->tfBroadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));
    // constant publish frequency of 100 Hz
    this->publishTimer = this->create_wall_timer(10ms, std::bind(&EstimationNode::estimationPublisherFunction, this));
}

void EstimationNode::initializeSubscribers() {
    rclcpp::QoS q(1);
    q.best_effort();
    rclcpp::SensorDataQoS sensorQos;
    this->throttleSubscriber = this->create_subscription<turtle_interfaces::msg::ActuatorCmd>("/cmd", q, std::bind(&EstimationNode::throttleCallback, this, std::placeholders::_1));
    this->steeringSubscriber = this->create_subscription<turtle_interfaces::msg::Steering>("/steering_actual", q, std::bind(&EstimationNode::steeringCallback, this, std::placeholders::_1));
    this->resolverSubscriber = this->create_subscription<turtle_interfaces::msg::RPM>("/rpm_motor", q, std::bind(&EstimationNode::resolverCallback, this, std::placeholders::_1));
    this->imuSubscriber = this->create_subscription<turtle_interfaces::msg::VnImu>("/vn_imu", q, std::bind(&EstimationNode::imuCallback, this, std::placeholders::_1));
    this->gpsVel1Subscriber = this->create_subscription<turtle_interfaces::msg::VnGpsVelocity>("/vn_gps_velocity_1", q, std::bind(&EstimationNode::gpsVelCallback1, this, std::placeholders::_1));
    this->gpsVel2Subscriber = this->create_subscription<turtle_interfaces::msg::VnGpsVelocity>("/vn_gps_velocity_2", q, std::bind(&EstimationNode::gpsVelCallback2, this, std::placeholders::_1));
    this->eulerAnglesSubscriber = this->create_subscription<turtle_interfaces::msg::VnEuler>("/vn_euler", q, std::bind(&EstimationNode::eulerAnglesCallback, this, std::placeholders::_1));
    this->insStatusSubscriber = this->create_subscription<turtle_interfaces::msg::VnInsStatus>("/vn_ins_status", q, std::bind(&EstimationNode::insStatusCallback, this, std::placeholders::_1));
    this->insOdomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>("/vn_odom", q, std::bind(&EstimationNode::insOdomCallback, this, std::placeholders::_1));
}

void EstimationNode::changeStateAndCov(estimationT estimation, rclcpp::Time updateTime) {
    // change the values
    this->state = estimation.first;
    this->P = estimation.second;
    this->lastUpdateTime = updateTime;
}

// subscriber callbacks and checkers
// drift error detection is not possible since there is no redundancy for our measurements
void EstimationNode::throttleCallback(turtle_interfaces::msg::ActuatorCmd::SharedPtr msg) {
    this->input[0] = std::min(std::max(msg->throttle, -1.0f), 1.0f);
    this->input[1] = std::min(std::max((float) (msg->steering / this->vParams.SR), -1.0f), 1.0f);
}

void EstimationNode::steeringCallback(turtle_interfaces::msg::Steering::SharedPtr msg) {
    // this->input[1] = std::min(std::max(msg->steering, -1.0f), 1.0f);
    // this->input[1] = msg->steering - this->params.steeringOffset;
    // if (this->input[1] < -M_PI) {
    //     this->input[1] += M_PI;
    // }
}

void EstimationNode::insStatusCallback(turtle_interfaces::msg::VnInsStatus::SharedPtr msg) {
    // need to check the first two bits
    uint16_t firstBits = msg->status & 3;
    if ((firstBits == 2) && !this->insInitialized) {
        this->insInitialized = true;
    }
    if ((firstBits == 0 || firstBits == 1 || firstBits == 3) && this->insInitialized) {
        this->insInitialized = false;
    }
}

void EstimationNode::insOdomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!(this->params.insAfterInit && this->insInitialized)) {
        return;
    }

    // meed to change state and P
    // get message time
    rclcpp::Time msgTime(msg->header.stamp.sec, msg->header.stamp.nanosec);

    // formulate the objects
    // set up variables
    doubleT insPosition;
    stateT insState;
    double x = state[0];
    double y = state[1];
    double yaw = state[2];
    double ux = state[3];
    double uy = state[4];
    double r = state[5];

    // performance optimization
    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);

    // arithmetic integration
    double xDot = (ux * cosyaw - uy * sinyaw);
    double yDot = (ux * sinyaw + uy * cosyaw);
    double yawDot = r;

    // calculate delta time
    double dt = (msgTime - this->lastUpdateTime).seconds();

    x = x + xDot * dt;
    y = y + yDot * dt;
    yaw = yaw + yawDot * dt;
    insState << x, y, yaw, msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z;
    covT insCov;
    double posCov = msg->pose.covariance[0];
    double yawCov = msg->pose.covariance[35];
    insCov << posCov, 0, 0, 0, 0, 0,
        0, posCov, 0, 0, 0, 0,
        0, 0, yawCov, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01;

    estimationT insStatePair = std::make_pair(insState, insCov);

    // call the function to change them
    this->g_calc_mutex.lock();

    this->changeStateAndCov(insStatePair, msgTime);

    this->g_calc_mutex.unlock();
}

void EstimationNode::eulerAnglesCallback(turtle_interfaces::msg::VnEuler::SharedPtr msg) {
    // // determine if a heading is available
    // sike, heading always available
    this->headingInitialized = true;
    this->heading = msg->angle.z;
}

void EstimationNode::resolverCallback(turtle_interfaces::msg::RPM::SharedPtr msg) {
    if (!this->params.resolver) {
        return;
    }

    if (this->params.insAfterInit && this->insInitialized) {
        return;
    }

    // messageTime
    rclcpp::Time msgTime(msg->header.stamp.sec, msg->header.stamp.nanosec);

    // check if it is the first message, and we have no delta time available
    if (this->firstMessage) {
        // perform an update step normaly
        doubleT z;
        z << msg->left, msg->right;

        // observation matrix
        doubleObsMatrix H;

        H << 0, 0, 0, 1, 0, -(this->vParams.t / 2),
            0, 0, 0, 1, 0, (this->vParams.t / 2);
        H = H * 60.0 * 3.2 / (2 * M_PI * this->vParams.r_eff); // m/s in wheels, to inverter's rpm

        // this is done to prevent multiple predict steps to take place before an update step
        this->g_calc_mutex.lock();

        if(msg->left < 0 || msg->right < 0) {
            z << 0.0, 0.0;
        }

        // update
        estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rresolver);
        this->changeStateAndCov(updated, msgTime);

        this->g_calc_mutex.unlock();

        // ensure this will code will not run from now on
        this->firstMessage = false;
        return;
    }

    // dt for prediction
    double dt = (msgTime - this->lastUpdateTime).seconds();
    // if laggy sensor ignore it
    if (dt < 0) {
        return;
    }

    // loading observation
    doubleT z;
    z << msg->left, msg->right;
    if (!this->firstResolverMessage && !checkResolver(z, msgTime)) {
        return;
    }

    // if it is the first message
    if (this->firstResolverMessage) {
        // the next one won't be
        this->firstResolverMessage = false;
    }

    // observation matrix
    doubleObsMatrix H;

    H << 0, 0, 0, 1, 0, -(this->vParams.t / 2),
        0, 0, 0, 1, 0, (this->vParams.t / 2);
    H = H * 60.0 * 3.2 / (2 * M_PI * this->vParams.r_eff); // m/s in wheels, to inverter's rpm

    // this is done to prevent multiple predict steps to take place before an update step
    this->g_calc_mutex.lock();

    // predict
    if (!this->params.asyncPredict) {
        estimationT predicted = this->estimator.predict(this->state, this->input, this->P, dt);
        this->changeStateAndCov(predicted, msgTime);
    }

    if(msg->left < 0 || msg->right < 0) {
        z << 0.0, 0.0;
    }
    // update
    estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rresolver);
    this->changeStateAndCov(updated, msgTime);

    this->g_calc_mutex.unlock();
}

bool EstimationNode::checkResolver(doubleT &values, rclcpp::Time time) {
    // maybe implement new range checks?
    int positionLimit = 1000;
    bool positionCheck = false;

    int velocityLimit = 1440 * 3.2;
    bool velocityCheck = false;

    int differenceLimit = 20;
    bool differenceCheck = false;

    double dt = (time - this->resolverLastUpdate).seconds();

    // position check
    if ((values[0] > positionLimit || values[1] > positionLimit) && positionCheck) {
        return false;
    }

    // velocity check
    if ((abs((values[0] - this->resolverLastValue[0])) / dt > velocityLimit || abs((values[1] - this->resolverLastValue[1])) / dt > velocityLimit) && velocityCheck) {
        return false;
    }

    // difference check
    if ((abs(values[0] - values[1]) > differenceLimit) && differenceCheck) {
        return false;
    }

    this->resolverLastValue = values;
    this->resolverLastUpdate = time;

    return true;
}

void EstimationNode::hallCallback(turtle_interfaces::msg::RPM::SharedPtr msg) {
    if (!this->params.hall) {
        return;
    }

    if (this->params.insAfterInit && this->insInitialized) {
        return;
    }

    // messageTime
    rclcpp::Time msgTime(msg->header.stamp.sec, msg->header.stamp.nanosec);

    // steering angle
    double delta = this->vParams.SR * this->input[1];
    double cosdelta = cos(delta);
    double sindelta = sin(delta);

    // check if it is the first message, and we have no delta time available
    if (this->firstMessage) {
        // loading observation
        doubleT z;
        z << msg->left, msg->right;

        // observation matrix depends on steering angle delta
        doubleObsMatrix H;
        double lf = this->vParams.L - this->vParams.l_r;

        H << 0, 0, 0, cosdelta, sindelta, -(this->vParams.t / 2.0) * cosdelta + lf * sindelta,
            0, 0, 0, cosdelta, sindelta, (this->vParams.t / 2.0) * cosdelta + lf * sindelta;
        H = H * 60.0 / (2 * M_PI * this->vParams.r_eff); // m/s to rpm // for some reason gets to negative inf here(?)

        // this is done to prevent multiple predict steps to take place before an update step
        this->g_calc_mutex.lock();

        // update
        estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rhall);
        this->changeStateAndCov(updated, msgTime);

        this->g_calc_mutex.unlock();

        // ensure this will code will not run from now on
        this->firstMessage = false;
        return;
    }

    // dt for prediction
    double dt = (msgTime - this->lastUpdateTime).seconds();
    // if laggy sensor ignore it
    if (dt < 0) {
        return;
    }

    // loading observation
    doubleT z;
    z << msg->left, msg->right;

    // applying filters
    if (!this->firstHallMessage && !this->checkHall(z, msgTime)) {
        return;
    }

    // if it is the first message
    if (this->firstHallMessage) {
        // the next one won't be
        this->firstHallMessage = false;
    }

    // observation matrix depends on steering angle delta
    doubleObsMatrix H;
    float lf = this->vParams.L - this->vParams.l_r;

    H << 0, 0, 0, cosdelta, sindelta, -(this->vParams.t / 2.0) * cosdelta + lf * sindelta,
        0, 0, 0, cosdelta, sindelta, (this->vParams.t / 2.0) * cosdelta + lf * sindelta;
    H = H * 60.0 / (2 * M_PI * this->vParams.r_eff); // m/s to rpm

    // this is done to prevent multiple predict steps to take place before an update step
    this->g_calc_mutex.lock();

    // predict
    estimationT predicted = this->estimator.predict(this->state, this->input, this->P, dt);
    this->changeStateAndCov(predicted, msgTime);

    // update
    estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rhall);
    this->changeStateAndCov(updated, msgTime);

    this->g_calc_mutex.unlock();
}

bool EstimationNode::checkHall(doubleT &values, rclcpp::Time time) {
    // TODO: implement new filter checks
    int positionLimit = 900;
    bool positionCheck = true;

    int velocityLimit = 1440;
    bool velocityCheck = true;

    int differenceLimit = 200;
    bool differenceCheck = true;

    double dt = (time - this->resolverLastUpdate).seconds();

    // position check
    if ((values[0] > positionLimit || values[1] > positionLimit) && positionCheck) {
        return false;
    }

    // velocity check
    if ((abs((values[0] - this->resolverLastValue[0])) / dt > velocityLimit || abs((values[1] - this->resolverLastValue[1])) / dt > velocityLimit) && velocityCheck) {
        return false;
    }

    // difference check
    if ((abs(values[0] - values[1]) > differenceLimit) && differenceCheck) {
        return false;
    }

    this->hallLastValue = values;
    this->hallLastUpdate = time;

    return true;
}

void EstimationNode::imuCallback(turtle_interfaces::msg::VnImu::SharedPtr msg) {
    if (!this->params.imu) {
        return;
    }

    if (this->params.insAfterInit && this->insInitialized) {
        return;
    }

    // messageTime
    rclcpp::Time msgTime(msg->header.stamp.sec, msg->header.stamp.nanosec);

    // check if it is the first message, and we have no delta time available
    if (this->firstMessage) {
        // loading observation
        singleT z;

        // IMU frame to odom frame
        z << msg->gyro.z;

        // observation matrix
        singleObsMatrix H;
        H << 0, 0, 0, 0, 0, 1;

        // this is done to prevent multiple predict steps to take place before an update step
        this->g_calc_mutex.lock();

        // update
        estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rimu);
        this->changeStateAndCov(updated, msgTime);

        this->g_calc_mutex.unlock();

        // ensure this will code will not run from now on
        this->firstMessage = false;
        return;
    }

    // dt for prediction
    double dt = (msgTime - this->lastUpdateTime).seconds();
    // if laggy sensor ignore it
    if (dt < 0) {
        return;
    }

    // loading observation
    singleT z;

    // IMU frame to odom frame
    z << msg->gyro.z;

    // applying filters
    if (!this->firstImuMessage && !this->checkImu(z, msgTime)) {
        return;
    }

    // if it is the first message
    if (this->firstImuMessage) {
        // the next one won't be
        this->firstImuMessage = false;
    }

    // observation matrix
    singleObsMatrix H;
    H << 0, 0, 0, 0, 0, 1;

    // this is done to prevent multiple predict steps to take place before an update step
    this->g_calc_mutex.lock();

    // predict
    if (!this->params.asyncPredict) {
        estimationT predicted = this->estimator.predict(this->state, this->input, this->P, dt);
        this->changeStateAndCov(predicted, msgTime);
    }

    // update
    estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rimu);
    this->changeStateAndCov(updated, msgTime);

    this->g_calc_mutex.unlock();
}

bool EstimationNode::checkImu(singleT &values, rclcpp::Time time) {
    // possible check for outliers here
    // imu is pretty reliable, no need to discard any values

    this->imuLastValue = values;
    this->imuLastUpdate = time;

    return true;
}

void EstimationNode::gpsVelCallback1(turtle_interfaces::msg::VnGpsVelocity::SharedPtr msg) {
    if (!this->params.gps1) {
        return;
    }

    if (this->params.insAfterInit && this->insInitialized) {
        return;
    }

    // if no heading exists
    if (!this->headingInitialized) {
        return;
    }

    // messageTime
    rclcpp::Time msgTime(msg->header.stamp.sec, msg->header.stamp.nanosec);

    // check if it is the first message, and we have no delta time available
    if (this->firstMessage) {
        // loading observation
        doubleT z;
        z << msg->vel.x, -msg->vel.y; // NE frame to NW frame that coincides with our vehicle frame orientation (y axis on the left)
        double uncertainty = msg->uncertainty;
        // rotate the velocities from the NW frame to the odom frame
        // determine true angle of heading versus north
        double angle = this->heading * M_PI / 180.0; // angle is in degrees, convert to radians
        Eigen::Matrix<double, 2, 2> R = StateEstimationUtils::getRotationMatrix(angle);
        z = R * z;

        // save values
        this->vel1LastValue = z;
        this->vel1LastUpdate = msgTime;

        // ensure the filters run properly
        this->firstGPS1Message = false;

        // observation matrix
        doubleObsMatrix H;
        H << 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0;

        // this is done to prevent multiple predict steps to take place before an update step
        this->g_calc_mutex.lock();

        // update
        estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rvel1 * uncertainty * uncertainty);
        this->changeStateAndCov(updated, msgTime);

        this->g_calc_mutex.unlock();

        // ensure this will code will not run from now on
        this->firstMessage = false;
        return;
    }

    // dt for prediction
    double dt = (msgTime - this->lastUpdateTime).seconds();
    // if laggy sensor ignore it
    if (dt < 0) {
        return;
    }

    // loading observation
    doubleT z;
    z << msg->vel.x, -msg->vel.y; // NE frame to NW frame that coincides with our vehicle frame orientation (y axis on the left)
    double uncertainty = msg->uncertainty;
    // rotate the velocities from the NW frame to the odom frame
    // determine true angle of heading versus north
    double angle = this->heading * M_PI / 180.0; // angle is in degrees, convert to radians
    Eigen::Matrix<double, 2, 2> R = StateEstimationUtils::getRotationMatrix(angle);
    z = R * z;

    // applying filters
    if (!this->firstGPS1Message) {
        // currently no filters here
    }

    // if it is the first message
    if (this->firstGPS1Message) {
        // the next one won't be
        this->firstGPS1Message = false;
    }

    // save current values
    this->vel1LastValue = z;
    this->vel1LastUpdate = msgTime;

    // observation matrix
    doubleObsMatrix H;
    H << 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0;

    // this is done to prevent multiple predict steps to take place before an update step
    this->g_calc_mutex.lock();

    // predict
    if (!this->params.asyncPredict) {
        estimationT predicted = this->estimator.predict(this->state, this->input, this->P, dt);
        this->changeStateAndCov(predicted, msgTime);
    }

    // update
    estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rvel1 * uncertainty * uncertainty);
    this->changeStateAndCov(updated, msgTime);

    this->g_calc_mutex.unlock();
}

void EstimationNode::gpsVelCallback2(turtle_interfaces::msg::VnGpsVelocity::SharedPtr msg) {
    if (!this->params.gps2) {
        return;
    }

    if (this->params.insAfterInit && this->insInitialized) {
        return;
    }

    // if no heading exists
    if (!this->headingInitialized) {
        return;
    }

    // messageTime
    rclcpp::Time msgTime(msg->header.stamp.sec, msg->header.stamp.nanosec);

    // check if it is the first message, and we have no delta time available
    if (this->firstMessage) {
        // loading observation
        doubleT z;
        z << msg->vel.x, -msg->vel.y; // NE frame to NW frame that coincides with our vehicle frame orientation (y axis on the left)
        double uncertainty = msg->uncertainty;
        // rotate the velocities from the NW frame to the odom frame
        // determine true angle of heading versus north
        double angle = this->heading * M_PI / 180.0; // angle is in degrees, convert to radians
        Eigen::Matrix<double, 2, 2> R = StateEstimationUtils::getRotationMatrix(angle);
        z = R * z;

        // save values
        this->vel2LastValue = z;
        this->vel2LastUpdate = msgTime;

        // ensure the filters run properly
        this->firstGPS2Message = false;

        // observation matrix
        doubleObsMatrix H;
        H << 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0;

        // this is done to prevent multiple predict steps to take place before an update step
        this->g_calc_mutex.lock();

        // update
        estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rvel2 * uncertainty * uncertainty);
        this->changeStateAndCov(updated, msgTime);

        this->g_calc_mutex.unlock();

        // ensure this will code will not run from now on
        this->firstMessage = false;
        return;
    }

    // dt for prediction
    double dt = (msgTime - this->lastUpdateTime).seconds();
    // if laggy sensor ignore it
    if (dt < 0) {
        return;
    }

    // loading observation
    doubleT z;
    z << msg->vel.x, -msg->vel.y; // NE frame to NW frame that coincides with our vehicle frame orientation (y axis on the left)
    double uncertainty = msg->uncertainty;
    // rotate the velocities from the NW frame to the odom frame
    // determine true angle of heading versus north
    double angle = this->heading * M_PI / 180.0; // angle is in degrees, convert to radians
    Eigen::Matrix<double, 2, 2> R = StateEstimationUtils::getRotationMatrix(angle);
    z = R * z;

    // applying filters
    if (!this->firstGPS2Message) {
        // currently no filters here
    }

    // if it is the first message
    if (this->firstGPS2Message) {
        // the next one won't be
        this->firstGPS2Message = false;
    }

    // save current values
    this->vel2LastValue = z;
    this->vel2LastUpdate = msgTime;

    // observation matrix
    doubleObsMatrix H;
    H << 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0;

    // this is done to prevent multiple predict steps to take place before an update step
    this->g_calc_mutex.lock();

    // predict
    if (!this->params.asyncPredict) {
        estimationT predicted = this->estimator.predict(this->state, this->input, this->P, dt);
        this->changeStateAndCov(predicted, msgTime);
    }

    // update
    estimationT updated = this->estimator.update(this->state, this->P, H * this->state, z, H, this->Rvel2 * uncertainty * uncertainty);
    this->changeStateAndCov(updated, msgTime);

    this->g_calc_mutex.unlock();
}

// publisher functions
// publish the current state every time it is called
void EstimationNode::estimationPublisherFunction() {
    // setting odom msg
    nav_msgs::msg::Odometry::SharedPtr msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.frame_id = "odom";
    msg->child_frame_id = "base_link";
    // pose position and orientation with covariance
    rclcpp::Time headerTime;

    if (this->params.lockOnPublish) {
        this->g_calc_mutex.lock();
    }

    headerTime = this->lastUpdateTime;
    double x = this->state[0];
    double y = this->state[1];
    double yaw = this->state[2];
    double ux = this->state[3];
    double uy = this->state[4];
    double r = this->state[5];
    // set message covariances
    StateEstimationUtils::setCovariances(msg, P);

    if (this->params.lockOnPublish) {
        this->g_calc_mutex.unlock();
    }

    msg->header.stamp = headerTime;
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;

    msg->twist.twist.linear.x = ux;
    msg->twist.twist.linear.y = uy;
    msg->twist.twist.angular.z = r;

    // pose rotation
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    msg->pose.pose.orientation.x = q.x();
    msg->pose.pose.orientation.y = q.y();
    msg->pose.pose.orientation.z = q.z();
    msg->pose.pose.orientation.w = q.w();

    // setting transform msg
    geometry_msgs::msg::TransformStamped odomTransform;
    odomTransform.header.stamp = headerTime;
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";

    odomTransform.transform.translation.x = x;
    odomTransform.transform.translation.y = y;
    odomTransform.transform.translation.z = 0;
    odomTransform.transform.rotation.x = q.x();
    odomTransform.transform.rotation.y = q.y();
    odomTransform.transform.rotation.z = q.z();
    odomTransform.transform.rotation.w = q.w();

    // publishing odom
    this->estimationPublisher->publish(*msg);
    // Send transform
    this->tfBroadcaster->sendTransform(odomTransform);
}

void EstimationNode::asyncPredictFunction() {
    this->g_calc_mutex.lock();
    // fixed delta time of 10ms ==  0.01 sec
    estimationT predicted = this->estimator.predict(this->state, this->input, this->P, 0.01);
    // we use the duration (seconds, nanoseconds) constructor
    rclcpp::Time newTime = this->lastUpdateTime + rclcpp::Duration(0, 10000);
    this->changeStateAndCov(predicted, newTime);
    this->g_calc_mutex.unlock();
}