#include "Types.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <stdlib.h>
#include <string>

namespace StateEstimationUtils {

inline Eigen::Matrix<double, 2, 2> getRotationMatrix(double angle) {
    Eigen::Matrix<double, 2, 2> R;
    R << cos(angle), -sin(angle),
        sin(angle), cos(angle);
    return R;
}

inline void setCovariances(nav_msgs::msg::Odometry::SharedPtr &msg, covT P) {
    // position covariances
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++)
            msg->pose.covariance[j + 6 * i] = P(i, j);
    }
    for (int i = 0; i < 3; i++) {
        msg->pose.covariance[i + 6 * 2] = P(2, i);
        msg->pose.covariance[2 + 6 * i] = P(i, 2);
    }

    // twist covariances
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++)
            msg->twist.covariance[j + 6 * i] = P(i + 3, j + 3);
    }
    for (int i = 3; i < 6; i++) {
        msg->twist.covariance[i + 6 * 5] = P(5, i);
        msg->twist.covariance[5 + 6 * i] = P(i, 5);
    }
}

// debug utility
inline void printMatrix(msgT mat, std::string name) {
    std::cout << "DEBUG: printing matrix " << name << std::endl;
    std::cout << mat << std::endl;
}

} // namespace StateEstimationUtils
