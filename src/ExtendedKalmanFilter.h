#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "Types.h"
#include "turtle_common/ECUParams.hpp"
#include "turtle_common/VehicleParams.h"
#include <eigen3/Eigen/Dense>
#include <utility>

class ExtendedKalmanFilter {
private:
    // params
    VehicleParams vParams;
    ECUParams eParams;
    std::string model;
    double lowSpeed;
    double highSpeed;

    // model cov
    covT Q;

    // process model specifics
    stateT processModelBicycle(stateT state, inputT input, double dt);
    stateT processModelDynamic(stateT state, inputT input, double dt);
    stateT processModelBlended(stateT state, inputT input, double dt);
    covT linearizeProcessModel(stateT state, inputT input, double dt);

    // misc
    double deltaPrev;

public:
    // constructor
    ExtendedKalmanFilter(VehicleParams vParams, ECUParams eParams, std::string model, double lowSpeed, double highSpeed);
    ExtendedKalmanFilter();

    // filter equations
    estimationT predict(stateT state, inputT input, covT P, double dt);
    estimationT update(
        stateT state, // state
        covT P,       // covariance of state
        msgT Hk,      // observation
        msgT zk,      // measurement
        msgT H,       // observation matrix
        msgT R);      // covariance matrix
};

#endif