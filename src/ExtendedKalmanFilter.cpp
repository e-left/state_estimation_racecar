#include "ExtendedKalmanFilter.h"
#include "Types.h"
#include "processModel.hpp"
#include "utils.hpp"
#include <string>
#include <utility>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    this->vParams = VehicleParams();
    this->eParams = ECUParams();
    this->deltaPrev = 0;

    this->Q << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(VehicleParams vParams, ECUParams eParams, std::string model, double lowSpeed, double highSpeed) {
    this->vParams = vParams;
    this->eParams = eParams;
    this->model = model;
    this->lowSpeed = lowSpeed;
    this->highSpeed = highSpeed;

    // model covariance
    this->Q << 0.5, 0, 0, 0, 0, 0,
        0, 0.5, 0, 0, 0, 0,
        0, 0, 0.5, 0, 0, 0,
        0, 0, 0, 0.5, 0, 0,
        0, 0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0, 0.5;
    this->Q *= 1000.0;
}

estimationT ExtendedKalmanFilter::predict(stateT state, inputT input, covT P, double dt) {
    // state prediction
    stateT stateResult;
    if (this->model == "dynamic") {
        stateResult = this->processModelDynamic(state, input, dt);
    } else if (this->model == "blended") {
        stateResult = processModelBlended(state, input, dt);
    } else { // fall back to bicyclemodel
        stateResult = processModelBicycle(state, input, dt);
    }

    // state covariance
    covT Fk = this->linearizeProcessModel(state, input, dt);
    covT covResult = Fk * P * Fk.transpose() + this->Q;

    return std::make_pair(stateResult, covResult);
}

stateT ExtendedKalmanFilter::processModelBicycle(stateT state, inputT input, double dt) {
    double yaw = state[2];
    double ux = state[3];
    double uy = state[4];
    double r = state[5];

    double throttle = input[0];                 // throttle
    double delta = input[1] * this->vParams.SR; // delta angle

    stateT dot;

    // positions
    // performance optimization
    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);

    // arithmetic integration
    double xDot = (ux * cosyaw - uy * sinyaw);
    double yDot = (ux * sinyaw + uy * cosyaw);
    double yawDot = r;

    // -----
    double F_brake;
    if (throttle < 0)
        F_brake = -this->vParams.max_brake_torque * throttle / this->vParams.r_eff;
    else
        F_brake = 0;
    // Drag
    double F_aero = 0.5 * this->vParams.c_d * this->vParams.rho * this->vParams.A * ux * ux;
    // Rolling resistance, c_r0 here needs to be zero because of v_y_dot formula
    double R_x = this->vParams.c_r0 + this->vParams.c_r1 * abs(ux) + this->vParams.c_r2 * ux * ux;
    double Fload = F_brake + F_aero + R_x;

    double T_motor = ProcessModel::powertrain(throttle, ux, this->vParams, this->eParams);
    double F_load_eff = Fload - this->vParams.m * r * uy;
    double J_m = this->vParams.I_t * this->vParams.GR * this->vParams.GR + this->vParams.I_w + this->vParams.m * this->vParams.r_eff * this->vParams.r_eff;

    double vxDot = this->vParams.r_eff * (T_motor * this->vParams.GR - F_load_eff * this->vParams.r_eff) / J_m;
    double delta_dot = (delta - this->deltaPrev) / dt;
    // double vyDot = (delta_dot * ux + vxDot * delta) * this->vParams.l_r / this->vParams.L;
    double vyDot = (delta_dot * ux / (cos(delta) * cos(delta)) + vxDot * tan(delta)) * this->vParams.l_r / this->vParams.L;
    double rDot = vyDot / this->vParams.l_r;
    this->deltaPrev = delta;
    // -----

    dot << xDot, yDot, yawDot, vxDot, vyDot, rDot;
    state = state + dt * dot;

    return state;
}

// implement dynamic model
stateT ExtendedKalmanFilter::processModelDynamic(stateT state, inputT input, double dt) {
    double yaw = state[2];
    double ux = state[3];
    double uy = state[4];
    double r = state[5];

    double throttle = input[0];                 // throttle
    double delta = input[1] * this->vParams.SR; // delta angle

    stateT dot;

    // positions
    // performance optimization
    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);

    // arithmetic integration
    double xDot = (ux * cosyaw - uy * sinyaw);
    double yDot = (ux * sinyaw + uy * cosyaw);
    double yawDot = r;

    // -----
    double F_brake;
    if (throttle < 0)
        F_brake = -this->vParams.max_brake_torque * throttle / this->vParams.r_eff;
    else
        F_brake = 0;
    // Drag
    double F_aero = 0.5 * this->vParams.c_d * this->vParams.rho * this->vParams.A * ux * ux;
    // Rolling resistance, c_r0 here needs to be zero because of v_y_dot formula
    double R_x = this->vParams.c_r0 + this->vParams.c_r1 * abs(ux) + this->vParams.c_r2 * ux * ux;
    double Fload = F_brake + F_aero + R_x;

    double T_motor = ProcessModel::powertrain(throttle, ux, this->vParams, this->eParams);
    double F_load_eff = Fload - this->vParams.m * r * uy;
    double J_m = this->vParams.I_t * this->vParams.GR * this->vParams.GR + this->vParams.I_w + this->vParams.m * this->vParams.r_eff * this->vParams.r_eff;

    double vxDot = this->vParams.r_eff * (T_motor * this->vParams.GR - F_load_eff * this->vParams.r_eff) / J_m;

    double l_f = this->vParams.L - this->vParams.l_r;

    // tire slip angles
    double a_f = 0;
    if (fabs(ux) > 0.01) {
        a_f = delta - atan((uy + (this->vParams.L - this->vParams.l_r) * r) / ux);
    }
    double a_r = 0.0;
    if (fabs(ux) > 0.01) {
        a_r = -atan((uy - this->vParams.l_r * r) / ux);
    }

    // normal tire force calculations
    double N_r = (this->vParams.m * GRAVITY * l_f) / this->vParams.L;
    double N_f = this->vParams.m * GRAVITY - N_r;
    // Adding down force
    N_f = N_f + ux * ux * this->vParams.Cl * this->vParams.DF_PercentageFront;
    N_r = N_r + ux * ux * this->vParams.Cl * (1 - this->vParams.DF_PercentageFront);

    // tire model for lateral force
    double F_yf = 2 * ProcessModel::tireFy(a_f, N_f / 2.0, ux);
    double F_yr = 2 * ProcessModel::tireFy(a_r, N_r / 2.0, ux);

    double vyDot = (F_yr + F_yf * cos(delta) - this->vParams.m * ux * r) / this->vParams.m;
    // if(ux < 0.01) {
    //     vyDot = 0.0;
    // }
    // double tvTorque = ProcessModel::tvTorque(delta, ux, r, this->vParams, this->eParams);
    double rDot = (F_yf * l_f * cos(delta) - F_yr * this->vParams.l_r /* + tvTorque */) / this->vParams.I_z;
    // -----

    dot << xDot, yDot, yawDot, vxDot, vyDot, rDot;

    state = state + dt * dot;

    if (state[3] < 0.01) {
        state[3] = 0;
        state[4] = 0;
        state[5] = 0;
    }

    return state;
}

// implement blended model
stateT ExtendedKalmanFilter::processModelBlended(stateT state, inputT input, double dt) {
    double ux = state[0];
    double lambda = std::min(std::max((ux - this->lowSpeed) / (this->highSpeed - this->lowSpeed), 0.0), 1.0);
    stateT bicycleState = this->processModelBicycle(state, input, dt);
    stateT dynamicState = this->processModelDynamic(state, input, dt);
    stateT blendedState = (1 - lambda) * bicycleState + lambda * dynamicState;
    return blendedState;
}

covT ExtendedKalmanFilter::linearizeProcessModel(stateT state, inputT input, double dt) {
    covT F;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            stateT stateA = state;
            stateT stateB = state;

            // discrete derivative
            double d = 1e-15;
            stateA[j] -= d;
            stateB[j] += d;

            // fi(a-d)
            if (this->model == "dynamic") {
                stateA = this->processModelDynamic(state, input, dt);

            } else if (this->model == "blended") {
                stateA = processModelBlended(state, input, dt);

            } else { // fall back to bicyclemodel
                stateA = processModelBicycle(state, input, dt);
            }

            // fi(a+d)
            if (this->model == "dynamic") {
                stateB = this->processModelDynamic(state, input, dt);

            } else if (this->model == "blended") {
                stateB = processModelBlended(state, input, dt);

            } else { // fall back to bicyclemodel
                stateB = processModelBicycle(state, input, dt);
            }

            // fi(a+d) - fi(a-d)
            stateT tempState;
            tempState = stateB - stateA;

            //(fi(a+d) - fi(a-d))/(2*d)
            F(i, j) = tempState[i] / (2 * d);
        }
    }
    return F;
}

estimationT ExtendedKalmanFilter::update(
    stateT state,
    covT P,
    msgT Hk,  // observation
    msgT zk,  // measurement
    msgT H,   // observation matrix
    msgT R) { // covariance matrix
    covT I;
    I.setIdentity(6, 6);

    msgT yk = zk - Hk;
    msgT Sk = H * P * H.transpose() + R;

    // old slow way
    if (Sk.determinant() == 0) {
        std::cout << "1/Sk is 0. Skipping update." << std::endl;
    }

    msgT Kk = P * H.transpose() * Sk.inverse();

    stateT computedState = state + Kk * yk;
    covT computedCov = (I - Kk * H) * P;

    return std::make_pair(computedState, computedCov);
}
