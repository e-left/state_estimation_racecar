#ifndef PROCESS_MODEL_H
#define PROCESS_MODEL_H

#include "turtle_common/ECUParams.hpp"
#include "turtle_common/VehicleParams.h"
#include <math.h>

constexpr double GRAVITY = 9.81;

namespace ProcessModel {

// double inline tvTorque(double delta, double ux, double r, VehicleParams &vParams, ECUParams &eParams) {
//     double rKin = ux * std::sin(delta) / vParams.L;
//     return (rKin - r);
// }

double inline powertrain(double throttle, double ux, VehicleParams &vParams, ECUParams &eParams) {
    double omegaRear = vParams.GR * ux / vParams.r_eff;

    double motorTorque = throttle * eParams.inverter_i_rms_max * vParams.motor_k;

    // clamp between limits
    double maxTorque = omegaRear > 10 ? 1000 * eParams.power_target_kw / omegaRear : eParams.inverter_i_rms_max * vParams.motor_k;

    // maximum regenerative torque
    double minTorque = omegaRear > 10 ? -vParams.batt_max_charging_idc * vParams.batt_min_vdc / omegaRear : 0;
    motorTorque = std::min(motorTorque, maxTorque);
    motorTorque = std::max(motorTorque, minTorque);
    return motorTorque;
}

double tireFy(double a, double Fz, double ux) {
    // alpha = rad2deg(alpha); //Inputs in rad -> make it degrees
    a = a * 180 / M_PI;
    Fz = Fz / 1000; // Input in Newtons -> make in KN
    double camber = 0;

    // Lateral force
    double a0 = 1.509415301550684;
    double a1 = -113.8011661054276523;
    double a2 = 1.780756948582455795 * 1000;
    double a3 = 350.00;
    double a4 = 1.1;
    double a5 = -0.03;
    double a6 = 0.6000000000000;
    double a7 = -1.0202200000000000;
    double a8 = 0.015379596200965;
    double a9 = -0.078706902542296;
    double a10 = -0.1;
    double a11 = -0.0011796790292920;
    double a12 = 0;
    double a13 = -4;
    double a14 = -10;
    double a15 = 0.003;
    double a16 = 0.1;
    double a17 = 0.000001;

    double Cy = a0;                                                      // Shape factor
    double Hy = a8 * Fz + a9 + a10 * camber;                             // Horizontal shift
    double Dy = Fz * (a1 * Fz + a2) * (1 - (a15 * camber * camber));     // Peak factor
    double BCDy = a3 * sin(atan(Fz / a4) * 2) * (1 - a5 * fabs(camber)); // Stiffness
    double By = BCDy / (Cy * Dy);                                        // Stiffness factor

    int sign = 0;
    if (a + Hy > 0) {
        sign = 1;
    } else if (a + Hy < 0) {
        sign = -1;
    }

    double Ey = (a6 * Fz + a7) * (1 - (a16 * camber + a17) * sign); // Curvature factor
    double Vy = a11 * Fz + a12 + (a13 * Fz + a14) * camber * Fz;    // Vertical shift

    if (ux < 0.1) {
        Hy = 0;
        Vy = 0;
    }

    // Returning the lateral force
    // This returns for positive slip angles negative lateral forces, following the Pacejka documentation
    return (Dy * sin(Cy * atan(By * (a + Hy) - Ey * atan(By * (a + Hy) - atan(By * (a + Hy))))) + Vy);
}

} // namespace ProcessModel

#endif
