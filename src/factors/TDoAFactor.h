//
// Created by xr on 11/29/23.
//

#ifndef BUILD_TDOAFACTOR_H
#define BUILD_TDOAFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include "../utility.h"

class TDoAFactor : public ceres::SizedCostFunction<1, 3, 3, 3, 7> {
    // res 1D (tdoa), 2 anchor positons in 3D, 1 tag extrinsic, 1 transf R,t
public:
    double  tdoa_meas;
    Eigen::Vector3d W_p_I;
    Eigen::Matrix3d W_R_I;
    double  scale;

    TDoAFactor(
        const double &tdoa_,
        const Eigen::Vector3d & W_p_I_,
        const Eigen::Matrix3d & W_R_I_,
        const double& scale_):
        tdoa_meas(tdoa_),
        W_p_I(W_p_I_),
        W_R_I(W_R_I_),
        scale(scale_) {}

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        // parse parameters
        Eigen::Vector3d a_s(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d a_i(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d I_p_t(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Vector3d U_p_W(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Quaterniond U_q_W(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

        // compute tag position in UWB system
        Eigen::Vector3d W_p_t = W_R_I * I_p_t + W_p_I;
        Eigen::Vector3d U_p_t = U_q_W * W_p_t + U_p_W;

        // compute error
        Eigen::Vector3d relative_position_ai_t = a_i - U_p_t;
        double d_ai_t = relative_position_ai_t.norm();

        Eigen::Vector3d relative_position_as_t = a_s - U_p_t;
        double d_as_t = relative_position_as_t.norm();

        double tdoa_pred = d_as_t - d_ai_t;

        residuals[0] = scale*(tdoa_pred - tdoa_meas);

        // compute Jacobians
        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Vector3d> jacobian_x(jacobians[0]);
                Eigen::Vector3d H1_vec(scale * relative_position_as_t / d_as_t);
                jacobian_x << H1_vec[0], H1_vec[1], H1_vec[2];
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Vector3d> jacobian_x(jacobians[1]);
                Eigen::Vector3d H2_vec(- scale * relative_position_ai_t / d_ai_t);
                jacobian_x << H2_vec[0], H2_vec[1], H2_vec[2];
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Vector3d> jacobian(jacobians[2]);
                jacobian.setZero();
                jacobian << U_q_W * W_R_I * ((relative_position_ai_t / d_ai_t) -  (relative_position_as_t / d_as_t));
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian(jacobians[3]);
                jacobian.setZero();
                Eigen::Vector3d Jp((relative_position_ai_t / d_ai_t) -  (relative_position_as_t / d_as_t));

                Eigen::Matrix3d R(U_q_W.toRotationMatrix() * Util::skewSymmetric(W_p_t));
                Eigen::Vector3d JR(R * ((relative_position_as_t / d_as_t) - (relative_position_ai_t / d_ai_t)));

                jacobian << Jp[0], Jp[1], Jp[2], JR[0], JR[1], JR[2], 0.0;
            }
        }
        return true;
    }
};

#endif //BUILD_TDOAFACTOR_H
