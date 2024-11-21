//
// Created by xr on 11/24/23.
//

#ifndef BUILD_TAGPOSITIONXYFACTOR_H
#define BUILD_TAGPOSITIONXYFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include "../utility.h"

// SizedCostFunction with residual in 2D (x,y), variables in 3D (IMU_p_tag) and 7D (U_T_W)
class TagPositionXYFactor : public ceres::SizedCostFunction<2, 3, 7> {
public:
    Eigen::Vector2d xy_meas;
    Eigen::Vector3d W_p_I;
    Eigen::Matrix3d W_R_I;
    double  scale;
    long    tag_id;

    TagPositionXYFactor(
            long tag_id_,
            const Eigen::Vector2d & xy_meas_,
            const Eigen::Vector3d & W_p_I_,
            const Eigen::Matrix3d & W_R_I_,
            const double& scale_):
            tag_id(tag_id_),
            xy_meas(xy_meas_),
            W_p_I(W_p_I_),
            W_R_I(W_R_I_),
            scale(scale_) {}

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        // parse parameters
        Eigen::Vector3d I_p_t(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d U_p_W(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond U_q_W(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        // compute error
        Eigen::Vector3d W_p_t = W_R_I * I_p_t + W_p_I;
        Eigen::Vector3d U_p_t = U_q_W * W_p_t + U_p_W;

        // Take only xy coordinates
        Eigen::Vector2d xy_pred = U_p_t.head(2);
        Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residuals);
        residual = xy_pred - xy_meas;

        // ROS_INFO("tag id %d,pos %f %f %f, residual %f %f", tag_id, I_p_t.x(), I_p_t.y(), I_p_t.z(), residual.x(), residual.y());

        // compute Jacobians
        if (jacobians)
        {
            if (jacobians[0]) // w.r.t. I_p_t
            {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[0]);
                jacobian.setZero();
                Eigen::Matrix3d J(U_q_W * W_R_I);
                jacobian.block<2, 3>(0, 0) = J.block<2, 3>(0, 0);
            }
            if (jacobians[1]) // w.r.t. U_T_W
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian(jacobians[1]);
                jacobian.setZero();
                jacobian.block<2, 3>(0, 0).setIdentity();

                Eigen::Matrix3d J(- U_q_W.toRotationMatrix() * Util::skewSymmetric(W_p_t));
                jacobian.block<2, 3>(0, 3) = J.block<2, 3>(0, 0);
            }
        }
        return true;
    }
};

#endif //BUILD_TAGPOSITIONXYFACTOR_H
