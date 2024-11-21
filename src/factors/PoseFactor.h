//
// Created by xr on 12/19/23.
//

#ifndef BUILD_POSEFACTOR_H
#define BUILD_POSEFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include "../utility.h"

class PoseFactor : public ceres::SizedCostFunction<6, 7> { // residual 6D, param 7D
public:
    double  scale;
    Eigen::Vector3d p_prior;
    Eigen::Quaterniond q_prior;
    PoseFactor(const Eigen::Vector3d &p_prior_, const Eigen::Quaterniond q_prior_, const double& scale_) :
    p_prior(p_prior_), q_prior(q_prior_), scale(scale_) {}

    bool Evaluate(double const*  const* parameters, double* residuals, double** jacobians) const {
        Eigen::Vector3d p(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.block<3, 1>(0, 0) = p - p_prior;
        residual.block<3, 1>(3, 0) = 2 * (q_prior.inverse() * q).vec();
        residual = scale * residual;

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
                jacobian_pose.setZero();
                jacobian_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                jacobian_pose.block<3, 3>(3, 3) = Util::Qleft(q_prior.inverse() * q).bottomRightCorner<3, 3>();
                jacobian_pose = scale * jacobian_pose;
            }
        }
        return true;
    }

};



#endif //BUILD_POSEFACTOR_H
