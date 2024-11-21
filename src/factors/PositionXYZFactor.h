//
// Created by xr on 12/9/23.
//

#ifndef BUILD_POSITIONXYZFACTOR_H
#define BUILD_POSITIONXYZFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include "../utility.h"

class PositionXYZFactor : public ceres::SizedCostFunction<3, 3> { // residual 3D, var 3D
public:
    double  scale;
    Eigen::Vector3d x_prior_;
    PositionXYZFactor(const Eigen::Vector3d &x_prior, const double& scale_) : x_prior_(x_prior), scale(scale_) {}

    bool Evaluate(double const*  const* parameters, double* residuals, double** jacobians) const {
        Eigen::Vector3d x(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d diff = scale * (x - x_prior_);
        residuals[0] = diff.x();
        residuals[1] = diff.y();
        residuals[2] = diff.z();

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix3d> jacobian_x(jacobians[0]);
                jacobian_x << scale * 1.0, 0.0, 0.0, 0.0, scale * 1.0, 0.0, 0.0, 0.0, scale * 1.0;
            }
        }
        return true;
    }

};

#endif //BUILD_POSITIONXYZFACTOR_H
