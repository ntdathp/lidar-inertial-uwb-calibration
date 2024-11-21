//
// Created by xr on 12/19/23.
//

#ifndef BUILD_POSITIONZFACTOR_H
#define BUILD_POSITIONZFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include "../utility.h"

class PositionZFactor : public ceres::SizedCostFunction<1, 3> { // residual 1D, var 3D
public:
    double  scale;
    double z_prior;
    PositionZFactor(const double & z_prior_, const double & scale_) : z_prior(z_prior_), scale(scale_) {}

    bool Evaluate(double const*  const* parameters, double* residuals, double** jacobians) const {
        Eigen::Vector3d x(parameters[0][0], parameters[0][1], parameters[0][2]);
        residuals[0] = scale * (x.z() - z_prior);

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Vector3d> jacobian_x(jacobians[0]);
                jacobian_x << 0.0, 0.0, scale * 1.0;
            }
        }
        return true;
    }

};




#endif //BUILD_POSITIONZFACTOR_H
