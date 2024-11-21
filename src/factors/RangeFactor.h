//
// Created by xr on 12/9/23.
//

#ifndef BUILD_RANGEFACTOR_H
#define BUILD_RANGEFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include "../utility.h"

class RangeFactor : public ceres::SizedCostFunction<1, 3, 3> { // residual 1D, var 1 and 2 3D
public:
    double range_meas;
    double scale;
    RangeFactor(const double &range, const double& scale_): range_meas(range), scale(scale_) {}
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

        Eigen::Vector3d a_i(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d a_j(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d relative_position = a_i - a_j;
        double range_pred = relative_position.norm();
        residuals[0] = scale * (range_pred - range_meas);

        if (jacobians)
        {
            Eigen::Vector3d J_vec(scale * relative_position / range_pred);
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Vector3d> jacobian_a_i(jacobians[0], 3);
                jacobian_a_i << J_vec[0], J_vec[1], J_vec[2];
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Vector3d> jacobian_a_j(jacobians[1], 3);
                jacobian_a_j << -J_vec[0], -J_vec[1], -J_vec[2];
            }
        }
        return true;
    }
};


#endif //BUILD_RANGEFACTOR_H
