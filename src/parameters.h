#pragma once

#include <iostream>
#include <string> 
#include <fstream>
#include <ros/ros.h>
#include <gtsam/base/Vector.h>

extern int  n_iter,
            n_iter_each_opt,
            n_splits,
            discard_nth_split,
            robust_loss_fn;

extern bool mode_fine_opt;

extern double   td,
                scale_res_a2a,
                scale_res_tdoa,
                scale_res_prior,
                huber_loss_tdoa;

extern Eigen::Vector3d L_t_U;

void readConfigFile(ros::NodeHandle &n);