#include <mutex>
#include <deque>
#include <fstream>
#include <string>
#include <iostream>
#include <filesystem>
#include<tuple> 
#include <random>
#include <uwb_system/json.hpp>

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include "visualization_msgs/Marker.h"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "parameters.h"
#include "factors/TagPositionXYFactor.h"
#include "factors/TDoAFactor.h"
#include "factors/PositionXYZFactor.h"
#include "factors/PositionZFactor.h"
#include "factors/PoseFactor.h"
#include "factors/RangeFactor.h"
#include "PoseLocalParameterization.h"


using json = nlohmann::json;

// visualization
ros::Publisher vis_pub;

// measurements
typedef std::tuple<long, int, int, double> TupleTDoAFactorData; // id_tag, id_synchronizer, id_anchor, tdoa
typedef std::tuple<long, double, double> TupleTagPositionFactorData; // id_tag, x, y
typedef std::pair<TupleTagPositionFactorData, std::vector<TupleTDoAFactorData>> UWBMeasurement;
std::vector<TupleTagPositionFactorData> meas_vector;
std::vector<TupleTagPositionFactorData> meas_tag_position_vector;
std::vector<nav_msgs::Odometry::ConstPtr> msgs_odometry;
std::deque<int> anchor_ids;
std::deque<gtsam::Vector3> anchor_locations;
std::deque<std::pair<double, std::string>> time_file_pairs;
int USE_EVERY_N_FILES = 1;

bool DISCARD_WITH_RSS = true;
float rss_threshold = -105.0;

enum ROBUST_LOSS_FN_TYPE {
    NONE = 0,
    HUBER = 1,
    CAUCHY = 2,
    ARCTAN = 3
};
ROBUST_LOSS_FN_TYPE robust_loss_fn_type;
long tdoa_meas_index = 0;

// optimization
const int NUM_OF_ANCHORS = 31;
std::map<long, double*>     PARAM_TAG_EXTRINSICS;
std::map<double, double*>   PARAM_UWB_T_SLAM;
std::map<int, double*>      PARAM_ANCHOR;

bool MODE_EXTRINSIC_CALIBRATION = false;
bool MODE_ANCHOR_LOCALIZATION = !MODE_EXTRINSIC_CALIBRATION;
bool MODE_3_TAGS = true;
/*
const int NUM_OF_TAGS = 1;
long tag_ids[1] = { 30304 };
*/
const int NUM_OF_TAGS = 3;
long tag_ids[3] = { 10001910, 10001911, 30304 };

UWBMeasurement process_meas_file(std::string path) {

    // get tag position data
    std::ifstream f(path);
    json data = json::parse(f);
    json tags = data[0]["data"]["debugging"]["positioning_input"]["toas"];

    // get id_tag
    std::string id_tag_str = data[0]["data"]["debugging"]["tagData"][0]["tagId"];
    long id_tag = std::stol(id_tag_str);

    // get x,y
    json position = data[0]["data"]["debugging"]["positioning_output"][0]["status"]["diagnostics"]["main_positioner"]["estimator"]["position"];
    double x = position.at(0);
    double y = position.at(1);

    // mm to m
    x *= 0.001;
    y *= 0.001;

    TupleTagPositionFactorData meas_tag_position(id_tag, x , y);

    // get TDoA data
    std::vector<TupleTDoAFactorData> meas_tdoa;
    meas_tdoa.clear();
    json synchronizers = data[0]["data"]["debugging"]["positioning_input"]["toas"][id_tag_str]; // std::to_string(tag_id).c_str()];
    std::map<int, float> rsss;

    // get rss values
    if(DISCARD_WITH_RSS) {
        json json_rsss = data[0]["data"]["debugging"]["positioning_input"]["rsss"][id_tag_str];
        for (json::iterator it = json_rsss.begin(); it != json_rsss.end(); ++it) {
            int id_anchor = std::stoi(it.key());
            float rss = it.value();
            rsss[id_anchor] = rss;
        }
    }

    // for each synchronizer anchor
    for (json::iterator outer_it = synchronizers.begin(); outer_it != synchronizers.end(); ++outer_it) {
        // create list of pairs<anchor id, tdoa>
        std::vector<std::pair<int, double>> sync_data;

        // fill the list and identify synchronizer anchor
        json sync_data_json = outer_it.value();
        int id_synchronizer = -1;
        for (json::iterator inner_it = sync_data_json.begin(); inner_it != sync_data_json.end(); ++inner_it) {
            // std::cout << std::setw(2) << inner_it.key() << " : " << inner_it.value() << "\n";
            double tdoa_time = inner_it.value();
            double tdoa = -tdoa_time / 1e3;
            // we use - as in TDoAFactor, we have: double tdoa_pred = d_as_t - d_ai_t;
            int id_anchor = std::stoi(inner_it.key());

            // assign it as synchronizer or create tdoa tuple
            if (tdoa == 0.0) {
                id_synchronizer = id_anchor;
            } else {
                if (DISCARD_WITH_RSS) {
                    if (rsss[id_anchor] < rss_threshold) {
                        continue;
                    }
                }
                sync_data.push_back(std::pair<int, double>(id_anchor, tdoa));
            }
        }

        // create measurement tuples
        for (std::vector<std::pair<int, double>>::iterator it = sync_data.begin(); it != sync_data.end(); ++it) {
            meas_tdoa.push_back(TupleTDoAFactorData(id_tag, id_synchronizer, it->first, it->second));
        }
    }

    return { meas_tag_position, meas_tdoa };
}

void add_tdoa_and_tag_extrinsics_factors(ceres::Problem & problem, std::string path_odom, std::string path_meas_dir) {

    // read and parse odometry and UWB data
    Util::read_odometry_bag(path_odom, msgs_odometry);
    time_file_pairs = Util::read_meas_directory(path_meas_dir, USE_EVERY_N_FILES);
    ROS_INFO("read odometry and parsed UWB measurements.");

    ROS_INFO("odom starts at: %f", msgs_odometry.front()->header.stamp.toSec());
    ROS_INFO("odom ends at:   %f", msgs_odometry.back()->header.stamp.toSec());
    ROS_INFO("odom duration:  %f",  msgs_odometry.back()->header.stamp.toSec() - msgs_odometry.front()->header.stamp.toSec());
    ROS_INFO("meas starts at: %f", time_file_pairs.front().first);
    ROS_INFO("meas ends at:   %f", time_file_pairs.back().first);
    ROS_INFO("meas duration:  %f", time_file_pairs.back().first - time_file_pairs.front().first);


    // process each measurement file
    int count_valid_meas = 0;
    int id_odom = 0;
    int sgn_search = 1;
    for(auto time_file_it = time_file_pairs.begin();
        time_file_it != time_file_pairs.end();
        ++time_file_it)
    {
        double t_uwb = time_file_it->first;
        UWBMeasurement meas_uwb = process_meas_file(time_file_it->second);
        TupleTagPositionFactorData factor_data = meas_uwb.first;

        // find the pose before and the pose after
        auto msg_odom = msgs_odometry.at(id_odom);
        auto msg_odom_next = msgs_odometry.at(id_odom + 1);

        // check if it fits our needs - CH TO DO: refactor 
        if(msg_odom->header.stamp.toSec() < t_uwb && t_uwb < msg_odom_next->header.stamp.toSec()) {
            // good, it's already the right pose indices
        }
        else {
          // not good, find in which direction to search
          if(t_uwb < msg_odom->header.stamp.toSec()) {
            sgn_search = -1;
          }
          if(t_uwb > msg_odom_next->header.stamp.toSec()) {
            sgn_search = 1;
          }
          // search by shifting current index
          while(true) {
            id_odom += sgn_search;
            // check boundaries and break if needed
            if(id_odom < 0) {
              id_odom -= sgn_search;
              break;
            }
            if(id_odom > msgs_odometry.size() - 2) {
              id_odom -= sgn_search;
              break;
            }
            msg_odom = msgs_odometry.at(id_odom);
            msg_odom_next = msgs_odometry.at(id_odom + 1);
            // check if it fits our needs, if so: break
            if(msg_odom->header.stamp.toSec() < t_uwb && t_uwb < msg_odom_next->header.stamp.toSec()) {
              break;
            }
          }
        }

        // check if it fits
        double t_odom_prev =  msg_odom->header.stamp.toSec();
        double t_odom_next =  msg_odom_next->header.stamp.toSec();
        if(t_uwb > t_odom_prev && t_uwb < t_odom_next) {
            count_valid_meas += 1;

            // process measurement
            long id_tag = std::get<0>(factor_data);
            double x = std::get<1>(factor_data);
            double y = std::get<2>(factor_data);
            Eigen::Vector2d xy_meas(x, y);

            // discard if tag information is not used in current mode
            if(!MODE_3_TAGS && id_tag != tag_ids[0])
                continue;

            // interpolate pose
            double s = (t_uwb - t_odom_prev) / (t_odom_next - t_odom_prev);

            Eigen::Vector3d Pi;
            Eigen::Quaterniond Qi;
            Util::parse_msg_odom_pose(msg_odom, Pi, Qi);

            Eigen::Vector3d Pj;
            Eigen::Quaterniond Qj;
            Util::parse_msg_odom_pose(msg_odom_next, Pj, Qj);

            Eigen::AngleAxis<double> Phi(Qi.inverse() * Qj);
            Eigen::AngleAxis<double> sxPhi(s * Phi.angle(), Phi.axis());
            Eigen::AngleAxis<double> sbxPhi((s - 1) * Phi.angle(), Phi.axis());

            Eigen::Matrix3d W_R_I(Qi * sxPhi);
            Eigen::Vector3d W_p_I = (1 - s) * Pi + s * Pj;

            // set scale
            double scale = 1.0;

            // create tag position factor
            if(MODE_EXTRINSIC_CALIBRATION) {
                TagPositionXYFactor *tag_position_xy_factor = new TagPositionXYFactor(id_tag, xy_meas, W_p_I, W_R_I,
                                                                                      scale);
                problem.AddResidualBlock(tag_position_xy_factor, new ceres::HuberLoss(0.2),
                                         PARAM_TAG_EXTRINSICS[id_tag],
                                         PARAM_UWB_T_SLAM[0]);
            }

            // create tdoa factors
            for (std::vector<TupleTDoAFactorData>::iterator meas_it = meas_uwb.second.begin();
                 meas_it != meas_uwb.second.end(); ++meas_it) {
                int meas_id_sync = std::get<1>(*meas_it);
                int meas_id_anchor = std::get<2>(*meas_it);
                double meas_tdoa = std::get<3>(*meas_it);

                int id_anchor_s = anchorIdToOrder[meas_id_sync];
                int id_anchor_i = anchorIdToOrder[meas_id_anchor];

                // check split
                int tdoa_split_current = tdoa_meas_index % n_splits;
                // std::cout << "tdoa_meas_index    " << tdoa_meas_index << std::endl;
                // std::cout << "tdoa_split_current " << tdoa_split_current << std::endl;
                if(tdoa_split_current != discard_nth_split) {
                    // std::cout << "yes " << std::endl;
                    TDoAFactor* tdoa_factor = new TDoAFactor(meas_tdoa, W_p_I, W_R_I, scale_res_tdoa);
                    ceres::LossFunction* loss_fn;
                    switch(robust_loss_fn_type) {
                        case NONE:
                            loss_fn = NULL;
                            break;
                        case HUBER:
                            loss_fn = new ceres::HuberLoss(huber_loss_tdoa);
                            break;
                        case CAUCHY:
                            loss_fn = new ceres::CauchyLoss(huber_loss_tdoa);
                            break;
                        case ARCTAN:
                            loss_fn = new ceres::ArctanLoss(huber_loss_tdoa);
                            break;
                    }
                    problem.AddResidualBlock(
                        tdoa_factor,
                        loss_fn,
                        PARAM_ANCHOR[id_anchor_s],
                        PARAM_ANCHOR[id_anchor_i],
                        PARAM_TAG_EXTRINSICS[id_tag],
                        PARAM_UWB_T_SLAM[0]);
                }
                tdoa_meas_index += 1;
            }
            // std::cout << "current tdoa meas index: " << tdoa_meas_index << std::endl;
        }
    }

  ROS_INFO("added %d tag position factors", count_valid_meas);
}

void add_anchor_prior_factors(ceres::Problem & problem, std::string path) {
    // check mode
    double scale_res_prior_effective = scale_res_prior;
    if(mode_fine_opt) {
        ROS_INFO("[mode] fine opt: adding strong priors on anchor locations");
        scale_res_prior_effective = scale_res_prior * 10;
    }
    else {
        ROS_INFO("[mode] coarse opt: adding weak priors on few anchors");
        ROS_INFO("[mode] coarse opt: adding weak priors on anchor's height");
    }

    // parse file with known anchors, update initial location and add prior where need be
    std::ifstream f(path);
    json data = json::parse(f);

    for (json::iterator it = data.begin(); it != data.end(); ++it) {
        int id_anchor = anchorIdToOrder[std::stoi(it.key())];
        Eigen::Vector3d anchor_position(it.value()[0], it.value()[1], it.value()[2]);
        PositionXYZFactor* prior_factor = new PositionXYZFactor(anchor_position, scale_res_prior_effective);
        problem.AddResidualBlock(prior_factor, NULL, PARAM_ANCHOR[id_anchor]);
        ROS_INFO("added PriorFactor to anchor %d, scale %f", id_anchor, scale_res_prior_effective);
    }

    if(!mode_fine_opt) {
        for (int a_i = 0; a_i < NUM_OF_ANCHORS; a_i++) {
            PositionZFactor *prior_z_factor = new PositionZFactor(10, scale_res_prior_effective);
            problem.AddResidualBlock(prior_z_factor, NULL, PARAM_ANCHOR[a_i]);
        }
        ROS_INFO("added PriorFactor Z to all anchors");
    }

}

void add_slam_to_world_transform_factor(ceres::Problem & problem) {
    PoseFactor* prior_U_T_W = new PoseFactor(
            Eigen::Vector3d(27.86, 3.2, 0.9),
            Eigen::Quaterniond(0.996283, -0.00405273, 0.00321833, -0.0859795),
            50.0);
    problem.AddResidualBlock(prior_U_T_W, NULL, PARAM_UWB_T_SLAM[0]);
}

void add_tag_extrinsics_priors(ceres::Problem & problem) {

    Eigen::Vector3d I_p_t_dev(-0.0526, -.11, 0.05);
    PositionXYZFactor* prior_factor = new PositionXYZFactor(I_p_t_dev, 100.0);
    problem.AddResidualBlock(prior_factor, NULL, PARAM_TAG_EXTRINSICS[30304]);

    if(MODE_3_TAGS) {
        Eigen::Vector3d I_p_t_10001911(-0.069, 0.08, 0.96);
        PositionXYZFactor* prior_factor_3 = new PositionXYZFactor(I_p_t_10001911, 100.0);
        problem.AddResidualBlock(prior_factor_3, NULL, PARAM_TAG_EXTRINSICS[10001911]);

        Eigen::Vector3d I_p_t_10001910(-.04, 0.06, -0.36);
        PositionXYZFactor* prior_factor_2 = new PositionXYZFactor(I_p_t_10001910, 100.0);
        problem.AddResidualBlock(prior_factor_2, NULL, PARAM_TAG_EXTRINSICS[10001910]);
    }


}

void add_anchor_to_anchor_factors(ceres::Problem & problem, std::string path) {
    std::ifstream f(path);
    json data = json::parse(f);
    float count = 0;
    for (json::iterator outer_it = data.begin(); outer_it != data.end(); ++outer_it) {
        for (json::iterator inner_it = outer_it.value().begin(); inner_it != outer_it.value().end(); ++inner_it) {
            // parse
            int pozyx_id_anchor_i = std::stoi(outer_it.key());
            int pozyx_id_anchor_j = std::stoi(inner_it.key());
            int id_anchor_i = anchorIdToOrder[pozyx_id_anchor_i];
            int id_anchor_j = anchorIdToOrder[pozyx_id_anchor_j];
            double range = double(inner_it.value()["range"]) / 1e3;

            RangeFactor* range_factor = new RangeFactor(range, scale_res_a2a);
            problem.AddResidualBlock(range_factor, NULL, PARAM_ANCHOR[id_anchor_i], PARAM_ANCHOR[id_anchor_j]);
            count += 0.1;
        }
    }
}

void report() {

    // print output
    std::cout << "optimization done." << std::endl;
    if(MODE_EXTRINSIC_CALIBRATION) {
        ROS_INFO("tag extrinsics:");
        for (int tag_i = 0; tag_i < NUM_OF_TAGS; tag_i++) {
            ROS_INFO("tag #%d - %d: %f,\t %f,\t %f",
                     tag_i, tag_ids[tag_i],
                     PARAM_TAG_EXTRINSICS[tag_ids[tag_i]][0], PARAM_TAG_EXTRINSICS[tag_ids[tag_i]][1],
                     PARAM_TAG_EXTRINSICS[tag_ids[tag_i]][2]);
        }
        ROS_INFO("SLAM to UWB system transformation:");
        Eigen::Vector3d U_p_W(PARAM_UWB_T_SLAM[0][0], PARAM_UWB_T_SLAM[0][1], PARAM_UWB_T_SLAM[0][2]);
        Eigen::Quaterniond U_q_W(PARAM_UWB_T_SLAM[0][6], PARAM_UWB_T_SLAM[0][3], PARAM_UWB_T_SLAM[0][4],
                                 PARAM_UWB_T_SLAM[0][5]);
        Eigen::Matrix3d U_R_W(U_q_W.toRotationMatrix());

        std::cout << "position: " << std::endl;
        std::cout << U_p_W << std::endl;
        std::cout << "rotation: " << std::endl;
        std::cout << U_R_W << std::endl;
        std::cout << U_q_W.coeffs() << std::endl;
    }

    std::cout << std::endl << "anchor positions:" << std::endl;
    for(int a_i = 0; a_i < NUM_OF_ANCHORS; a_i++) {
        ROS_INFO("anchor #%d - %d: %f,\t %f,\t %f",
                 a_i, anchorOrderToId[a_i],
                 PARAM_ANCHOR[a_i][0], PARAM_ANCHOR[a_i][1], PARAM_ANCHOR[a_i][2]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uwb_slam_calibrator");
    ros::NodeHandle n;

    // read parameters
    readConfigFile(n);

    robust_loss_fn_type = ROBUST_LOSS_FN_TYPE(robust_loss_fn);
    switch(robust_loss_fn_type) {
        case NONE: std::cout << "Trivial loss" << std::endl;
            break;
        case HUBER: std::cout << "Huber loss" << std::endl;
            break;
        case CAUCHY: std::cout << "Cauchy loss" << std::endl;
            break;
        case ARCTAN: std::cout << "Arctan loss" << std::endl;
            break;
    }
    std::cout << "using: " << robust_loss_fn_type << std::endl;

    // init google logging for ceres
    GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // open log file stream
    std::string path_base = "/home/xr/Documents/research/slam/uwb-cal/";
    std::fstream stream_log(path_base + "eval/out.txt", std::fstream::out);

    // set up tag publisher
    vis_pub = n.advertise<visualization_msgs::Marker>("/uwb_system/tags_estimated", 100);

    // create ceres problem
    ceres::Problem problem;

    bool use_anchors = true;

    // add param for extrinsic calibration
    for(int tag_i = 0; tag_i < NUM_OF_TAGS; tag_i++) {
        PARAM_TAG_EXTRINSICS[tag_ids[tag_i]] = new double[3];
        PARAM_TAG_EXTRINSICS[tag_ids[tag_i]][0] = 0.0;
        PARAM_TAG_EXTRINSICS[tag_ids[tag_i]][1] = 0.0;
        PARAM_TAG_EXTRINSICS[tag_ids[tag_i]][2] = 0.0;
    }

    // add param for anchor location
    if(mode_fine_opt) {
        ROS_INFO("[mode] fine opt: setting initial anchor locations to previous estimate");
        std::ifstream f(path_base + "data/anchors_coarse_estimate.json");
        json data = json::parse(f);

        for (json::iterator it = data.begin(); it != data.end(); ++it) {
            int a_i = anchorIdToOrder[std::stoi(it.key())];

            PARAM_ANCHOR[a_i] = new double[3];
            PARAM_ANCHOR[a_i][0] = it.value()[0];
            PARAM_ANCHOR[a_i][1] = it.value()[1];
            PARAM_ANCHOR[a_i][2] = it.value()[2];
            ROS_INFO("added initial estimate to anchor %d, x %f", a_i, PARAM_ANCHOR[a_i][0]);
        }

    }
    else {
        ROS_INFO("[mode] coarse opt: setting initial anchor locations to pseudo-random values");
        
        // addition
        double mean = 0.0;
        double std_dev = 2.0;

        std::default_random_engine generator;
        std::normal_distribution<double> distribution(mean, std_dev);
        for(int a_i = 0; a_i < NUM_OF_ANCHORS; a_i++) {
            PARAM_ANCHOR[a_i] = new double[3];
            PARAM_ANCHOR[a_i][0] = distribution(generator);
            PARAM_ANCHOR[a_i][1] = distribution(generator);
            PARAM_ANCHOR[a_i][2] = distribution(generator);
        }
    }

    // add initial param for coordinate transformation between SLAM and UWB systems
    // doesn't affect the result, only visualization during the opt
    PARAM_UWB_T_SLAM[0] = new double[7];
    PARAM_UWB_T_SLAM[0][0] = 27.8983;
    PARAM_UWB_T_SLAM[0][1] = 3.15785;
    PARAM_UWB_T_SLAM[0][2] = 1.377243;
    PARAM_UWB_T_SLAM[0][3] = -0.00405273;
    PARAM_UWB_T_SLAM[0][4] = 0.00321833;
    PARAM_UWB_T_SLAM[0][5] = -0.0859795;
    PARAM_UWB_T_SLAM[0][6] = 0.996283; // 6 is w
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(PARAM_UWB_T_SLAM[0], 7, local_parameterization);


    // add factors
    add_anchor_to_anchor_factors(problem, path_base + "data/anchor_ranges.json");
    if(MODE_ANCHOR_LOCALIZATION) {
        if(mode_fine_opt) {
            ROS_INFO("[mode] fine opt: using file with previous estimates");
            add_anchor_prior_factors(problem, path_base + "data/anchors_coarse_estimate.json");
        }
        else {
            ROS_INFO("[mode] coarse opt: no prior");
        }
    }

    add_tag_extrinsics_priors(problem);
    add_tdoa_and_tag_extrinsics_factors(problem, path_base + "data/odometry-run.bag", path_base + "data/run/");
    add_slam_to_world_transform_factor(problem);

    // set constant parameters depending on mode
    if(MODE_ANCHOR_LOCALIZATION) {
        // set I_p_t constant
        // Livox Avia imu coordinate system: x fw, y left, z up
        PARAM_TAG_EXTRINSICS[30304][0] = -0.052601; // this might indicate a small time sync issue
        PARAM_TAG_EXTRINSICS[30304][1] =  -0.117753;
        PARAM_TAG_EXTRINSICS[30304][2] = 0.087502;
        problem.SetParameterBlockConstant(PARAM_TAG_EXTRINSICS[30304]);

        if(MODE_3_TAGS) {
            PARAM_TAG_EXTRINSICS[10001911][0] = -0.069202; // this might indicate a small time sync issue
            PARAM_TAG_EXTRINSICS[10001911][1] = 0.081464;
            PARAM_TAG_EXTRINSICS[10001911][2] = 1.004216;
            problem.SetParameterBlockConstant(PARAM_TAG_EXTRINSICS[10001911]);
            PARAM_TAG_EXTRINSICS[10001910][0] = -0.037316; // this might indicate a small time sync issue
            PARAM_TAG_EXTRINSICS[10001910][1] = 0.060746;
            PARAM_TAG_EXTRINSICS[10001910][2] = -0.362709;
            problem.SetParameterBlockConstant(PARAM_TAG_EXTRINSICS[10001910]);
        }

        // set UWB_T_SLAM constant
        problem.SetParameterBlockConstant(PARAM_UWB_T_SLAM[0]);
    }
    else if(MODE_EXTRINSIC_CALIBRATION) {

        // parse file with known anchors, add parameter and fix it
        std::ifstream f(path_base + "data/known_anchors.json");
        json data = json::parse(f);
        for (json::iterator it = data.begin(); it != data.end(); ++it) {
            int id_anchor = anchorIdToOrder[std::stoi(it.key())];
            Eigen::Vector3d anchor_position(it.value()[0], it.value()[1], it.value()[2]);

            PARAM_ANCHOR[id_anchor][0] = anchor_position.x();
            PARAM_ANCHOR[id_anchor][1] = anchor_position.y();
            PARAM_ANCHOR[id_anchor][2] = anchor_position.z();
            problem.SetParameterBlockConstant(PARAM_ANCHOR[id_anchor]);

            ROS_INFO("added fixed parameter to anchor %d", id_anchor);
        }
    }


    // set solver options
    ceres::Solver::Options options;
    // options.dense_linear_algebra_library_type = ceres::CUDA;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = n_iter;
    ceres::Solver::Summary summary;

    // solve
    ROS_INFO("attempting solve...");
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // publish tags
    Eigen::Vector3d U_p_W(PARAM_UWB_T_SLAM[0][0], PARAM_UWB_T_SLAM[0][1], PARAM_UWB_T_SLAM[0][2]);
    Eigen::Quaterniond U_q_W(PARAM_UWB_T_SLAM[0][6], PARAM_UWB_T_SLAM[0][3], PARAM_UWB_T_SLAM[0][4], PARAM_UWB_T_SLAM[0][5]);

    for (int i = 0; i < NUM_OF_ANCHORS; i++) {
        // to compare with cloud, transform anchor position in slam coordinate frame
        Eigen::Vector3d U_a_i(PARAM_ANCHOR[i][0], PARAM_ANCHOR[i][1], PARAM_ANCHOR[i][2]);
        Eigen::Vector3d W_a_i(U_q_W.toRotationMatrix().transpose() * (U_a_i - U_p_W));

        // publish in rviz
        Util::publish_tag_estimated(i, W_a_i[0], W_a_i[1], W_a_i[2], vis_pub);

        // log for computing metrics
        stream_log  << "0" << ",\t"
                    << anchorOrderToId[i] << ",\t" << std::setprecision(9)
                    << PARAM_ANCHOR[i][0] << ",\t" <<  PARAM_ANCHOR[i][1]<< ",\t" <<  PARAM_ANCHOR[i][2] << std::endl;
    }

    report();

    // close program

    stream_log.close();
    ros::shutdown();
    return 0;
}