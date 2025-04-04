

#pragma once

#ifndef _Util_UWB_SLAM_CALIBRATOR_H_
#define _Util_UWB_SLAM_CALIBRATOR_H_

#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <filesystem>

namespace fs = std::filesystem;

double parse_timestamp(std::string s, char del, double td)
{
    std::stringstream ss(s);
    std::string word;
    while (!ss.eof()) {
        std::getline(ss, word, del);
    }
    // convert string timestamp to double
    // then divide by 1e6 (as it is in microseconds)
    return std::stod(word) / 1e6 + td;
}


std::map<int, int> anchorIdToOrder = {
        {12961, 0}, {13409, 1}, {14649, 2}, {15606, 3}, {16366, 4}, { 16744, 5}, {18744, 6}, {19712, 7},
        {20272, 8}, {20967, 9}, {21044, 10}, {26555, 11}, {28400, 12}, { 28622, 13}, {33643, 14}, {33667, 15},
        {36687, 16}, {36924, 17}, {38189, 18}, {41209, 19}, {4356, 20}, { 46435, 21}, {47419, 22}, {47434, 23},
        {48416, 24}, {52706, 25}, {61680, 26}, {62659, 27}, {62863, 28}, { 65433, 29}, {839, 30}
};

std::map<int, int> anchorOrderToId = {
        {0, 12961}, { 1, 13409}, { 2, 14649}, { 3, 15606}, { 4, 16366}, { 5, 16744}, {6,  18744}, {7, 19712},
        {8, 20272}, { 9, 20967}, { 10, 21044}, { 11, 26555 }, { 12, 28400 }, { 13, 28622 }, {14, 33643 }, {15, 33667 },
        {16, 36687 }, { 17, 36924 }, { 18, 38189 }, { 19, 41209 }, { 20, 4356 }, { 21, 46435 }, {22, 47419}, {23, 47434 },
        {24, 48416 }, {25,  52706 }, { 26,  61680 }, { 27,  62659 }, { 28, 62863 }, { 29, 65433 }, {30, 839}
};

namespace Util {

    void parse_msg_odom_pose(nav_msgs::Odometry::ConstPtr & msg_odom, Eigen::Vector3d & p, Eigen::Quaterniond & q) {
        p.x() = msg_odom->pose.pose.position.x;
        p.y() = msg_odom->pose.pose.position.y;
        p.z() = msg_odom->pose.pose.position.z;

        q.x() = msg_odom->pose.pose.orientation.x;
        q.y() = msg_odom->pose.pose.orientation.y;
        q.z() = msg_odom->pose.pose.orientation.z;
        q.w() = msg_odom->pose.pose.orientation.w;
    }

    std::deque<std::pair<double, std::string>> read_meas_directory(std::string path_meas_dir, const int use_every_n) {
        std::deque<std::pair<double, std::string>> time_file_pairs;

        // open directory, list files and sort them by name
        std::vector<std::string> paths_meas;
        for (const auto & entry : fs::directory_iterator(path_meas_dir)) {
            paths_meas.push_back(entry.path());
        }
        std::sort(paths_meas.begin(), paths_meas.end());

        // for each file, extract timestamp and parse data
        int k = 0;
        for (std::vector<std::string>::iterator i = paths_meas.begin(); i != paths_meas.end(); ++i) {
            double t = parse_timestamp(*i, '_', td);
            std::pair<double, std::string> time_file_pair(t, *i);
            if(k % use_every_n == 0) {
                time_file_pairs.push_back(time_file_pair);
            }
            k++;
        }
        return time_file_pairs;
    }

    void read_odometry_bag(std::string path, std::vector<nav_msgs::Odometry::ConstPtr> & msgs_odometry) {
        rosbag::Bag bag(path.c_str());
        rosbag::View view(bag, rosbag::TopicQuery("/Odometry"));

        int count = 0;
        for (const rosbag::MessageInstance& message : view) {
            nav_msgs::Odometry::ConstPtr odometry_msg = message.instantiate<nav_msgs::Odometry>();
            if (odometry_msg != nullptr) {
                msgs_odometry.push_back(odometry_msg);
                count += 1;
            }
        }
        ROS_INFO("found %d odometry messages.", count);

        bag.close();
    }


    void publish_tag_estimated(int id, double x, double y, double z, ros::Publisher & vis_pub) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = ros::Time();
        marker.ns = "tags_estimated";
        marker.id = 100 + id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = .45;
        marker.color.r = 0.5;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        if(id == 62659) {
            marker.color.r = 0.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
        }
        if(id == 62863) {
            marker.color.r = 0.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
        }
        if(id == 20272) {
            marker.color.r = 0.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
        }
        if(id == 48416) {
            marker.color.r = 0.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
        }
        if(id == 47434) {
            marker.color.r = 0.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
        }
        if(id == 52706) {
            marker.color.r = 0.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
        }
        vis_pub.publish( marker );
    }


    template<typename Derived>
    static typename Derived::Scalar
    angleDiff(const Eigen::QuaternionBase <Derived> &q1, const Eigen::QuaternionBase <Derived> &q2) {
        return (Eigen::AngleAxis<typename Derived::Scalar>(q1.inverse() * q2).angle() * 180.0 / M_PI);
    }



    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }



    template<typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase <Derived> &theta) {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion <Scalar_t> dq;

        Scalar_t theta_nrm = theta.norm();

        if (theta_nrm < 1e-5) {
            Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
            half_theta /= static_cast<Scalar_t>(2.0);
            dq.w() = static_cast<Scalar_t>(1.0);
            dq.x() = half_theta.x();
            dq.y() = half_theta.y();
            dq.z() = half_theta.z();
        } else {
            Scalar_t costheta = cos(theta_nrm / 2);
            Scalar_t sintheta = sin(theta_nrm / 2);
            Eigen::Matrix<Scalar_t, 3, 1> quat_vec = theta / theta_nrm * sintheta;

            dq.w() = costheta;
            dq.vec() = quat_vec;
        }

        // printf("dq: %f, %f, %f, %f. norm: %f\n", dq.x(), dq.y(), dq.z(), dq.w(), dq.norm());

        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
                q(2), typename Derived::Scalar(0), -q(0),
                -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

}

#endif