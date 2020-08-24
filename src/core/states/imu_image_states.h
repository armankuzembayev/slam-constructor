#ifndef SLAM_CTOR_CORE_IMU_IMAGE_STATES_H
#define SLAM_CTOR_CORE_IMU_IMAGE_STATES_H

#include <map>
#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/Imu.h>

struct ImuImageMsgs {
    sensor_msgs::ImuConstPtr imu;
    sensor_msgs::ImageConstPtr image;
};

using Feature_storage = std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;
using ImuImageMeasurements = std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::pair<std_msgs::Header, Feature_storage>>;

#endif
