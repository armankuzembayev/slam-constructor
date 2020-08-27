#ifndef SLAM_CTOR_UTILS_INIT_VIO_H
#define SLAM_CTOR_UTILS_INIT_VIO_H

#include "../core/states/world.h"
#include "../core/states/imu_image_states.h"
#include "vins_estimator/parameters.h"
#include "vins_estimator/utility/visualization.h"


class InitVio: public SensorDataObserver<std::vector<ImuImageMeasurements>> {
  using EstimatorPtr = std::shared_ptr<Estimator>;
public:
  InitVio(EstimatorPtr estimator): estimator_(estimator) {}
  
  void handle_sensor_data(std::vector<ImuImageMeasurements>& measurements) override {
        for (auto &measurement : measurements) {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg.first.stamp.toSec() + estimator_->td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator_->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator_->processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg.first.stamp.toSec());

            TicToc t_s;
            std_msgs::Header image_header;
	          image_header = img_msg.first;
            estimator_->processImage(img_msg.second, image_header);
	          std_msgs::Header header = image_header;
            header.frame_id = "world";

            pubOdometry(*estimator_, header);
            pubKeyPoses(*estimator_, header);
            pubCameraPose(*estimator_, header);
            pubPointCloud(*estimator_, header);
            pubTF(*estimator_, header);
            pubKeyframe(*estimator_);
            // ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        if (estimator_->solver_flag == Estimator::SolverFlag::NON_LINEAR){}
            update();
    handle_observation();
  }

  void handle_observation() {
    if (estimator_->solver_flag == Estimator::SolverFlag::NON_LINEAR)
        ROS_INFO("position: %f, %f, %f\r", estimator_->Ps[WINDOW_SIZE].x(), estimator_->Ps[WINDOW_SIZE].y(), estimator_->Ps[WINDOW_SIZE].z());
  }

  void current_time_update(double curr_time) {
    current_time = curr_time;
  }

  void update_imu(const sensor_msgs::ImuConstPtr &imu_msg) {
    imu_buf.push(imu_msg);
    predict(imu_msg);
    std_msgs::Header header = imu_msg->header;
    header.frame_id = "world";
    if (estimator_->solver_flag == Estimator::SolverFlag::NON_LINEAR)
        pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
  }

  queue<sensor_msgs::ImuConstPtr> imu_buf_get() {
    return imu_buf;
  }

  void imu_buf_pop() {
    imu_buf.pop();
  }

  void predict(const sensor_msgs::ImuConstPtr &imu_msg) {
    double t = imu_msg->header.stamp.toSec();
    if (init_imu) {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator_->g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator_->g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update() {
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator_->Ps[WINDOW_SIZE];
    tmp_Q = estimator_->Rs[WINDOW_SIZE];
    tmp_V = estimator_->Vs[WINDOW_SIZE];
    tmp_Ba = estimator_->Bas[WINDOW_SIZE];
    tmp_Bg = estimator_->Bgs[WINDOW_SIZE];
    acc_0 = estimator_->acc_0;
    gyr_0 = estimator_->gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

private:
  EstimatorPtr estimator_;

  queue<sensor_msgs::ImuConstPtr> imu_buf;

  double current_time = -1;
  double latest_time;
  Eigen::Vector3d tmp_P;
  Eigen::Quaterniond tmp_Q;
  Eigen::Vector3d tmp_V;
  Eigen::Vector3d tmp_Ba;
  Eigen::Vector3d tmp_Bg;
  Eigen::Vector3d acc_0;
  Eigen::Vector3d gyr_0;
  bool init_feature = 0;
  bool init_imu = 1;
  double last_imu_t = 0;
};


#endif
