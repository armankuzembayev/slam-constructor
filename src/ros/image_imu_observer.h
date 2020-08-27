#ifndef SLAM_CTOR_ROS_IMAGE_IMU_OBSERVER_H_INCLUDED
#define SLAM_CTOR_ROS_IMAGE_IMU_OBSERVER_H_INCLUDED

#include <iostream>
#include <queue>
#include <map>
#include <utility>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>

#include "topic_with_transform.h"
#include "../core/states/world.h"
#include "vins_estimator/parameters.h"
#include "../core/states/imu_image_states.h"
#include "vins_estimator/feature_tracker.h"
#include "vins_estimator/estimator.h"
#include "init_vio.h"

class ImageImuObserver: public TopicObserver<ImuImageMsgs> {
  using ImageImuPtr = boost::shared_ptr<ImuImageMsgs>;
  using DsPtr = std::shared_ptr<InitVio>;
  using EstimatorPtr = std::shared_ptr<Estimator>;
public:
  ImageImuObserver(ros::NodeHandle& nh, const int& image_queue_size,
                   const int& imu_queue_size, const int& feature_queue_size,
                   EstimatorPtr estimator, DsPtr vio): nh_(nh), estimator_(estimator), vio_(vio) {
    for (int i = 0; i < NUM_OF_CAM; i++)
          trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
    sub_img_ = nh_.subscribe(IMAGE_TOPIC, image_queue_size, &ImageImuObserver::img_callback, this);
    sub_imu_ = nh_.subscribe(IMU_TOPIC, imu_queue_size, &ImageImuObserver::imu_callback, this);
    pub_match_ = nh_.advertise<sensor_msgs::Image>("feature_img", feature_queue_size);

  }
  void handle_transformed_msg(const ImageImuPtr msg,
                              const tf::StampedTransform& t) override {}
  void handle_msg() override {
    std::vector<ImuImageMeasurements> measurements;
    while (true) {
      if (vio_->imu_buf_get().empty() || feature_buf.empty())
        break;

      if (!(vio_->imu_buf_get().back()->header.stamp.toSec() > feature_buf.front().first.stamp.toSec() + estimator_->td)) {
        //ROS_WARN("wait for imu, only should happen at the beginning");
        sum_of_wait++;
        break;
      }

      if (!(vio_->imu_buf_get().front()->header.stamp.toSec() < feature_buf.front().first.stamp.toSec() + estimator_->td)) {
        // ROS_WARN("throw img, only should happen at the beginning");
        feature_buf.pop();
        continue;
      }
      std::pair<std_msgs::Header, Feature_storage> img_msg = feature_buf.front();
      feature_buf.pop();

      std::vector<sensor_msgs::ImuConstPtr> IMUs;
      while (vio_->imu_buf_get().front()->header.stamp.toSec() < img_msg.first.stamp.toSec() + estimator_->td) {
        IMUs.emplace_back(vio_->imu_buf_get().front());
        vio_->imu_buf_pop();
      }
      IMUs.emplace_back(vio_->imu_buf_get().front());
      if (IMUs.empty())
        ROS_WARN("no imu between two image");

      measurements.emplace_back(IMUs, img_msg);
    }
      if (measurements.size() != 0) {
        vio_->handle_sensor_data(measurements);
      }
  }

private:
  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    vio_->update_imu(imu_msg);
    handle_msg();
  }

  void restart(const bool restart_flag) {
    if (restart_flag == true) {
        ROS_WARN("restart the estimator!");
        // m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!vio_->imu_buf_get().empty())
            vio_->imu_buf_pop();
        estimator_->clearState();
        estimator_->setParameter();
        current_time = -1;
        vio_->current_time_update(current_time);
        last_imu_t = 0;
    }
    return;
  }

  void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    if(first_image_flag) {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time) {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        bool restart_flag = true;
	      restart(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
     if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }
    if (PUB_THIS_FRAME)
    {
        Feature_storage feature_storage_;
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++) {
                if (trackerData[i].track_cnt[j] > 1) {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
            		    int v = p_id * NUM_OF_CAM + i + 0.5;
            		    int feature_id = v / NUM_OF_CAM;
            		    int camera_id = v % NUM_OF_CAM;
            		    double x = un_pts[j].x;
            		    double y = un_pts[j].y;
            		    double z = 1;
            		    double p_u = cur_pts[j].x;
            		    double p_v = cur_pts[j].y;
            		    double velocity_x = pts_velocity[j].x;
            		    double velocity_y = pts_velocity[j].y;
            		    ROS_ASSERT(z == 1);
            		    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            		    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            		    feature_storage_[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
            }
        }
        if (!init_pub) {
            init_pub = 1;
        }
        else
	         feature_buf.emplace(img_msg->header, feature_storage_);

        if (SHOW_TRACK) {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++) {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++) {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE_CAM);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }
            pub_match_.publish(ptr->toImageMsg());
        }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_img_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_match_;

  FeatureTracker trackerData[NUM_OF_CAM];
  EstimatorPtr estimator_;
  DsPtr vio_;

  double last_imu_t = 0;
  double current_time = -1;
  int pub_count = 1;
  bool init_pub = 0;

  queue<std::pair<std_msgs::Header, Feature_storage>> feature_buf;
  int sum_of_wait = 0;

  bool first_image_flag = true;
  double first_image_time;
  double last_image_time = 0;
};

#endif
