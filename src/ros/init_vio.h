#ifndef SLAM_CTOR_UTILS_INIT_VIO_H
#define SLAM_CTOR_UTILS_INIT_VIO_H

#include "../core/states/world.h"
#include "../core/states/imu_image_states.h"
#include "../utils/parameters.h"
#include "utility/visualization.h"


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

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

  void readParameters(ros::NodeHandle &n) {
      std::string config_file;
      config_file = readParam<std::string>(n, "config_file");
      cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
      if(!fsSettings.isOpened()) {
          std::cerr << "ERROR: Wrong path to settings" << std::endl;
      }

      //camera params
      fsSettings["imu_topic"] >> IMU_TOPIC;
      fsSettings["image_topic"] >> IMAGE_TOPIC;
      MAX_CNT = fsSettings["max_cnt"];
      MIN_DIST = fsSettings["min_dist"];
      FREQ = fsSettings["freq"];
      F_THRESHOLD = fsSettings["F_threshold"];
      SHOW_TRACK = fsSettings["show_track"];
      EQUALIZE = fsSettings["equalize"];
      FISHEYE = fsSettings["fisheye"];

      CAM_NAMES.push_back(config_file);

      WINDOW_SIZE_CAM = 20;
      STEREO_TRACK = false;
      FOCAL_LENGTH = 460;
      PUB_THIS_FRAME = false;

      if (FREQ == 0)
          FREQ = 100;


      //pose_grap params
      // read param
      n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
      n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
      n.getParam("skip_cnt", SKIP_CNT);
      n.getParam("skip_dis", SKIP_DIS);

      CAMERA_VISUAL_SIZE = fsSettings["visualize_camera_size"];
      LOOP_CLOSURE = fsSettings["loop_closure"];
      if (LOOP_CLOSURE)
      {
          std::string pkg_path = ros::package::getPath("slam_constructor");
          std::string VOCABULARY_FILE = pkg_path + "/config/vio/support_files/brief_k10L6.bin";
        	std::string BRIEF_PATTERN_FILE = pkg_path + "/config/vio/support_files/brief_pattern.yml";
          std::cout << "VOCABULARY_FILE" << VOCABULARY_FILE << std::endl;
        	std::cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << std::endl;
        	m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        	fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        	fsSettings["save_image"] >> DEBUG_IMAGE;
        	FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
          VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
          LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
          FAST_RELOCALIZATION = fsSettings["fast_relocalization"];

      }


      SOLVER_TIME = fsSettings["max_solver_time"];
      NUM_ITERATIONS = fsSettings["max_num_iterations"];
      MIN_PARALLAX = fsSettings["keyframe_parallax"];
      MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

      std::string OUTPUT_PATH;
      fsSettings["output_path"] >> OUTPUT_PATH;
      VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
      std::cout << "result path " << VINS_RESULT_PATH << std::endl;

      // create folder if not exists
      FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

      std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
      fout.close();

      ACC_N = fsSettings["acc_n"];
      ACC_W = fsSettings["acc_w"];
      GYR_N = fsSettings["gyr_n"];
      GYR_W = fsSettings["gyr_w"];
      G.z() = fsSettings["g_norm"];
      ROW = fsSettings["image_height"];
      COL = fsSettings["image_width"];
      ROS_INFO("ROW: %f COL: %f ", ROW, COL);

      ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
      if (ESTIMATE_EXTRINSIC == 2)
      {
          ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
          RIC.push_back(Eigen::Matrix3d::Identity());
          TIC.push_back(Eigen::Vector3d::Zero());
          EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
      }
      else
      {
          if ( ESTIMATE_EXTRINSIC == 1)
          {
              ROS_WARN(" Optimize extrinsic param around initial guess!");
              EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
          }
          if (ESTIMATE_EXTRINSIC == 0)
              ROS_WARN(" fix extrinsic param ");

          cv::Mat cv_R, cv_T;
          				fsSettings["extrinsicRotation"] >> cv_R;
          fsSettings["extrinsicTranslation"] >> cv_T;
          Eigen::Matrix3d eigen_R;
          Eigen::Vector3d eigen_T;
          cv::cv2eigen(cv_R, eigen_R);
          cv::cv2eigen(cv_T, eigen_T);
          Eigen::Quaterniond Q(eigen_R);
          eigen_R = Q.normalized();
          RIC.push_back(eigen_R);
          TIC.push_back(eigen_T);
          ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
          ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());

      }

      INIT_DEPTH = 5.0;
      BIAS_ACC_THRESHOLD = 0.1;
      BIAS_GYR_THRESHOLD = 0.1;

      TD = fsSettings["td"];
      ESTIMATE_TD = fsSettings["estimate_td"];
      if (ESTIMATE_TD)
          ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
      else
          ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

      ROLLING_SHUTTER = fsSettings["rolling_shutter"];
      if (ROLLING_SHUTTER)
      {
          TR = fsSettings["rolling_shutter_tr"];
          ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
      }
      else
      {
          TR = 0;
      }

      fsSettings.release();
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
