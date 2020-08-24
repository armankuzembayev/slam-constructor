#include <iostream>
#include <ros/ros.h>

#include "image_imu_observer.h"
#include "init_vio.h"
#include "estimator.h"
#include "utility/visualization.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
std::vector<std::string> CAM_NAMES;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
int MAX_CNT;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int EQUALIZE;
int FISHEYE;
int WINDOW_SIZE_CAM;
int STEREO_TRACK;
int FOCAL_LENGTH;
bool PUB_THIS_FRAME;
//pose graph variables
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;
int SKIP_CNT;
int SKIP_DIS;
double CAMERA_VISUAL_SIZE;
std::string POSE_GRAPH_SAVE_PATH;
int LOAD_PREVIOUS_POSE_GRAPH;
std::string VOCABULARY_FILE;
std::string BRIEF_PATTERN_FILE;
camodocal::CameraPtr m_camera;


std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
std::string IMAGE_TOPIC;
double ROW, COL;
double TD, TR;

int image_queue_size = 100;
int imu_queue_size = 2000;
int feature_queue_size = 1000;

int main(int argc, char** argv){
  ros::init(argc, argv, "vio_vins");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  registerPub(n);

  auto estimator = std::make_shared<Estimator>();

  auto vio = std::make_shared<InitVio>(estimator);
  vio->readParameters(n);
  estimator->setParameter();


  auto image_imu_obs = std::make_shared<ImageImuObserver>(n, image_queue_size,
                                                          imu_queue_size,
                                                          feature_queue_size,
                                                          estimator, vio);

  ros::spin();
  return 0;
}
