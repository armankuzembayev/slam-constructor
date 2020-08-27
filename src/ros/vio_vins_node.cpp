#include <iostream>
#include <ros/ros.h>


#include "image_imu_observer.h"
#include "init_vio.h"
#include "vins_estimator/estimator.h"
#include "vins_estimator/utility/visualization.h"

int image_queue_size = 100;
int imu_queue_size = 2000;
int feature_queue_size = 1000;

int main(int argc, char** argv){
  ros::init(argc, argv, "vio_vins");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  registerPub(n);
  
  auto estimator = std::make_shared<Estimator>();
  readParameters(n);
  estimator->setParameter();

  auto vio = std::make_shared<InitVio>(estimator);
  
  auto image_imu_obs = std::make_shared<ImageImuObserver>(n, image_queue_size,
                                                          imu_queue_size,
                                                          feature_queue_size,
                                                          estimator, vio);

  ros::spin();
  return 0;
}
