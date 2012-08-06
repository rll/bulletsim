#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <bulletsim_msgs/Initialization.h>

#include "utils_tracking.h"
#include "config_tracking.h"
#include "utils/conversions.h"

/*
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
*/


ColorCloudPtr filteredCloud(new ColorCloud()); // filtered cloud in ground frame
CoordinateTransformer* transformer;
bool pending = false; // new message received, waiting to be processed
tf::TransformListener* listener;


/*
  Call back to display corners of towel found.
*/

void towelCornersCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {

  if (transformer == NULL) {
    transformer = new CoordinateTransformer(waitForAndGetTransform(*listener, 
								   "/ground",cloudMsg->header.frame_id));
  }

  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);

  std::cout<<"In the callback!"<<std::endl;
  pending = true;   

}

int main(int argc, char **argv) {

  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  GeneralConfig::scale = 10;
  parser.read(argc, argv);
  

  ros::init(argc, argv, "towel_corners_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Subscriber cloudSub = nh.subscribe(TrackingConfig::filteredCloudTopic, 1, &towelCornersCallback);
  
  listener = new tf::TransformListener();

  // wait for first message, then initialize
  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) throw std::runtime_error("caught signal while waiting for first message");
    }

  while (ros::ok()) {    
    bulletsim_msgs::Initialization init;

    pcl::toROSMsg(*scaleCloud(filteredCloud,1/METERS), init.request.cloud);
    init.request.cloud.header.frame_id = "/ground";
    bool success = ros::service::call(initializationService, init);
    
    if (success && init.response.objectInit.type == "towel_corners") {
      const std::vector<geometry_msgs::Point32>& points = init.response.objectInit.towel_corners.polygon.points;
      std::vector<btVector3> corners = scaleVecs(toBulletVectors(points),METERS);
      std::cout<<corners[0].getX()<<" "<<corners[0].getY()<<" "<<corners[0].getZ()<<std::endl;
      std::cout<<corners[1].getX()<<" "<<corners[1].getY()<<" "<<corners[1].getZ()<<std::endl;
      std::cout<<corners[2].getX()<<" "<<corners[2].getY()<<" "<<corners[2].getZ()<<std::endl;
      std::cout<<corners[3].getX()<<" "<<corners[3].getY()<<" "<<corners[3].getZ()<<std::endl<<std::endl;
    }
    pending = false;
    while (ros::ok() && !pending) {
      sleep(.01);
      ros::spinOnce();
    }
  }
}

