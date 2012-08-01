#include "config_tracking.h"

std::string TrackingConfig::filteredCloudTopic = "/kinect1/depth_registered/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/rgb/points";

float TrackingConfig::outlierParam = .1;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 1;
float TrackingConfig::kd_cloth = 0;
float TrackingConfig::kp_box = 10;
float TrackingConfig::kd_box = 0.1;
