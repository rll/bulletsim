#include <cmath>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_chessboard_pose.h"
#include "utils/my_exceptions.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::string inputTopic1;
  static std::string inputTopic2;
  static int imageCalibration;
  static float squareSize;
  static int chessBoardWidth;
  static int chessBoardHeight;
  static int SVDCalibration;
  static float minCorrFraction;
  static int ICPCalibration;
  static int maxIterations;
  static int RANSACIterations;
  static float RANSACOutlierRejectionThreshold;
  static float maxCorrespondenceDistance;
  static float transformationEpsilon;
  static float euclideanFitnessEpsilon;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic1", &inputTopic1, "input topic1"));
    params.push_back(new Parameter<string>("inputTopic2", &inputTopic2, "input topic2"));
    params.push_back(new Parameter<int>("imageCalibration", &imageCalibration, "0 means that image calibration is NOT run"));
    params.push_back(new Parameter<float>("squareSize", &squareSize, "the length (in meters) of the sides of the squares (if imageCalibration!=0)"));
    params.push_back(new Parameter<int>("chessBoardWidth", &chessBoardWidth, "number of inner corners along the width of the chess board (if imageCalibration!=0 or SVDCalibration!=0)"));
    params.push_back(new Parameter<int>("chessBoardHeight", &chessBoardHeight, "number of inner corners along the height of the chess board (if imageCalibration!=0 or SVDCalibration!=0)"));
    params.push_back(new Parameter<int>("SVDCalibration", &SVDCalibration, "0 means that SVD calibration is NOT run. 2 means that chess board corners points are published into outputTopic/corners1 and outputTopic/corners2."));
    params.push_back(new Parameter<float>("minCorrFraction", &minCorrFraction, "minimum number of valid checker board corner correspondences (i.e. minCorrFraction*chessBoardWidth*chessBoardHeight) in order to run SVD calibration (if SVDCalibration!=0)"));
    params.push_back(new Parameter<int>("ICPCalibration", &ICPCalibration, "0 means that ICP calibration is NOT run"));
    params.push_back(new Parameter<int>("maxIterations", &maxIterations, "maximum number of iterations the internal optimization should run for (if ICPCalibration!=0)"));
    params.push_back(new Parameter<int>("RANSACIterations", &RANSACIterations, "the number of iterations RANSAC should run for (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("RANSACOutlierRejectionThreshold", &RANSACOutlierRejectionThreshold, "the inlier distance threshold for the internal RANSAC outlier rejection loop (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("maxCorrespondenceDistance", &maxCorrespondenceDistance, "the maximum distance threshold between two correspondent points in source <-> target (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("transformationEpsilon", &transformationEpsilon, "the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("euclideanFitnessEpsilon", &euclideanFitnessEpsilon, "the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged (if ICPCalibration!=0)"));
  }
};

string LocalConfig::inputTopic1 = "/kinect1/depth_registered/points";
string LocalConfig::inputTopic2 = "/kinect2/depth_registered/points";
int LocalConfig::imageCalibration = 0;
float LocalConfig::squareSize = 0.0272;
int LocalConfig::chessBoardWidth = 6;
int LocalConfig::chessBoardHeight = 7;
int LocalConfig::SVDCalibration = 1;
float LocalConfig::minCorrFraction = 0.6;
int LocalConfig::ICPCalibration = 0;
int LocalConfig::maxIterations = 100;
int LocalConfig::RANSACIterations = 10;
float LocalConfig::RANSACOutlierRejectionThreshold = 0.005;
float LocalConfig::maxCorrespondenceDistance = 100;
float LocalConfig::transformationEpsilon = 0;
float LocalConfig::euclideanFitnessEpsilon = 0.0001;

typedef boost::shared_ptr< ::sensor_msgs::Image const> ImageConstPtr;

bool image_calib_init = false;
bool svd_calib_init = false;
bool icp_calib_init = false;
Matrix4f transform_diff = Matrix4f::Identity();
Matrix4f initial_cb_transform = Matrix4f::Identity();

boost::shared_ptr<tf::TransformBroadcaster> broadcaster1;
boost::shared_ptr<tf::TransformBroadcaster> broadcaster2;
boost::shared_ptr<tf::TransformListener> listener1;
boost::shared_ptr<tf::TransformListener> listener2;

void SVDCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	ColorCloudPtr cloud1_corners = chessBoardCorners(cloud1, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);
	ColorCloudPtr cloud2_corners = chessBoardCorners(cloud2, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);
	int nAllPoints = LocalConfig::chessBoardWidth * LocalConfig::chessBoardHeight;
	if ((cloud1_corners->size() == nAllPoints) &&
		(cloud2_corners->size() == nAllPoints)) {

		ColorCloudPtr cloudref_corners(new ColorCloud());
		for (int i=0; i<LocalConfig::chessBoardHeight; i++) {
			for (int j=(LocalConfig::chessBoardWidth-1); j>=0; j--) {
				ColorPoint pt;
				pt.x = LocalConfig::squareSize * (j - ((float) LocalConfig::chessBoardWidth - 1.0)/2.0);
				pt.y = LocalConfig::squareSize * (i - ((float) LocalConfig::chessBoardHeight - 1.0)/2.0);
				pt.z = 0;
				cloudref_corners->push_back(pt);
			}
		}

		//Filter out the bad points from both point clouds
		vector<int> badPoints;
		for (int i=0; i<cloud1_corners->size(); i++) {
			if (!pointIsFinite(cloud1_corners->at(i)) || !pointIsFinite(cloud2_corners->at(i)))
				badPoints.push_back(i);
		}
		for (int i=(badPoints.size()-1); i>=0; i--) {
			//ROS_INFO("bad points %d", badPoints[i]);
			cloud1_corners->erase(cloud1_corners->begin() + badPoints[i]);
			cloud2_corners->erase(cloud2_corners->begin() + badPoints[i]);
			cloudref_corners->erase(cloudref_corners->begin() + badPoints[i]);
		}

		int minCorr = LocalConfig::minCorrFraction * nAllPoints;
		if ((cloud1_corners->size()>minCorr) && (cloud2_corners->size()>minCorr)) {
			vector<int> indices;
			for (int i=0; i<cloud1_corners->size(); i++)
				indices.push_back(i);
			pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
			estimation_svd.estimateRigidTransformation(*cloud1_corners, indices, *cloudref_corners, indices, initial_cb_transform);
			estimation_svd.estimateRigidTransformation(*cloud2_corners, indices, *cloud1_corners, indices, transform_diff);
			svd_calib_init = true;
			ROS_INFO("SVD calibration succeeded with %d/%d points", (int) cloud1_corners->size(), nAllPoints);
			cout << "first  transform " << endl << initial_cb_transform << endl;
			cout << "second transform " << endl << transform_diff << endl;
		} else {
			ROS_WARN("SVD calibration failed. Only %d of a minimum of %d correspondences found.", (int) cloud1_corners->size(), minCorr);
		}
	} else {
		ROS_WARN("SVD calibration failed. Make sure both cameras sees the chess board. First camera sees %d/%d. Second camera sees %d/%d.", (int) cloud1_corners->size(),
																																			nAllPoints,
																																			(int) cloud2_corners->size(),
																																			nAllPoints);
	}
}

void ICPCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	ROS_WARN("ICPCalibration probably doesn't work");
	pcl::IterativeClosestPoint<ColorPoint, ColorPoint> icp;
	icp.setInputCloud(downsampleCloud(cloud2, 0.2));
	icp.setInputTarget(downsampleCloud(cloud1, 0.2));
	icp.setMaximumIterations(LocalConfig::maxIterations);
	icp.setRANSACIterations(LocalConfig::RANSACIterations);
	icp.setRANSACOutlierRejectionThreshold(LocalConfig::RANSACOutlierRejectionThreshold);
	icp.setMaxCorrespondenceDistance(LocalConfig::maxCorrespondenceDistance);
	icp.setTransformationEpsilon(LocalConfig::transformationEpsilon);
	icp.setEuclideanFitnessEpsilon(LocalConfig::euclideanFitnessEpsilon);
	ROS_INFO("ICP aligning of the two clouds...");
	ColorCloud Final;
	icp.align(Final, transform_diff);
	if (icp.hasConverged()) {
		transform_diff = icp.getFinalTransformation();
		icp_calib_init = true;
		ROS_INFO("ICP calibration succeeded with fitness score of %.4f", icp.getFitnessScore());
	} else {
		ROS_WARN("ICP calibration failed with fitness score of %.4f", icp.getFitnessScore());
	}
}

void imageCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	ROS_WARN("imageCalibration probably doesn't work");
	cv::Mat image1 = toCVMatImage(cloud1);
	cv::Mat image2 = toCVMatImage(cloud2);

	double CX = 320-.5;
	double CY = 240-.5;
	double F = 525;
	Matrix3f cam_matrix(Matrix3f::Identity());
	cam_matrix(0,0) = cam_matrix(1,1) = F;
	cam_matrix(0,2) = CX;
	cam_matrix(1,2) = CY;

	Matrix4f transform1, transform2;
	if (get_chessboard_pose(image1, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, cam_matrix, transform1) &&
		get_chessboard_pose(image2, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, cam_matrix, transform2)) {
		transform_diff = transform1*transform2.inverse();
		image_calib_init = true;
		ROS_INFO("Image calibration suceeded");
	} else {
		ROS_WARN("Image calibration failed. Make sure both cameras sees the chess board.");
	}
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg_in1, const sensor_msgs::PointCloud2ConstPtr& msg_in2) {
	ColorCloudPtr cloud_in1(new ColorCloud());
	ColorCloudPtr cloud_in2(new ColorCloud());
	pcl::fromROSMsg(*msg_in1, *cloud_in1);
	pcl::fromROSMsg(*msg_in2, *cloud_in2);

	if (LocalConfig::imageCalibration && !image_calib_init)
		imageCalibration(cloud_in1, cloud_in2);
	if (LocalConfig::SVDCalibration && !svd_calib_init)
		SVDCalibration(cloud_in1, cloud_in2);
	if (LocalConfig::ICPCalibration && !icp_calib_init)
		ICPCalibration(cloud_in1, cloud_in2);

	broadcastKinectTransform(toBulletTransform((Eigen::Affine3f) initial_cb_transform), msg_in1->header.frame_id, "ground", *broadcaster1, *listener1);
	broadcastKinectTransform(toBulletTransform((Eigen::Affine3f) (initial_cb_transform*transform_diff)), msg_in2->header.frame_id, "ground", *broadcaster2, *listener2);
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"merger");
	ros::NodeHandle nh;

	broadcaster1.reset(new tf::TransformBroadcaster());
	broadcaster2.reset(new tf::TransformBroadcaster());
	listener1.reset(new tf::TransformListener());
	listener2.reset(new tf::TransformListener());

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1Sub(nh, LocalConfig::inputTopic1, 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2Sub(nh, LocalConfig::inputTopic2, 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;
	message_filters::Synchronizer<CloudSyncPolicy> cloudSync(CloudSyncPolicy(30), cloud1Sub, cloud2Sub);
	cloudSync.registerCallback(boost::bind(&callback,_1,_2));

	ros::spin();
}