#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <stdio.h>
#include <time.h>
#include <boost/thread/thread.hpp>




boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	   std::string cloud1_name="cloud1",
	   std::string cloud2_name="cloud2") {
 
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
    viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);

  viewer->addPointCloud<pcl::PointXYZ>(cloud1, cloud1_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud1_name);

  viewer->addPointCloud<pcl::PointXYZ>(cloud2, cloud2_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud2_name);

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}



int main (int argc, char** argv) {

  std::string real = "cloud_data1804289383.pcd";
  std::string sim  = "sim_cloud_init.pcd";//"sim_cloud17146.pcd";
 

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_sim(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_real(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (sim, *cloud_sim) == -1) {
    PCL_ERROR ("Couldn't read SIM cloud. \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (real, *cloud_real) == -1) {
    PCL_ERROR ("Couldn't read REAL cloud. \n");
    return (-1);
  }

  clock_t startTime = clock();

  Eigen::Matrix4f guess;
  guess<<  1,   0,   0,  -0.45,
	   0,  -1,   0,   0.0,
	   0,   0,  -1,   2.0,
	   0,   0,   0,   1.0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud(*cloud_sim, *cloud_transformed, guess);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1
    = simpleVis(cloud_real, cloud_transformed);


  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_transformed);
  icp.setInputTarget(cloud_real);
  icp.setMaxCorrespondenceDistance (0.1);
  pcl::PointCloud<pcl::PointXYZ> cloud_final;
  icp.align(cloud_final);


  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  std::cout << double( clock() - startTime ) / (double)CLOCKS_PER_SEC
	    << " seconds." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr final(&cloud_final);  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
  viewer2 = simpleVis(cloud_real, final);
  while (!viewer2->wasStopped ()) {
    viewer1->spinOnce(100);
    viewer2->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
