#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/features/vfh.h>
#include "pcl/io/pcd_io.h" 
//#include "pcl/visualization/cloud_viewer.h"
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <tf/transform_listener.h>
#include <vector>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> cloud_pos;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_prgb;
typedef pcl::PointCloud<pcl::VFHSignature308> cloud_vfh;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pos_ptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prgb_ptr;
typedef pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr;

namespace vfh_food_classifier
{
  class VFHFoodClassifier
  {
  private:
   

  public:
    VFHFoodClassifier();
    ~VFHFoodClassifier ()
    {
    }
    cloud_pos_ptr getClosestMatch(cloud_prgb_ptr);
  };

cloud_pos_ptr VFHFoodClassifier::getClosestMatch(cloud_prgb_ptr in_cloud_ptr) {
    cloud_prgb_ptr cloud (new cloud_prgb);
    cloud = in_cloud_ptr;
    // Remove nans
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    ne.setRadiusSearch (0.03);      //cms
    ne.compute (*normals);
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    vfh.setSearchMethod (tree);
    // Output datasets
    cloud_vfh_ptr vfhs (new cloud_vfh ());

    // Compute the features
    vfh.compute (*vfhs);
     
    }
}
