#include <ros/ros.h>
#include <string>
#include <vector>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include "tabletop_object_detector/TabletopDetection.h"
#include "tabletop_object_detector/TabletopSegmentation.h"
#include "tabletop_object_detector/TabletopObjectRecognition.h"

namespace detect_food_container {

class FoodDetector
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceClient seg_srv_;
  ros::ServiceClient rec_srv_;
  ros::ServiceServer object_recognition_srv_;
  
  //! Whether to perform a merge step based on model fit results
  bool perform_fit_merge_;

  bool serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response);

public:
  FoodDetector();
};

FoodDetector::FoodDetector(): nh_(""),priv_nh_("~") {

    std::string service_name;

    rec_srv_ = nh_.serviceClient<TabletopObjectRecognition>(service_name, true);

    object_recognition_srv_ = nh_.advertiseService("object_detection", &FoodDetector::serviceCallback, this);
    ROS_INFO("Food Detector node ready");

    priv_nh_.param<bool>("perform_fit_merge", perform_fit_merge_, true);
}

bool FoodDetector::serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response)
{
    ROS_INFO("Incoming service call");
    //iterate over the clusters and call detection
    for (size_t i = 0; i < request.clusters.size (); ++i)
    {
        ROS_INFO("Processing model %d", i);
        int numModels = request.num_models;
        std::vector<int> scaled_model_ids;
        std::vector<float> confidences;
        std::vector < geometry_msgs::Pose > poses;

        std::cout << "NUM_MODELS:" << numModels << std::endl;
        numModels = 10;
        //transform cluster to sensor_msg/PointCloud2
        sensor_msgs::PointCloud2* pc2 = new sensor_msgs::PointCloud2;
        ROS_INFO("  Original point cloud has %d points in frame %s", request.clusters[i].points.size(),
        request.clusters[i].header.frame_id.c_str());
        sensor_msgs::convertPointCloudToPointCloud2(request.clusters[i], *pc2);
        ROS_INFO("  Converted point cloud has data size %d", (int)pc2->data.size());

        // Call VFH recognizer
        ROS_INFO("  Recognition complete. Found %d models", (int)scaled_model_ids.size());

        //add the models together with pose to the response
        response.models.push_back(pcd2);
        response.cluster_model_indices.push_back(i);
        ROS_INFO("  Model %d complete", (int)i);
    }
    return true;
} 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "food_detector");
  ros::NodeHandle nh;
  detect_food_container::FoodDetector node;
  ros::spin();
  return true;
}
