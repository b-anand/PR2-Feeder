#include <ros/ros.h>
#include <string>
#include <vector>

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
  ros::ServiceServer complete_srv_;
  
  //! Whether to perform a merge step based on model fit results
  bool perform_fit_merge_;

  bool serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response);

public:
  FoodDetector();
};

FoodDetector::FoodDetector(): nh_(""),priv_nh_("~") {

    std::string service_name;

    priv_nh_.param<std::string>("segmentation_srv", service_name, "/tabletop_segmentation");

    while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) {
        ROS_INFO("Waiting for %s service to come up", service_name.c_str());
    }

    if (!nh_.ok()) 
        exit(0);
    seg_srv_ = nh_.serviceClient<TabletopSegmentation>(service_name, true);

    priv_nh_.param<std::string>("recognition_srv", service_name, "/tabletop_object_recognition");
    while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) {
        ROS_INFO("Waiting for %s service to come up", service_name.c_str());
    }
    if (!nh_.ok()) 
        exit(0);

    rec_srv_ = nh_.serviceClient<TabletopObjectRecognition>(service_name, true);

    complete_srv_ = nh_.advertiseService("object_detection", &FoodDetector::serviceCallback, this);
    ROS_INFO("Food Detector node ready");
    priv_nh_.param<bool>("perform_fit_merge", perform_fit_merge_, true);
}

bool FoodDetector::serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response)
{
  tabletop_object_detector::TabletopSegmentation segmentation_srv;
  if (!seg_srv_.call(segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    response.detection.result = response.detection.OTHER_ERROR;
    return true;
  }
  response.detection.result = segmentation_srv.response.result;
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    return true;
  }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
  response.detection.table = segmentation_srv.response.table;
  response.detection.clusters = segmentation_srv.response.clusters;
  if (segmentation_srv.response.clusters.empty() || !request.return_models) return true;

  tabletop_object_detector::TabletopObjectRecognition recognition_srv;
  recognition_srv.request.table = segmentation_srv.response.table;
  recognition_srv.request.clusters = segmentation_srv.response.clusters;
  recognition_srv.request.num_models = request.num_models;
  recognition_srv.request.perform_fit_merge = perform_fit_merge_;
  if (!rec_srv_.call(recognition_srv))
  {
    ROS_ERROR("Call to recognition service failed");
    response.detection.result = response.detection.OTHER_ERROR;
    return true;
  }
  response.detection.models = recognition_srv.response.models;
  response.detection.cluster_model_indices = recognition_srv.response.cluster_model_indices;
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
