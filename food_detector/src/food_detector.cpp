#include <ros/ros.h>
#include <string>
#include <vector>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>

namespace detect_food_container {
using namespace tabletop_object_detector;
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
    bool startMainLoop();
};

FoodDetector::FoodDetector(): nh_(""),priv_nh_("~") 
{
    std::string service_name;
    priv_nh_.param<std::string>("segmentation_srv", service_name, "/tabletop_segmentation");
     while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) 
     {
       ROS_INFO("Waiting for %s service to come up", service_name.c_str());
     }
     if (!nh_.ok()) exit(0);
     seg_srv_ = nh_.serviceClient<TabletopSegmentation>(service_name, true);

     priv_nh_.param<std::string>("detection_srv", service_name, "/object_detector");
     while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) 
     {
       ROS_INFO("Waiting for %s service to come up", service_name.c_str());
     }
     if (!nh_.ok()) exit(0);
     rec_srv_ = nh_.serviceClient<TabletopObjectRecognition>(service_name, true);
     
//	object_recognition_srv_ = nh_.advertiseService("food_detector", &FoodDetector::serviceCallback, this);
 
     ROS_INFO("complete node ready");
     priv_nh_.param<bool>("perform_fit_merge", perform_fit_merge_, true);

     
     startMainLoop();
    
}
bool cluster_segmenter() {
    tabletop_object_detector::TabletopSegmentation segmentation_srv;
    if (!seg_srv_.call(segmentation_srv))
    {
        ROS_ERROR("Call to segmentation service failed");
//      response.detection.result = response.detection.OTHER_ERROR;
        return true;
    }

//    	response.detection.result = segmentation_srv.response.result;
    if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
    {
        ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
        return true;
    }
    ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
    if (segmentation_srv.response.clusters.empty()) return true;

    //iterate over the clusters and call detection
    for (size_t i = 0; i < segmentation_srv.response.clusters.size (); ++i)
    {
        ROS_INFO("Processing Cluster %d", i);
        //transform cluster to sensor_msg/PointCloud2
        sensor_msgs::PointCloud2* pc2 = new sensor_msgs::PointCloud2;
        ROS_INFO("  Original point cloud has %d points in frame %s", segmentation_srv.response.clusters[i].points.size(),
        		segmentation_srv.response.clusters[i].header.frame_id.c_str());
        sensor_msgs::convertPointCloudToPointCloud2(segmentation_srv.response.clusters[i], *pc2);
        ROS_INFO("  Converted point cloud has data size %d", (int)pc2->data.size());
        
//		Call VFH recognizer
//      ROS_INFO("  Recognition complete. Found %d models", (int)scaled_model_ids.size());        
        ROS_INFO("  Model %d complete", (int)i);
    }

}

bool object_recognizer() {
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
//bool FoodDetector::serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response)
bool FoodDetector::startMainLoop()
{
    //cluster_segmenter();
    object_recognizer();
    return true;
} 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "food_detector");
  ros::NodeHandle nh;
  detect_food_container::FoodDetector node;
  ros::spin();
  return true;
}
