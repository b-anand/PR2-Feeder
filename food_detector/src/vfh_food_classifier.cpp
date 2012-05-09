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

    
    bool
      loadFileList (std::vector<vfh_model> &models, const std::string &filename)
    {
      std::ifstream fs;
      fs.open (filename.c_str ());
      if (!fs.is_open () || fs.fail ())
        return (false);

      std::string line;
      while (!fs.eof ())
      {
        getline (fs, line);
        if (line.empty ())
          continue;
        vfh_model m;
        m.first = line;
        models.push_back (m);
      }
      fs.close ();
      return (true);
    }



    double computeThreshold (const flann::Matrix<float> &data, int k, double sigma)
    {
      std::vector<float> distances (k);
      for (int i = 0; i < k; ++i)
        distances[i] = data[0][i];

      double mean, stddev;
      pcl::getMeanStd (distances, mean, stddev);

      // Outlier rejection
      std::vector<float> inliers;
      for (int i = 0; i < k; ++i)
        if ((distances[i] <= (mean + sigma * stddev)) && (distances[i] >= (mean - sigma * stddev)))
          inliers.push_back (distances[i]);

      pcl::getMeanStd (inliers, mean, stddev);

      return (mean);
    }

    VFHClassifier::VFHClassifier(const std::string& dataset_location) :
    		dataset_location_(dataset_location)
    {
    	bf::path dpath = dataset_location;
    	bf::path training_data_h5_file_name = dpath / "training_data.h5";
    	bf::path training_data_list_file_name = dpath/"training_data.list";
    	bf::path index_filename = dpath/"kdtree.idx";

    	// Check if the data has already been saved to disk
    	if (!boost::filesystem::exists (training_data_h5_file_name) ||
    			!boost::filesystem::exists (training_data_list_file_name) ||
    			!boost::filesystem::exists (index_filename) )
    	{
    		print_error ("Could not find training data models files %s, %s or %s!\n",
    				training_data_h5_file_name.string().c_str (), training_data_list_file_name.string().c_str (),
    				index_filename.string().c_str());
    	}
    	else
    	{
    		loadFileList (models_, training_data_list_file_name.string());
    		flann::load_from_file (data_, training_data_h5_file_name.string(), "training_data");
    		print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", (int)data_.rows,
    				training_data_h5_file_name.string().c_str (), training_data_list_file_name.string().c_str ());

    		//flann_set_distance_type ((flann_distance_t)7, 0);
    		index_ =new flann::Index<flann::ChiSquareDistance<float> >( data_,flann::SavedIndexParams (index_filename.string().c_str()));
    		index_->buildIndex ();
    	}

    	knn_ = 1;
    }

    bool VFHClassifier::detection_valid(const Detection& detection)
    {
    	const int THRESH = 25;

        typedef pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> VFHEstimatio;
        typedef pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> NormalEstimation;
        typedef pcl::EuclideanClusterExtraction<pcl::PointXYZ> EuclideanClusterExtraction;

        NormalEstimation n3d;
        VFHEstimatio vfh;
        EuclideanClusterExtraction clusterer;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

    	PointCloud cloud;
    	PointCloud::Ptr object_cloud = boost::make_shared<PointCloud>();
    	PointCloudNormal::Ptr normals = boost::make_shared<PointCloudNormal>();
        pcl::PointCloud<pcl::VFHSignature308> vfh_signature;

    	pcl::fromROSMsg(*point_cloud_, cloud);
        printf("Cloud size: %d\n", (int)cloud.points.size());
        printf("Indices size: %d\n", (int)detection.mask.indices.indices.size());
        copyPointCloud (cloud, detection.mask.indices, *object_cloud);

        clusterer.setClusterTolerance (0.01);
        clusterer.setMinClusterSize (300);
        clusterer.setInputCloud(object_cloud);
        std::vector<pcl::PointIndices> clusters;
        clusterer.extract(clusters);
        if (clusters.size()==0) return false;

    	PointCloud::Ptr object_cloud2 = boost::make_shared<PointCloud>();
    	copyPointCloud (*object_cloud, clusters[0], *object_cloud2);

    	n3d.setKSearch (10);                  // 10 k-neighbors by default
    	pcl::KdTree<pcl::PointXYZ>::Ptr normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    	n3d.setSearchMethod (normals_tree);

        // estimate normals
        n3d.setInputCloud(object_cloud2);
        n3d.compute(*normals);

    	pcl::KdTree<pcl::PointXYZ>::Ptr vfh_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
        vfh.setSearchMethod (vfh_tree);
        vfh.setInputCloud(object_cloud2);
        vfh.setInputNormals(normals);
        vfh.compute(vfh_signature);

        std::string stamp =  boost::lexical_cast<std::string> (point_cloud_->header.stamp.toSec ()) + ".pcd";

    //    pcl::io::savePCDFile("cloud_"+stamp, *object_cloud, true);
    //    pcl::io::savePCDFile("vfh_"+stamp, vfh_signature, false);

        assert(vfh_signature.points.size()==1);
        float* hist = vfh_signature.points[0].histogram;

        // KNN search
        flann::Matrix<float> p = flann::Matrix<float>(hist, 1, 308);
        flann::Matrix<int> indices = flann::Matrix<int>(new int[knn_], 1, knn_);
        flann::Matrix<float> distances = flann::Matrix<float>(new float[knn_], 1, knn_);
        index_->knnSearch (p, indices, distances, knn_, flann::SearchParams (512));

        for (size_t i=0;i<indices.cols;++i) {
            print_info ("    %d - %s (%d) with a distance of: %f\n", i, models_.at (indices[0][i]).first.c_str (), indices[0][i], distances[0][i]);
        }

        std::string vfh_model =  models_[indices[0][0]].first.c_str();
        size_t pos1 = vfh_model.find('/');
        size_t pos2 = vfh_model.find('/',pos1+1);
        std::string vfh_label = vfh_model.substr(pos1+1,pos2-pos1-1);
        while (vfh_label.find(".bag")!=std::string::npos) {
        	vfh_label = vfh_label.substr(0, vfh_label.size()-4);
        }
        printf("VFH label: %s\n", vfh_label.c_str());
        printf("BiGG label: %s\n", detection.label.c_str());

        if (distances[0][0]<THRESH && vfh_label==detection.label) {
        	return true;
        }
        else {
        	return false;
        }
    }

    void VFHClassifier::detect()
    {
    	detections_.detections.clear();
    	for (size_t i=0;i<input_detections_->detections.size();++i) {
    		const Detection &d = input_detections_->detections[i];
    		if (detection_valid(d)) {
    			detections_.detections.push_back(d);
    		}
    	}
    }


//cloud_pos_ptr VFHFoodClassifier::getClosestMatch(cloud_prgb_ptr in_cloud_ptr) {
//    cloud_prgb_ptr cloud (new cloud_prgb);
//    cloud = in_cloud_ptr;
//    // Remove nans
//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
//    // Create the normal estimation class, and pass the input dataset to it
//    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//    ne.setInputCloud (cloud);
//
//    // Create an empty kdtree representation, and pass it to the normal estimation object.
//    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//    ne.setSearchMethod (tree);
//
//    // Output datasets
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//
//    ne.setRadiusSearch (0.03);      //cms
//    ne.compute (*normals);
//    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
//    vfh.setInputCloud (cloud);
//    vfh.setInputNormals (normals);
//    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
//    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    vfh.setSearchMethod (tree);
//    // Output datasets
//    cloud_vfh_ptr vfhs (new cloud_vfh ());
//
//    // Compute the features
//    vfh.compute (*vfhs);
//     
//    }
}
