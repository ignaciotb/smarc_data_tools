//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <ros/ros.h>
#include <vector>
  // PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include "utils/utils.h"

// #define inputSonarScan	"/beamFilter"
#define inputSonarScan	"/scanFilter/scanFilter"
#define outputSonarScan	"beamClustering"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace Eigen;
using namespace utils::params;

class beamClustering{
public:
    ros::NodeHandle nh;
private:
	ros::Subscriber sub;
	ros::Publisher pub;
	std::vector<PointCloudT::Ptr> clouds;
	std::vector<int> clouds_count;

	tf::TransformListener listener;

	int bufferSize, scanSize;
	float radiusSearch, radiusMinNeighbours;
	float ecTolerance, ecMinClusterSize;

public:
	beamClustering(){
		nh = ros::NodeHandle("~");
		sub = nh.subscribe(inputSonarScan, 1, &beamClustering::manageBuffer, this);
		pub = nh.advertise<PointCloudT >(outputSonarScan, 1);

		// Parameters		//
		scanSize = loadParam<int>("scanSize", nh);
		bufferSize = loadParam<int>("bufferSize", nh);;
		radiusSearch = loadParam<double>("radiusSearch", nh);
		radiusMinNeighbours = loadParam<double>("radiusMinNeighbours", nh);
		ecTolerance = loadParam<double>("ecTolerance", nh);
		ecMinClusterSize = loadParam<double>("ecMinClusterSize", nh);

		// Initialize vector containing all cloud buffers and vector containing
		// how many scans each vector holds
		for(size_t i = 0; i < bufferSize; ++i){
			PointCloudT::Ptr cloud_tmp (new PointCloudT());
			clouds.push_back(cloud_tmp);
			clouds_count.push_back(-1);
		}
	}

	private:

	// Recieves a single sonarscan in odom frame and places it in buffers.
	void manageBuffer(PointCloudT::Ptr cloud_tr){
		PointCloudT::Ptr cloud (new PointCloudT() );
		//std::cout << "frame_id : " << cloud_tr->header.frame_id << std::endl;
		cloud_tr->header.frame_id = "/sonar";
	    pcl_ros::transformPointCloud("/odom", *cloud_tr, *cloud, listener );

// filing buffer
		for(size_t i = 0; i < clouds.size(); ++i){
			if( clouds_count.at(i) >= 0 || i == 0){ // has been initalized
				*clouds.at(i) += *cloud;
				clouds_count.at(i)++;
			} else if( clouds_count.at(i-1) >= scanSize/bufferSize ){ // Initialize it
				*clouds.at(i) += *cloud;
				clouds_count.at(i)++;
			} else { // Not time to initalize buffer
				break;
			}

			// If buffer is full, process the data.
			if(clouds_count.at(i) >= scanSize){
				processBuffer(clouds.at(i));
				clouds.at(i)->points.clear();
				clouds_count.at(i) = 0;
			}
		}
	}

	// Takes in a buffer of sonarscans, removes outliers
	// and finds clusters and then the highest intensity within each cluster.
	void processBuffer(PointCloudT::Ptr cloud){

		// Start by removing noise/outliers using radius outlier removal.
		PointCloudT::Ptr cloud_fi (new PointCloudT());

		pcl::RadiusOutlierRemoval<PointT> outrem;
	    // // build the filter
	    // outrem.setInputCloud(cloud);
	    // outrem.setRadiusSearch(radiusSearch);
	    // outrem.setMinNeighborsInRadius(radiusMinNeighbours);
	    // // apply filter
	    // outrem.filter (*cloud_fi);

		cloud_fi = cloud;

		// Cluster the remaining data
		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		tree->setInputCloud (cloud_fi);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointT> ec;
		ec.setClusterTolerance (ecTolerance); // 2cm
		ec.setMinClusterSize (ecMinClusterSize);
		ec.setMaxClusterSize (1000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_fi);
		ec.extract (cluster_indices);

		int j = 0, idx;
		PointT tmp_max;
		PointCloudT::Ptr cloud_cm (new PointCloudT());
		std::vector<pcl::PointIndices>::const_iterator it;
		int i = 0;
		for ( it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			idx = 0;
			tmp_max.intensity = 0.0;
			// Find highest intensity point within each cluster.
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
				if(cloud_fi->points[*pit].intensity > tmp_max.intensity){
					tmp_max = cloud_fi->points[*pit];
				}
			}
			cloud_cm->points.push_back(tmp_max);

		}
		cloud_cm->header.frame_id = "odom";
		pub.publish(cloud_cm);
	}


};


int main (int argc, char** argv)
{
	std::cout << "this" << std::endl;
	// Initialize ROS
	ros::init (argc, argv, "beamClustering");

	beamClustering bc;
	// Spin
	ros::spin ();
	return(0);
}
