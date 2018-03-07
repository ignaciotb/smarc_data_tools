/**
 *	Maintaines a buffer of previous scans.
 *  @author Unnar Axelsson
 */

#ifndef SCANBUFFER_H
#define SCANBUFFER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class scanBuffer{

	std::vector<PointCloudT::Ptr> clouds_;
	tf::TransformListener listener;
	tf::TransformBroadcaster br;
	std::vector<int> cloudsCount_;
	int idxMax;
	tf::TransformListener listener_;
	int bufferSize_, scanSize_, maxIntensity_;
	float ecTolerance_, ecMinClusterSize_;
	ros::Publisher *pubLines_;

public:
	scanBuffer(void);

	bool newScan(const PointCloudT::Ptr scan, PointCloudT::Ptr out);
	void updateBuffer(Eigen::Affine3f trans);

	void setPublisher(ros::Publisher *pubLines){
		pubLines_ = pubLines;
	}

private:
	void processBuffer(PointCloudT::Ptr cloud, PointCloudT::Ptr out);
	void loadParams(void);

};

#endif