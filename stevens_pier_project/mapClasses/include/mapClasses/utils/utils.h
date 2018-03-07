#ifndef UTILS_H
#define UTILS_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <queue>

namespace utils{

	template<typename PointCloudT>
	void printCloud(PointCloudT a);

	template<typename PointT>
	float Distance(PointT a, PointT b);

	template<typename PointT>
	float Distance(PointT a);
	
	template<typename PointT>
	float squaredDistance(PointT a, PointT b);

	template<typename PointT>
	float squaredDistance(PointT a);

	float dotVec(std::vector<float> a, std::vector<float> b);

	float lenVec(std::vector<float> a);

	template<typename PointT>
	bool isPointPerpendicularToLine(const PointT a, const PointT b, const PointT p, float &theta);

	void calcMeanVar(const std::vector<float> vec, float &ave, float &var);


	class poseTracker{
		std::queue<Eigen::Vector3f> pose;
	public:	
		poseTracker(){ };
		bool newPose(tf::StampedTransform transform);
	};


	namespace params{
		template<typename T>
		T loadParam( std::string name, ros::NodeHandle &nh);

		template<typename T>
		T loadParam( std::string name, ros::NodeHandle *nh);
	}

}

#include "utils.hpp"
#endif