//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <vector>
  // PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <mapClasses/scanFilter/scanFilter.h>


// Takes in single sonar scan in form of a point cloud, filters it based on
// distance and intensity and sends the maximum intensity point to a buffer
// if it satisfies following criteria:
// 1: Maximum intensity higher than a certain threshold.
// 2: Has a certain number of high intensity neighboring points.
//
// void scanFilter::subscribeCloud(PointCloudT::Ptr cloud)
void scanFilter::subscribeCloud(const pcl::PCLPointCloud2ConstPtr& inputCloud)
{
	// we ignore the first sonar scans
	if(in < par.delay){
		in++;
		return;
	}
	// point cloud varibles

	PointCloudT::Ptr cloud (new PointCloudT()); // each point in a cloud is an intensity on one beam at the distance (x,y) of the sonar 


	// // convert to pcl::PointCloud
	pcl::fromPCLPointCloud2(*inputCloud,*cloud);

	// pub.publish(cloud);
	// std::cout << "hmmmmm" << std::endl;
	// std::cout << "topic : /tritech_micron_node/sonarscan : scanFilter" << cloud->points.size() << std::endl; //797
	processBeam(cloud);
	// projectToPlane(cloud);
}

void scanFilter::projectToPlane(PointCloudT::Ptr cloud){
	for(size_t i = 0; i < cloud->points.size(); ++i){
		cloud->points.at(i).z = 0;
	}
	processBeam(cloud);
}

void scanFilter::processBeam(PointCloudT::Ptr cloud){
	//std::cout << " buffer " << cloud->points.size();

// param cloud_pu : will contain the local maximums selected
	PointCloudT::Ptr cloud_pu (new PointCloudT());
	cloud_pu->header.frame_id = cloud->header.frame_id; // Sonar

	float 	dist;
	bool	withinRange = false;
	int 	maxIdx = -1;
	std::vector<int> vmaxIdx;


	// std::cout << "dist 1: " << utils::Distance(cloud->points[1])-utils::Distance(cloud->points[0]) << std::endl;
	// std::cout << "dist 2: " << utils::Distance(cloud->points[2])-utils::Distance(cloud->points[1]) << std::endl;
	// std::cout << "dist 3: " << utils::Distance(cloud->points[3])-utils::Distance(cloud->points[2]) << std::endl;
	// std::cout << "dist 4: " << utils::Distance(cloud->points[4])-utils::Distance(cloud->points[3]) << std::endl;
	// std::cout << "dist 5: " << utils::Distance(cloud->points[5])-utils::Distance(cloud->points[4]) << std::endl;
	// std::cout << "dist 2: " << utils::squaredDistance(cloud->points[1]) << std::endl;
	// std::cout << "dist 3: " << utils::squaredDistance(cloud->points[2]) << std::endl;
	// std::cout << "dist 4: " << utils::squaredDistance(cloud->points[4]) << std::endl;
	// std::cout << "Intesity: ";
	// for(int i = 0; i < cloud->points.size(); ++i)
	// {
	// 	std::cout << cloud->points[i].intensity << ", ";
	// }
	// std::cout << " " << std::endl;

	// first check if there is interference with the usbl system in this beam
	// Simple check if more than a certain number of points in a row is above a threshold discard beam
	int interference = 0;
	for(size_t i = 0; i < cloud->points.size(); ++i){
		if(cloud->points[i].intensity > 130){
			interference++;
			// return if we have found enough high intensity points, discard beam.
			if(interference > 20) return;
		} else {
			interference = 0;
		}
	}

	// Walk through points
	// extract region of interest and extract the best local maximums
	//std::cout << "extract region of interest" << std::endl;
	int outOfRange = 0;
	for(size_t i = 0; i < cloud->points.size(); ++i){
		// Check if point is within range
		dist = utils::squaredDistance(cloud->points.at(i));
		if(dist >= par.distLow && dist <= par.distHigh){
			// Check if Point is above threshold range.
			if(cloud->points.at(i).intensity > par.intensityHigh){
				// Check to see if we were within a high intensity area.
				if(outOfRange > 0) outOfRange=0;
				if(withinRange){
					if(cloud->points.at(i).intensity > cloud->points.at(maxIdx).intensity){
						maxIdx = i;
					}
				} else { // Just entered a high intensity area.
					maxIdx = i;
					withinRange = true;
				}
			} else if (withinRange && outOfRange < 5){
				outOfRange++;

			} else if (withinRange){ // Exited a high intensity area.
				withinRange = false;
				outOfRange = 0;
				// Check that the current max is far enough from the last max.
				if(vmaxIdx.size() > 0){
					// Far enough away from the last peak
					if(utils::squaredDistance(cloud->points.at(vmaxIdx.back()), cloud->points.at(maxIdx)) > par.peakDistance){
						vmaxIdx.push_back(maxIdx);
					} 
					// Check if we should replace last peak.
					else if(cloud->points.at(maxIdx).intensity > cloud->points.at(vmaxIdx.back()).intensity) {
						vmaxIdx.back() = maxIdx;
					}
				} else {
					vmaxIdx.push_back(maxIdx);
				}

			}

		}
	}
	// if(vmaxIdx.size() > 0){
	// 	std::cout << "count: " << vmaxIdx.size() << std::endl;
	// }

	// Process each extracted maximum based on neighbouring beam points.
	int idx;
	int count;
	float mean=0.0f, var=0.0f;
	std::vector<float> meanVar;
	for(size_t j = 0; j < vmaxIdx.size(); ++j){
		count = 0;
		idx = vmaxIdx.at(j);
		meanVar.clear();
	
		for(size_t i = idx - 10; i <= idx + 10; ++i){
			if(i < 0 || i >= cloud->points.size()) continue;
			meanVar.push_back(cloud->points.at(i).intensity);

			if(cloud->points.at(i).intensity > par.intensityLow){
				count++;
			}
		}
		if(count > meanVar.size()/4){
		// if(count > 5){
			calcMeanVar(meanVar, mean, var);
			// std::cout << "mean: " << mean << ", var: " << var << std::endl;
			if(var > par.varLow && var < par.varHigh){
				cloud_pu->points.push_back(cloud->points.at(vmaxIdx.at(j)));
				// std::cout << "idx: " << vmaxIdx.at(j) << ", mean: " << mean << ", var: " << var << ", ";
				// std::cout << "intensity: ";
				// for(size_t i = idx - 10; i <= idx + 10; ++i){
				// 	if(i < 0 || i >= cloud->points.size()) continue;
				// 	// std::cout << cloud->points.at(i).intensity << ", ";
				// }
				// std::cout << "" << std::endl;
			}
		}


	}


	// std::cout << "" << std::endl;
	// std::cout << " _ " << cloud_pu->header.frame_id << std::endl ;
	// Extend found points avail from robot to estimate middle of pillar.
	mapclasses::buff bu;
	float x,y,l,theta;
	for(size_t i = 0; i < cloud_pu->points.size(); ++i){
		x = cloud_pu->points.at(i).x;
		y = cloud_pu->points.at(i).y;
		theta = atan2(y,x);
		l = sqrt(x*x+y*y);
		bu.dists.push_back(l);
		cloud_pu->points.at(i).x = cos(theta)*(l+par.pillarRadius);
		cloud_pu->points.at(i).y = sin(theta)*(l+par.pillarRadius);
	}









	// Time in wicht the transformation ( from buffer(parent) to sonar(child) ) is performed
	ros::Time now =  ros::Time::now(); 
	// Cloud transformation from buffer to sonar : allow to transform a point expressed in sonar frame into buffer frame
	pcl_ros::transformPointCloud("/buffer", *cloud_pu, *cloud_pu, listener);
	cloud_pu->header.frame_id="buffer";
	for(size_t i = 0; i < cloud_pu->points.size(); ++i){
		cloud_pu->points.at(i).z = 0;
	}

	//std::cout << "HEURE " << now.toSec() << " " <<  ros::Time(0).toSec() << "        : scanFilter "<<std::endl ;
	// from odom(parent) to body(child) : allow to transform a point expressed in body frame into odom frame
	tf::StampedTransform tfOdomtoBody;  
	// tranformation at a precise time 
	listener.lookupTransform("/odom", "/body", ros::Time(0), tfOdomtoBody);


	double toDegree = 180 / M_PI;
	tf::Quaternion q = tfOdomtoBody.getRotation();
	tf::Vector3 v = tfOdomtoBody.getOrigin();
	double yaw, pitch, roll;
	// euler angles in radian 
	tfOdomtoBody.getBasis().getRPY(roll, pitch, yaw);

	// define a new PointT for the 2d pose where the landmark have been seen
	// X,Y,Z -> X,Y, yaw

	PointT poseXYyaw ; 
	poseXYyaw.x = v.getX(); 
	poseXYyaw.y = v.getY();
	poseXYyaw.z =  yaw /* * toDegree */ ; // yaw in radian !!!

	//std::cout << " size befor :" << poseXYyaw.z << "             : scanFilter "<<std::endl;
	cloud_pu->points.push_back(poseXYyaw);
	//std::cout << " size after :" <<cloud_pu->points.back().z << "             : scanFilter "<<std::endl;

	
	// convert cloud_pu to bu.cloud, resutat in bu.cloud
	// bu.cloud = cloud_pu;
	pcl::toROSMsg(*cloud_pu, bu.cloud);

	// Publish in topic pub : /scanFilterMA/scanFilter
	pub.publish(cloud_pu);
	 //std::cout << "cloud_pu : scanFilter" << cloud_pu->points.size() << std::endl; //797
	// Publish in topic pubBuff_ : /scanFilterMA/buff
	pubBuff_.publish(bu);

}


void scanFilter::calcMeanVar(const std::vector<float> vec, float &ave, float &var){

	float cnt = 0.0f;
	float sum = 0.0f;
	float cnt2= 0.0f;

	for(size_t i=0; i < vec.size(); ++i){
		cnt  += vec.at(i);
		sum  += i*vec.at(i);
		cnt2 += i*i*vec.at(i);
	}
	ave = sum/cnt;
	var = cnt2/cnt - ave*ave;
}


void scanFilter::setParameters(){

	par.delay           = loadParam<int>("delay", nh_);
	par.distLow         = pow(loadParam<double>("distLow",nh_), 2);
	par.distHigh        = pow(loadParam<double>("distHigh",nh_), 2);
	par.intensityLow    = loadParam<double>("intensityLow", nh_);
	par.intensityHigh   = loadParam<double>("intensityHigh", nh_);
	par.varLow          = loadParam<double>("varLow", nh_);
	par.varHigh         = loadParam<double>("varHigh", nh_);
	par.peakDistance    = loadParam<double>("peakDistance", nh_);
	par.pillarRadius    = 0.2 /* loadParam<double>("pillarRadius", nh_)*/;
	par.inputSonarScan  = loadParam<string>("inputSonarScan", nh_);
	par.outputSonarScan = loadParam<string>("outputSonarScan", nh_);
	par.outputBuff      = loadParam<string>("outputBuff", nh_);

}
