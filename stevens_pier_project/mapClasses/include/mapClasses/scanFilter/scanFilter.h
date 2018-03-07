#ifndef SCANFILTER_H
#define SCANFILTER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mapClasses/utils/utils.h>
#include <mapclasses/buff.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace Eigen;
using namespace utils::params;

struct scanFilterParams{
	int delay;              // number of sonar beams to ignore in the beginning
	float distLow;          // minimum distance to object
	float distHigh;         // maximum distance to object
	float intensityLow;     // minimum intensity around object
	float intensityHigh;    // minimum intensity of object
	float varLow;           // minimum variance around object
	float varHigh;          // maximum variance around object
	float peakDistance;     // minimum distance between objects
	float pillarRadius;     // radius of pillars
	string inputSonarScan;  // sonar topic straight from sonar
	string outputSonarScan; // output topic
	string outputBuff;		// output buff

	// Initialize numbers as negative and strings as empty, that should never be the input.
	scanFilterParams():delay(-1),distLow(-1),distHigh(-1),intensityLow(-1),
		intensityHigh(-1),varLow(-1),varHigh(-1),peakDistance(-1),pillarRadius(-1),
		inputSonarScan(""), outputSonarScan(""){}
};

class scanFilter{

    ros::NodeHandle* nh_;
		ros::Subscriber sub;
		ros::Publisher pub, pubPro, pubBuff_;
		tf::TransformListener listener;
    scanFilterParams par;
    int in;

public:
	scanFilter(ros::NodeHandle *nh){
		// nh = ros::NodeHandle("~");
		nh_ = nh;
		//load parameters with mapping/parameters/scanFilter.yaml file
		setParameters();

		// Suscribe to (mapping/parameters/scanFilter.yaml) inputSonarScan
		// In this case the topic is : /tritech_micron_node/sonarscan
		// param   1 : each message erase the next one
		// param   scanFilter::subscribeCloud : function callback after each received msg
		sub = nh_->subscribe(par.inputSonarScan, 1, &scanFilter::subscribeCloud, this);

		// Publish to (mapping/parameters/scanFilter.yaml) outputSonarScan
		// In this case the topic is : scanFilter
		// a message of type PointCloud
		// param   1 : buffer size
		pub = nh_->advertise<pcl::PointCloud<PointT> >(par.outputSonarScan, 1);

		// Publish to (mapping/parameters/scanFilter.yaml) outputSonarScan
		// In this case the topic is : buff
		// a message of type mapclasses
		// param   1 : buffer size
		pubBuff_ = nh_->advertise<mapclasses::buff>(par.outputBuff, 1);
		in = 0;
	}

private:

	// Takes in single sonar scan in form of a point cloud, filters it based on
	// distance and intensity and sends the maximum intensity point to a buffer
	// if it satisfies following criteria:
	// 1: Maximum intensity higher than a certain threshold.
	// 2: Has a certain number of high intensity neighboring points.
	void subscribeCloud(const pcl::PCLPointCloud2ConstPtr& inputCloud);

	void projectToPlane(PointCloudT::Ptr cloud);

	void processBeam(PointCloudT::Ptr cloud);

	void calcMeanVar(const std::vector<float> vec, float &ave, float &var);

	void setParameters();
};

#endif
