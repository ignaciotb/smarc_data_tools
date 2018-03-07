#include "scanBuffer.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
// #include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <algorithm>

scanBuffer::scanBuffer(void){

	loadParams();
	for(size_t i = 0; i < bufferSize_; ++i){
		PointCloudT::Ptr cloud_tmp (new PointCloudT());
		cloud_tmp->header.frame_id="buffer";
		clouds_.push_back(cloud_tmp);
		cloudsCount_.push_back(-1);
	}
}

bool scanBuffer::newScan(const PointCloudT::Ptr scan, PointCloudT::Ptr out){
	

	// pcl_ros::transformPointCloud("/odom", *scan, *scan, listener);

	idxMax = 0;
	bool isFull = false;
    for(size_t i = 0; i < clouds_.size(); ++i){
		if( cloudsCount_.at(i) >= 0 || i == 0){ // has been initalized
			*clouds_.at(i) += *scan;
			cloudsCount_.at(i)++;
		} else if( cloudsCount_.at(i-1) >= scanSize_/bufferSize_ ){ // Initialize it
			*clouds_.at(i) += *scan;
			cloudsCount_.at(i)++;
		} else { // Not time to initalize buffer
			break;
		}
		

		// If buffer is full, process the data.
		if(cloudsCount_.at(i) >= scanSize_){
			processBuffer(clouds_.at(i), out);
			clouds_.at(i)->points.clear();
			cloudsCount_.at(i) = 0;
			if(++idxMax >= bufferSize_) idxMax = 0;
			isFull = true;
			out->header.frame_id = "buffer";
			pcl_ros::transformPointCloud("/odom", *out, *out, listener);
		}
	}
	return isFull;
}


void scanBuffer::processBuffer(PointCloudT::Ptr cloud, PointCloudT::Ptr out){

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "buffer";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "lines";
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
	line_list.color.b = 1.0;
	line_list.color.a = 1.0;

	// Cluster the data
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(ecTolerance_); // 2cm
	ec.setMinClusterSize(ecMinClusterSize_);
	ec.setMaxClusterSize(1000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0, idx;
	PointT tmp_max;
	std::vector<pcl::PointIndices>::const_iterator it;
	int i = 0;
	for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		idx = 0;
		tmp_max.intensity = 0.0;
		std::cout << "Intensity: ";
		// Find highest intensity point within each cluster.
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){				
			std::cout << cloud->points[*pit].intensity << "  ";
			if(cloud->points[*pit].intensity > tmp_max.intensity){
				tmp_max = cloud->points[*pit];
			}
			// out->points.push_back(cloud->points[*pit]);
		}
		std::cout << "" << std::endl;
		// if(tmp_max.intensity > maxIntensity_){
		out->points.push_back(tmp_max);
		geometry_msgs::Point p;
		p.x = cloud->points[it->indices.front()].x;
		p.y = cloud->points[it->indices.front()].y;
		p.z = cloud->points[it->indices.front()].z;
		line_list.points.push_back(p);
		p.x = cloud->points[it->indices.back()].x;
		p.y = cloud->points[it->indices.back()].y;
		p.z = cloud->points[it->indices.back()].z;
		line_list.points.push_back(p);
		// }
	}	
	pubLines_->publish(line_list);	
}

void scanBuffer::updateBuffer(Eigen::Affine3f trans){
	for(size_t i = 0; i < clouds_.size(); ++i){
		pcl::transformPointCloud (*clouds_.at(i), *clouds_.at(i), trans);
	}
}

void scanBuffer::loadParams(void){
	scanSize_         = 120;
	bufferSize_       = 10;
	ecTolerance_      = 0.8;
	ecMinClusterSize_ = 3;
	maxIntensity_     = 90;
}