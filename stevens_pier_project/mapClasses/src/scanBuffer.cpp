#include "mapClasses/scanBuffer/scanBuffer.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
// #include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
// #include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <algorithm>
#include <mapClasses/utils/utils.h>
#include <mapclasses/buff.h>
// #include <ros/package.h>

using namespace utils::params;

scanBuffer::scanBuffer(){}

void scanBuffer::initialize(ros::NodeHandle *nh){
	nh_ = nh;
	 nbrBeamReceived = 0;
	loadParams();


	isFirstPose = true ;

	// Publish to outputLines
	// In this case the topic is : /mapBuilder/lines_update
	// a message of type visualization_msgs::Marker
	pubLines_      = nh_->advertise<visualization_msgs::Marker>(par_.outputLines, 1);

	// Publish to outputBuffUpdate
	// In this case the topic is : /mapBuilder/buff_update
	// a message of type PointCloudT
	pubBuffUpdate_ = nh_->advertise<PointCloudT>(par_.outputBuffUpdate, 1);

	// Publish to outputBuff
	// In this case the topic is : /mapBuilder/buff
	// a message of type mapclasses::buff
	pubBuff_       = nh_->advertise<mapclasses::buff>(par_.outputBuff, 1);


	for(size_t i = 0; i < par_.bufferSize; ++i){
		PointCloudT::Ptr cloud_tmp (new PointCloudT());
		cloud_tmp->header.frame_id="buffer";
		clouds_.push_back(cloud_tmp);
		cloudsCount_.push_back(-1);
	}
	countLastProcessed = 0;

	// ICP 
	poly_trainer.set_kernel(poly_kernel(0.6, 1, 2));
	trainer.set_trainer(poly_trainer);
	string pathPackage = ros::package::getPath("mapclasses");
	deserialize((pathPackage + "/include/svm/df.dat").c_str()) >> df;
	deserialize((pathPackage + "/include/svm/norm.dat").c_str()) >> normalizer;
	deserialize((pathPackage + "/include/svm/df_1_3.dat").c_str()) >> df_1_3;

}


bool scanBuffer::newScan(buff bu, PointCloudT::Ptr out, int &type){

	float dist, min;
	int idx;
	buff tmpBuff;

	
	 	if (nbrBeamReceived >= 360 ){
 			nbrBeamReceived = 0 ;
 		}
 		else {
 			nbrBeamReceived ++ ;
 		}
 	
 	//std::cout << " new scan : " << nbrBeamReceived << "                           : scanBuffer" << std::endl ;

	// find matching, search the smallest distance
 	for(size_t i = 0; i < bu.buffer->points.size(); ++i){

 		idx = -1;
 		min = 1.5;
 		for(size_t j = 0; j < buff_.size(); ++j){
 			dist = utils::Distance(buff_[j].buffer->points.back(), bu.buffer->points[i]);
 			if( dist < min){
 				idx = j;
 				min = dist;
 			}
 		}

 		// if we find a match between new beam and the oldest :
 		// we add the hits within the similar buffer
 		// sinon we ceate an other buffer 
 		if(idx >= 0){ 
			//std::cout << "for i = " << i << " find matching with j= "<< idx << std::endl;
 			buff_[idx].buffer->points.push_back(bu.buffer->points[i]);
			buff_[idx].dist.push_back(bu.dist[i]);
			buff_[idx].countLast = 0;
 		}
 		else{
 			// No matching buffer has been found
 			buff bufftmp;
 			bufftmp.buffer->points.push_back(bu.buffer->points[i]);
 			bufftmp.buffer->header.frame_id = "buffer";
 			bufftmp.dist.push_back(bu.dist[i]);
			// at the end of buff_ : the no matching features
			//std::cout << "for i = " << i << " don't find matching"  <<std::endl;
			//std::cout << " buff_ size, before pushback = "<< buff_.size() << std::endl;
			buff_.push_back(bufftmp);
			//std::cout << " buff_ size, after pushback = " << buff_.size() << std::endl;
 		}
	}
	for(size_t j = 0; j < buff_.size(); ++j){
		buff_[j].countStart++;
		buff_[j].countLast++;
	}
	//std::cout << " buff_ size = " << buff_.size() << "                           : scanBuffer" << std::endl ;
	

	// need to check what parts of the buffer we need to remove
	for(size_t i = buff_.size(); i-- > 0;){
	  std::cout << buff_[i].buffer->points.size() << std::endl;
	 // Too old data
		if(buff_[i].countStart >= 120){
			buff_.erase(buff_.begin() + i);
			 //std::cout << "erasing " << std::endl;
		} else if (buff_[i].countLast > 7 && buff_[i].buffer->points.size() < 3){
			buff_.erase(buff_.begin() + i);
			 //std::cout << "erasing" << std::endl;
		}
	}

	// for each cluster on buff_ 
	// if the cluster have been detected more than 5times and it contain more thant 3 hits 
	// the cluster is added to the tmpbuffer
	for(size_t j = 0; j < buff_.size(); ++j){
		if(buff_[j].countLast > 5 && buff_[j].buffer->points.size() >= 3){ // Probably not going to add a new point to it
			// buff_[j].buffer->header.frame_id = "odom";
			// pubBuff_.publish(buff_[j].buffer);
			// processBuffer(buff_[j], out, type);
			// buff_[j].countStart = 150;
			// std::cout << "hmmmm" << std::endl;
			*tmpBuff.buffer += *buff_[j].buffer;
			tmpBuff.dist.insert(tmpBuff.dist.end(), buff_[j].dist.begin(), buff_[j].dist.end());
		}
	}

	bool isFull = false;
	if(tmpBuff.buffer->points.size() > 0 && countLastProcessed > 8){
		//std::cout << " new scan : taille du beffer tmpBuff        : " << tmpBuff.buffer->points.size()<< " and " << tmpBuff.buffer->header.frame_id << "                 : scanBuffer.cpp " << std::endl ;       
		processBuffer(tmpBuff, out, type);
		if(type > 0) isFull = true;
		out->header.frame_id = "buffer";
		pcl_ros::transformPointCloud("/odom", *out, *out, listener_);

		// ros::Time now = ros::Time::now() ;
		// tf::StampedTransform transform;
		// if (now > ros::Time(1439997923.0) &&  now < ros::Time(1439997930.0)){
		// 	listener_.lookupTransform("/odom", "/body",ros::Time(0), transform);
		 	
		//  	tf::Quaternion q = transform.getRotation();
		// 	tf::Vector3 v = transform.getOrigin();
		 	
		//  	double x =  q.getX() ;
	 //   		double y =  q.getY() ;
	 //   		double z =  q.getZ() ;
	 //   		double w =  q.getW() ;
	 //   		double toDegree = 180 / M_PI;
	 //   		double yaw, pitch, roll;
	 //    	transform.getBasis().getRPY(roll, pitch, yaw);

		//  	ofstream myfile;
		// 	string file = "/home/clarisse/Documents/srcTestOctave/lookupTransform.txt";

		// 	myfile.open(file.c_str(), std::ofstream::out | std::ofstream::app);
		// 	myfile << "\n" <<  "body/odom " <<  v.getX() << " " <<  v.getY() << " " <<  v.getZ() << "\n";
		// 	myfile << x << " " << y << " " << z << " " << w << "\n";;
		// 	myfile << "deg : "<<  roll << " " << pitch<< " " << yaw <<  "\n" ;
		// 	myfile << "rad : "<< roll * toDegree << " " << pitch * toDegree << " " << yaw * toDegree <<  "\n" ;
		// 	myfile << ros::Time(0) << "\n" ;
			
		// 	myfile.close();
		// }

		countLastProcessed = 0;
		tmpBuff.buffer->header.frame_id="buffer";
		pubBuff_.publish(tmpBuff.buffer);
	} else {
		countLastProcessed++;
	}
	return isFull;

}


void scanBuffer::processBuffer(buff bu, PointCloudT::Ptr out, int &type){

	// Check if there are any points in the input cloud
	if(bu.buffer->points.size() < 5){
		std::cout << "Not enough points in buffer....  Nothing to do." << std::endl;
		return;
	}
	std::vector<PointCloudT::Ptr> pvec;
	scanBuffer::findWalls(bu, out, pvec);
	if(out->points.size() > 0){
		std::cout << "Found wall" << std::endl;
		type = 1;
	} else {
		scanBuffer::findPillars(bu, out);
		if(out->points.size() > 0){
			std::cout << "Found pillar" << std::endl;
			type = 2;
		} else {
			std::cout << "Found nothing" << std::endl;
			type = 0;
		}
	}

}


void scanBuffer::findPillars(const buff bu, PointCloudT::Ptr out){
	// Cluster the data
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (bu.buffer);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(par_.ecTolerance); // 2cm
	ec.setMinClusterSize(par_.ecMinClusterSize);
	ec.setMaxClusterSize(1000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(bu.buffer);
	ec.extract(cluster_indices);

	PointCloudT::Ptr tmpcloud (new PointCloudT());
	std::vector<float> tmpdists;


	std::vector<PointT> vPoint;
	std::vector<double> vProb;
	int j = 0, idx;
	PointT tmp_max;
	std::vector<pcl::PointIndices>::const_iterator it;
	int i = 0;
	int cProb = 0;
	for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		tmpcloud->clear();
		idx = 0;
		tmp_max.intensity = 0.0;


		sample_type_short intensities;
		std::vector<double> dists(20);

		for(int k = 0; k < intensities.size(); ++k){
			intensities(k) = 0;
			dists[k] = 0;
		}
		if(it->indices.size() >= intensities.size()){
			float rem = (it->indices.size()-(intensities.size()))/2.0;
			for(long k = 0; k < intensities.size(); ++k){
				intensities(k) = bu.buffer->points[it->indices[k]+std::ceil(rem)].intensity;
				dists[k] = bu.dist[it->indices[k]+std::ceil(rem)];
			}
		} else {
			float pad = (intensities.size()-it->indices.size())/2.0;
			for(long k = 0; k < it->indices.size(); ++k){
				intensities(k+std::ceil(pad)) = bu.buffer->points[it->indices[k]].intensity;
				dists[k+std::ceil(pad)] = bu.dist[it->indices[k]];
			}
		}

		double result = *std::max_element(dists.begin(), dists.end());

		intensities = normalizer(intensities);

		// std::vector<double> dists(bu.dist.size());
		sample_type st;
		for(int k = 0; k < dists.size(); ++k)
		{
			dists[k] = dists[k] / result;
		}

		for(int k = 0; k < 20; ++k)
		{
			st(k)    = intensities(k);
			st(k+20) = dists[k];
		}

// *****  SVM !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  ***** //
		// std::cout << "SVM: " << df(st) << std::endl;
		if(df(st) == 2) continue;
		if(df(st) == 3){
			cProb++;
		}
		// std::cout << "prob: " << df_1_3(st) << std::endl;
		// if(df_1_3(st) < 0.3) continue;


		vProb.push_back(df_1_3(st));
		// std::cout << "st: " << std::endl;
		// for(int k = 0; k < st.size()/2.0; ++k)
		// {
		// 	std::cout << st(k) << ", ";
		// }
		// std::cout << " " << std::endl;
		// for(int k = 0; k < st.size()/2.0; ++k)
		// {
		// 	std::cout << st(k+20) << ", " ;
		// }
		// std::cout << " " << std::endl;



		// std::cout << "Intensity: ";
		// Find highest intensity point within each cluster.
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			// std::cout << bu.buffer->points[*pit].intensity << "  ";
			if(bu.buffer->points[*pit].intensity > tmp_max.intensity){
				tmp_max = bu.buffer->points[*pit];
			}
			tmpcloud->points.push_back(bu.buffer->points[*pit]);
			tmpdists.push_back(bu.dist[*pit]);
			// out->points.push_back(cloud->points[*pit]);
		}
		//std::cout << tmpcloud->header.frame_id << "  tmpcloud "<< tmpcloud->points.size() << "                          : scanBuffer.cpp " << std::endl ;
		// std::cout << "" << std::endl;
		// if(tmp_max.intensity > maxIntensity_){
		vPoint.push_back(tmp_max);
		// out->points.push_back(tmp_max);
		tmpcloud->header.frame_id="buffer";
		mapclasses::buff buf;
		//std::cout << " try to pucblish on buf ; " << tmpcloud->points[1].x << "   ;  "<< tmpcloud->points[1].y <<"                      : scanBuffer" << std::endl;
		pcl::toROSMsg(*tmpcloud, buf.cloud);
		buf.dists = tmpdists;
		pubBuff_.publish(buf);
		pubBuffUpdate_.publish(tmpcloud);
		// geometry_msgs::Point p;
		// p.x = cloud->points[it->indices.front()].x;
		// p.y = cloud->points[it->indices.front()].y;
		// p.z = cloud->points[it->indices.front()].z;
		// line_list.points.push_back(p);
		// p.x = cloud->points[it->indices.back()].x;
		// p.y = cloud->points[it->indices.back()].y;
		// p.z = cloud->points[it->indices.back()].z;
		// line_list.points.push_back(p);
		// }
	}
	// int cProb = 0;
	double lProb = 0.5;
	if(cProb >= 2){
		lProb = 0.3;
	}
	// std::cout << "cprob: " << cProb << ",  lProp: " << lProb << std::endl;
	for(int i = 0; i < vProb.size(); ++i)
	{

		if(vProb[i] > lProb){
			out->points.push_back(vPoint[i]);
		}
	}

	// pubLines_.publish(line_list);
}

void scanBuffer::findWalls(const buff bu, PointCloudT::Ptr out, std::vector<PointCloudT::Ptr> &walls){

	PointCloudT::Ptr cloud_f (new PointCloudT ());
	PointCloudT::Ptr cloud_tmp (new PointCloudT ());
	std::vector<float> tmpdist = bu.dist;

	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.5);
	seg.setMaxIterations (100);

	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_LINE);

	*cloud_tmp = *bu.buffer;
	int firstcount = 0;
	int i=0, nr_points = cloud_tmp->points.size ();
	while (i < 5 && cloud_tmp->points.size() > 0 && cloud_tmp->points.size() > 0.05 * nr_points)
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		PointCloudT::Ptr cloud_line (new PointCloudT ());
		seg.setInputCloud (cloud_tmp);
		seg.segment (*inliers, *coefficients);

		if(inliers->indices.size() < 15){
			++i;
			break;
		}
		std::vector<int> new_indices;
		int count = 0;
		int inlSize = inliers->indices.size();
		float maxDist = inlSize/30.0 * 2.0;
		for(size_t j = 0; j < inlSize-1; ++j){
			if(utils::Distance(cloud_tmp->points[inliers->indices[j]], cloud_tmp->points[inliers->indices[j+1]]) < maxDist){
				count++;
			} else {
				if(count < 15){
					count = 0;
				} else {
					for(size_t k = j-count; k <= j; ++k){
						new_indices.push_back(inliers->indices[k]);
					}
					count = 0;
				}
			}
		}
		if(count >= 15){
			for(size_t k = inlSize-1-count; k <= inlSize-1; ++k){
				new_indices.push_back(inliers->indices[k]);
			}
		}


		if(new_indices.size() >= 15){
			inliers->indices = new_indices;

		    mapclasses::buff buf;

			// Extract the planar inliers from the input cloud
		    pcl::ExtractIndices<PointT> extract;
		    extract.setInputCloud (cloud_tmp);
		    extract.setIndices (inliers);
		    extract.setNegative (false);
		    extract.filter (*cloud_line);

		    // Remove the planar inliers, extract the rest
		    extract.setNegative (true);
		    extract.filter (*cloud_f);
		    cloud_tmp.swap (cloud_f);

		    for(size_t i = inliers->indices.size(); i-- > 0;){
        		buf.dists.push_back(tmpdist[inliers->indices[i]]);
        		tmpdist.erase(tmpdist.begin() + inliers->indices[i]);
    		}

			// proj.setInputCloud (cloud_line);
			// proj.setModelCoefficients (coefficients);
			// proj.filter (*cloud_line);

		    walls.push_back(cloud_line);
		    cloud_line->header.frame_id="buffer";

			pcl::toROSMsg(*cloud_line, buf.cloud);
		    pubBuff_.publish(buf);
		    pubBuffUpdate_.publish(cloud_line);
		    *out += *cloud_line;
		}
		i++;
	}

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "buffer";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "lines";
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.2;
	line_list.color.b = 1.0;
	line_list.color.a = 1.0;

	geometry_msgs::Point p;
	for(size_t i = 0; i < walls.size(); ++i){
		p.x = walls[i]->points.front().x;
		p.y = walls[i]->points.front().y;
		p.z = walls[i]->points.front().z;
		line_list.points.push_back(p);
		p.x = walls[i]->points.back().x;
		p.y = walls[i]->points.back().y;
		p.z = walls[i]->points.back().z;
		line_list.points.push_back(p);
	}
	pubLines_.publish(line_list);
}



void scanBuffer::updateBuffer(Eigen::Affine3f trans){
	for(size_t i = 0; i < clouds_.size(); ++i){
		pcl::transformPointCloud (*clouds_.at(i), *clouds_.at(i), trans);
	}
}

void scanBuffer::loadParams(void){
	par_.scanSize         = loadParam<int>("scanSize", nh_);
	par_.bufferSize       = loadParam<int>("bufferSize", nh_);
	par_.ecTolerance      = loadParam<double>("ecTolerance", nh_);
	par_.ecMinClusterSize = loadParam<double>("ecMinClusterSize", nh_);
	par_.outputLines      = loadParam<string>("outputLines",nh_);
	par_.outputBuffUpdate = loadParam<string>("outputBuffUpdate",nh_);
	par_.outputBuff       = loadParam<string>("outputBuff",nh_);
}
