namespace utils{

	template<typename PointT>
	float Distance(PointT a, PointT b){
		return std::sqrt(squaredDistance(a,b));
	}

	template<typename PointT>
	float Distance(PointT a){
		return std::sqrt(squaredDistance(a));
	}

	template<typename PointT>
	float squaredDistance(PointT a, PointT b){
		float x = a.x-b.x;
		float y = a.y-b.y;
		float z = a.z-b.z;
		return x*x+y*y+z*z;
	}

	template<typename PointT>
	float squaredDistance(PointT a){
		return a.x*a.x + a.y*a.y + a.z*a.z;
	}

	template<typename PointCloudT>
	void printCloud(PointCloudT a){
		std::cout << " " << std::endl;
		std::cout << "number of points: " << a->points.size() << std::endl;
		for(size_t i = 0; i < a->points.size(); ++i){
			std::cout << "x: " << a->points.at(i).x << " y: " << a->points.at(i).y << " z: " << a->points.at(i).z << std::endl;
		}
		std::cout << " " << std::endl;
	}

	template<typename PointT>
	bool isPointPerpendicularToLine(const PointT a, const PointT b, const PointT p, float &theta){
		std::vector<float> ab;
		ab.push_back(a.x-b.x);
		ab.push_back(a.y-b.y);
		std::vector<float> ap;
		ap.push_back(a.x-p.x);
		ap.push_back(a.y-p.y);

		theta = std::acos(dotVec(ab,ap)/(lenVec(ab)*lenVec(ap)));
		return true;
	}


	bool poseTracker::newPose(tf::StampedTransform transform){
		
		Eigen::Vector3f curr(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
		pose.push(curr);
		if(pose.size() < 50){
			return true;
		}

		float squaredNorm = (pose.front() - curr).squaredNorm();
		pose.pop();
		bool tmp = squaredNorm > 0.2;
		std::cout << tmp << "  Max: " << squaredNorm << std::endl;
		return squaredNorm > 0.2;

	}

	
	namespace params{
		
		template<typename T>
		T loadParam( std::string name, ros::NodeHandle &nh){
			T param;
			if (nh.hasParam( name )){
			    nh.getParam( name, param);
			    return param;
			} else {
			    std::cout << "Param " << name << " does not exist." << std::endl;
			    exit(0);
			}
	    }

		template<typename T>
		T loadParam( std::string name, ros::NodeHandle *nh){
			T param;
			if (nh->hasParam( name )){
			    nh->getParam( name, param);
			    return param;
			} else {
			    std::cout << "Param " << name << " does not exist." << std::endl;
			    exit(0);
			}
	    }

	}
}