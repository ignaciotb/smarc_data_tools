#include "utils.h"

namespace utils{

	bool listenForTransform(std::string from, std::string to, tf::StampedTransform transform){
		
		tf::TransformListener listener;
		try{
			listener.lookupTransform(from, to, ros::Time(0), transform);
			return true;
		}
		catch (tf::TransformException ex){
			std::cout << "exception: " << ex.what() << std::endl;
			return false;
		}
	}

}