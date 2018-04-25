
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <iostream>

class MBESParser{

public:
    MBESParser(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh) {

        std::ifstream infile("/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/NavUTM/170909022939_UserDefined_copy.nav");
        if (infile.is_open()) {
            ROS_INFO_STREAM(node_name_ << ": file open");
        }
        else{
            ROS_ERROR("Could not open file");
            return;
        }

        std::string delimiter = " ";
        std::string line;
        while (std::getline(infile, line)){
            while(true){
                if(line.find(delimiter) == -1){
                    std::cout << line << std::endl;
                    break;
                }
                std::cout << line.substr(0, line.find(delimiter)) << std::endl;
                line = line.substr(line.find(delimiter) + 1, line.size());
            }
        }
        ROS_INFO_STREAM(node_name_ << ", finished");
    }


private:
    std::string node_name_;
    ros::NodeHandle *nh_;
};


int main(int argc, char** argv){
    ros::init(argc, argv, "mbes_parser");

    ros::NodeHandle nh;
    boost::shared_ptr<MBESParser> mbes_parser(new MBESParser(ros::this_node::getName(), nh));

    ros::waitForShutdown();

    if(!ros::ok()){
        mbes_parser.reset();
    }

    return 0;
}
