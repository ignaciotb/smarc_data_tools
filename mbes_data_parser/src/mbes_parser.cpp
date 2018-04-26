
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <dirent.h>

class MBESParser{

public:
    MBESParser(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh) {

        // Open directory and check all files inside
        DIR *dir;
        if ((dir = opendir ("/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/NavUTM")) != NULL) {
            std::ifstream infile;
            std::string delimiter = " ";
            std::string line;

            // Check files and directories within directory
            struct dirent *ent;
            while ((ent = readdir (dir)) != NULL) {
                if( ent->d_type == DT_DIR ){
                    // If directory, move on
                    continue;
                }

                // Open file and clean up line
                infile.open(ent->d_name);
                line.clear();
                if (infile.is_open()) {
                    ROS_INFO_STREAM(node_name_ << ": file open");
                }
                else{
                    ROS_ERROR("Could not open file");
                    continue;
                }

                // Read each line
                while (std::getline(infile, line)){
                    // Extract space-separated numbers
                    while(true){
                        if(line.find(delimiter) == -1){
                            std::cout << line << std::endl;
                            std::cout << std::endl;
                            break;
                        }
                        std::cout << line.substr(0, line.find(delimiter)) << std::endl;
                        line = line.substr(line.find(delimiter) + 1, line.size());
                    }
                }
                infile.close();
            }
            closedir (dir);
        }
        else{
            /* could not open directory */
            ROS_ERROR("Could not check directory");
        }
        ROS_INFO_STREAM(node_name_ << ", finished reading files");
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
