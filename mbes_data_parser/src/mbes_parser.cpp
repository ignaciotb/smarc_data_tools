
#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <dirent.h>

#include <tuple>

class MBESParser{

public:
    MBESParser(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh) {

        char nav_folder;
//        nh_->param<char>((node_name_ + "/nav_folder"), nav_folder, "/NavUTM");

        first_pose_ = true;
        const char* dir_name = "/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/NavUTM/";
        readNavFilesInDir(dir_name);

    }


private:

    void readNavFilesInDir(auto dir_path){
        // Open directory and check all files inside
        DIR *dir;
        if ((dir = opendir (dir_path)) != NULL) {
            std::ifstream infile;
            std::string delimiter = " ";
            std::string line;

            double time_stamp, easting, northing, depth;
            unsigned int pose_in_line = 0;

            // Check files and directories within directory
            struct dirent *ent;
            while ((ent = readdir (dir)) != NULL) {
                if( ent->d_type == DT_DIR ){
                    // If directory, move on
                    continue;
                }

                // Open file and clean up line
                char buffer[256]; // <- danger, only storage for 256 characters.
                strncpy(buffer, dir_path, sizeof(buffer));
                strncat(buffer, ent->d_name, sizeof(buffer));

                std::cout << buffer << std::endl;
                infile.open(buffer);
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
                    if(first_pose_ == true){
                        first_pose_ = false;
                    }
                    pose_in_line = 0;
                    while(true){
                        if(line.find(delimiter) == -1){
                            // case 5:
                            std::cout << std::endl;
                            break;
                        }

                        // Extract space-separated numbers: Nav files
                        // 0: Day
                        // 1: Time
                        // 2: Easting
                        // 3: Northing
                        // 4: Depth (positive values!)
                        // 5: Zeros
                        switch(pose_in_line){
                            case 0:
                                break;
                            case 1:
                                time_stamp = std::stod(line.substr(0, line.find(delimiter)));
                                break;
                            case 2:
                                easting = std::stod(line.substr(0, line.find(delimiter)));
                                break;
                            case 3:
                                northing = std::stod(line.substr(0, line.find(delimiter)));
                                break;
                            case 4:
                                depth = std::stod(line.substr(0, line.find(delimiter)));
                                break;
                        }

                        std::cout << std::stod(line.substr(0, line.find(delimiter))) << std::endl;
                        line = line.substr(line.find(delimiter) + 1, line.size());
                        pose_in_line += 1;

                    }
                    std::cout << "Line content: " << time_stamp << ", " << easting << ", " << northing << ", " << depth << std::endl;
                    rov_coord_.emplace_back(time_stamp, easting, northing, depth);
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


    bool first_pose_;
    std::string node_name_;
    ros::NodeHandle *nh_;
    std::vector<std::tuple<double, double, double, double>> rov_coord_;

};


int main(int argc, char** argv){
    ros::init(argc, argv, "mbes_parser");

    ros::NodeHandle nh;
    boost::shared_ptr<MBESParser> mbes_parser(new MBESParser(ros::this_node::getName(), nh));

//    ros::waitForShutdown();

//    if(!ros::ok()){
        mbes_parser.reset();
//    }

    return 0;
}
