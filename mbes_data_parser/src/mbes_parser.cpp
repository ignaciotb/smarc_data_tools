
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
            std::string spc_delimiter = " ";
            std::string colon_delimiter = ":";
            std::string line;

            double time_stamp, easting, northing, depth;
            double time_stamp_origin, easting_origin, northing_origin, depth_origin;
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
                        if(line.find(spc_delimiter) == -1){
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
                        std::string time_str;
                        switch(pose_in_line){
                            case 0:
                                break;
                            case 1:
                                time_str = line.substr(0, line.find(spc_delimiter));
                                time_stamp = computeTime(time_str, colon_delimiter);
                                break;
                            case 2:
                                easting = std::stod(line.substr(0, line.find(spc_delimiter)));
                                break;
                            case 3:
                                northing = std::stod(line.substr(0, line.find(spc_delimiter)));
                                break;
                            case 4:
                                depth = std::stod(line.substr(0, line.find(spc_delimiter)));
                                break;
                        }

                        std::cout << std::stod(line.substr(0, line.find(spc_delimiter))) << std::endl;
                        line = line.substr(line.find(spc_delimiter) + 1, line.size());
                        pose_in_line += 1;

                    }
                    if(first_pose_ == true){
                        first_pose_ = false;
                        time_stamp_origin = time_stamp;
                        easting_origin = easting;
                        northing_origin = northing;
                        depth_origin = 0;

                    }
//                    std::cout << "Line content: " << time_stamp << ", " << easting << ", " << northing << ", " << depth << std::endl;
                    rov_coord_.emplace_back(time_stamp - time_stamp_origin,
                                            easting - easting_origin,
                                            northing - northing_origin,
                                            depth - depth_origin);
                }
                infile.close();
            }
            closedir (dir);
        }
        else{
            // Could not open directory
            ROS_ERROR("Could not check directory");
        }
        ROS_INFO_STREAM(node_name_ << ", finished reading files");
    }


    double computeTime(std::string time_str, std::string colon_delimiter){
        int time_field_cnt = 0;
        double time_stamp = 0.0;
        while(true){
            time_str = time_str.substr(0, time_str.size());
            // Extract colon-separated hours/minutes/seconds to compute time in seconds
            // 0: hours
            // 1: minutes
            // 2: seconds
            switch(time_field_cnt){
                case 0:
                    time_stamp += 3600*std::stod(time_str);
                    break;
                case 1:
                    time_stamp += 60*std::stod(time_str);
                    break;
                case 2:
                    time_stamp += std::stod(time_str);
                    break;
            }
            time_str = time_str.substr(time_str.find(colon_delimiter) + 1, time_str.size());
            time_field_cnt += 1;

            // When the hours/minutes/seconds fields have been parsed, exit
            if(time_field_cnt==3){
                break;
            }
        }

        return time_stamp;
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
