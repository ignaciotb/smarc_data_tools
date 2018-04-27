
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <dirent.h>

#include <tuple>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct mbes_ping{

    mbes_ping(unsigned int id, double time_stamp,double heading, double heave, double pitch, double roll){
        id_ = id;
        heading_ = heading;
        heave_ = heave;
        pitch_ = pitch;
        roll_ = roll;
        time_stamp_ = time_stamp;
    }

    unsigned int id_;
    double time_stamp_;
    double heading_;
    double heave_;
    double pitch_;
    double roll_;
    std::vector<std::vector<double>> beams;
};

class MBESParser{

public:
    MBESParser(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh) {

//        nh_->param<char>((node_name_ + "/nav_folder"), nav_folder, "/NavUTM");
        track_pub_ = nh_->advertise<visualization_msgs::Marker>((node_name_ + "/rov_track"), 10);
        fulltrack_pub_ = nh_->advertise<visualization_msgs::MarkerArray>((node_name_ + "/rov_fulltrack"), 10);
        map_frame_ = "map";

        // Parse ROV track files
        first_pose_ = true;
        const char* nav_dir = "/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/NavUTM/";
        readNavFilesInDir(nav_dir);

        // Parse MBES pings files
        const char* pings_dir = "/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/Pings/";
        readMBESFilesInDir(pings_dir);
        pcl_pub_ = nh_->advertise<PointCloud> ((node_name_ + "/mbes_pcl"), 100);

        std::cout << "Number of pings: " << mbes_pings_.size() << std::endl;

        // Publish
        unsigned int i = 0;
        unsigned int j = 0;
        PointCloud::Ptr pcl_msg;
//        pubROVFullTrack(1);

        while(ros::ok()){

            // Publish one MBES ping
            ROS_INFO("Publishing the MBES");
            if(i >= mbes_pings_.size()-1){
                i = 0;
            }
            pcl_msg = pubMBESPCL(i);
            i += 2;
            pcl_conversions::toPCL(ros::Time::now(), pcl_msg->header.stamp);
            pcl_pub_.publish (pcl_msg);
            pcl_msg->clear();

            // Publish the ROV track
            ROS_INFO("Publishing the ROV track");
            if(j == rov_coord_.size()-1){
                j = 0;
            }
            pubROVTrack(j);
            j += 1;
            ros::Duration(0.01).sleep();
        }
    }


private:

    PointCloud::Ptr pubMBESPCL(unsigned int i){

        // Check last ping (for debugging)
        PointCloud::Ptr pcl_msg (new PointCloud);
        pcl_msg->header.frame_id = map_frame_;
        pcl_msg->height = 1;
        pcl_msg->width = mbes_pings_.at(i).beams.size() + mbes_pings_.at(i+1).beams.size();

        // One side ping
        std::for_each(mbes_pings_.at(i).beams.begin(), mbes_pings_.at(i).beams.end(), [&pcl_msg](std::vector<double> beam){
            pcl_msg->points.push_back(pcl::PointXYZ(beam.at(0), beam.at(1), beam.at(2)));
        });

        // Other side ping
        std::for_each(mbes_pings_.at(i+1).beams.begin(), mbes_pings_.at(i+1).beams.end(), [&pcl_msg](std::vector<double> beam){
            pcl_msg->points.push_back(pcl::PointXYZ(beam.at(0), beam.at(1), beam.at(2)));
        });

        return pcl_msg;
    }

    void pubROVFullTrack(double color){

        int cnt = 0;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        for(const std::tuple<double, double, double, double> pose_t: rov_coord_){
            marker.header.frame_id = map_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "rov_track";
            marker.id = ++cnt;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = std::get<1>(pose_t);
            marker.pose.position.y = std::get<2>(pose_t);
            marker.pose.position.z = std::get<3>(pose_t);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            marker.color.a = 1.0;
            marker.color.r = color;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }
        fulltrack_pub_.publish(marker_array);
    }

    void pubROVTrack(unsigned int i){

        visualization_msgs::Marker marker;
        std::tuple<double, double, double, double> pose_t = rov_coord_.at(i);

        marker.header.frame_id = map_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rov_track";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = std::get<1>(pose_t);
        marker.pose.position.y = std::get<2>(pose_t);
        marker.pose.position.z = std::get<3>(pose_t);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        track_pub_.publish(marker);
    }


    void readMBESFilesInDir(auto dir_path){
        DIR *dir;
        if ((dir = opendir (dir_path)) != NULL) {
            // Open directory and check all files inside
            std::vector<std::string> files = checkFilesInDir(dir);

            // Sort the files before parsing them (naming == acquisition time)
            std::sort(files.begin(), files.end());

            std::vector<double> new_beam;
            double time_stamp, ping_id, beam_id;
            double x_pose, y_pose, z_pose;
            double time_stamp_origin;
            double heave, heading, pitch, roll;

            unsigned int pose_in_line = 0;
            std::ifstream infile;
            std::string tab_delimiter = "\t";
            std::string line;

            unsigned int beam_num_in_file;
            unsigned int prev_ping_id;
            // For every file in the dir
            for(const std::string file: files){

                // Open file
                infile.open(std::string(dir_path) + file);
                if(infile.is_open()) {
                    ROS_INFO_STREAM(std::string(dir_path) + file << " file open");
                }
                else{
                    ROS_ERROR_STREAM("Could not open file: " << std::string(dir_path) + file);
                    continue;
                }

                // Init state machine
                beam_num_in_file = 0;
                int pings_cnt = 0;

                // Read a new line
                while (std::getline(infile, line)){
                    // Logic for initialization
                    if(beam_num_in_file == 0){
                        // Discard first line with header!!!!
                        beam_num_in_file = 1;
                        continue;
                    }

                    // Parse a line
                    pose_in_line = 0;
                    while(true){
                        if(line.find(tab_delimiter) == -1){
                            // case 5: exit
                            break;
                        }

                        // Extract space-separated numbers: Nav files
                        // 0: Year
                        // 1: Time (day, hour)
                        // 2: Second
                        // 3: Ping num
                        // 4: Beam num
                        // 5: X
                        // 6: Y
                        // 7: Z
                        // 8: Tide
                        // 9: Heading
                        // 10: Heave
                        // 11: Pitch
                        // 12: Roll
                        switch(pose_in_line){
                            case 0:
                                break;
                            case 1:
                                time_stamp = computeTimePing(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 2:
                                time_stamp += std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 3:
                                ping_id = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 4:
                                beam_id = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 5:
                                x_pose = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 6:
                                y_pose = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 7:
                                z_pose = -1 * std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 8:
                                break;
                            case 9:
                                heading = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 10:
                                heave = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 11:
                                pitch = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                            case 12:
                                roll = std::stod(line.substr(0, line.find(tab_delimiter)));
                                break;
                        }
                        line = line.substr(line.find(tab_delimiter) + 1, line.size());
                        pose_in_line += 1;
                    }

                    // Init caches with first beam in file
                    if(beam_num_in_file == 1){
                        beam_num_in_file = 2;
                        prev_ping_id = 0;
                    }

                    // If new beam id found, create a new ping object
                    if(prev_ping_id != ping_id){
//                        if(pings_cnt > 10){
//                            ROS_INFO("1000 pings created, exiting");
//                            break;
//                        }
                        mbes_pings_.emplace_back(ping_id, time_stamp, heading, heave, pitch, roll);
                        prev_ping_id = ping_id;
                        time_stamp_origin = time_stamp;
                        pings_cnt += 1;
                    }

                    // Store new beam in current ping
                    std::vector<double> new_beam;
                    new_beam.push_back(x_pose - easting_origin_);
                    new_beam.push_back(y_pose - northing_origin_);
                    new_beam.push_back(z_pose);
                    mbes_pings_.back().beams.push_back(new_beam);
                }
                infile.close();
            }
        }
        else{
            // Could not open directory
            ROS_ERROR("Could not check directory");
        }
        ROS_INFO_STREAM(node_name_ << ", finished reading MBES files");

    }


    void readNavFilesInDir(auto dir_path){
        DIR *dir;
        if ((dir = opendir (dir_path)) != NULL) {
            // Open directory and check all files inside
            std::vector<std::string> files = checkFilesInDir(dir);

            // Sort the files before parsing them (naming == acquisition time)
            std::sort(files.begin(), files.end());

            double time_stamp, easting, northing, depth;
            double time_stamp_origin, depth_origin;
            unsigned int pose_in_line = 0;
            std::ifstream infile;
            std::string spc_delimiter = " ";
            std::string colon_delimiter = ":";
            std::string line;
            // For every file in the dir
            for(const std::string file: files){

                // Open file
                infile.open(std::string(dir_path) + file);
                if(infile.is_open()) {
                    ROS_INFO_STREAM(std::string(dir_path) + file << " file open");
                }
                else{
                    ROS_ERROR_STREAM("Could not open file: " << std::string(dir_path) + file);
                    continue;
                }

                // Read each line
                while (std::getline(infile, line)){
                    pose_in_line = 0;
                    while(true){
                        if(line.find(spc_delimiter) == -1){
                            // case 5: exit
                            break;
                        }

                        // Extract space-separated numbers: Nav files
                        // 0: Day
                        // 1: Time
                        // 2: Easting
                        // 3: Northing
                        // 4: Depth (given in positive values!)
                        // 5: Zeros
                        switch(pose_in_line){
                            case 0:
                                break;
                            case 1:
                                time_stamp = computeTimeNav(line.substr(0, line.find(spc_delimiter)), colon_delimiter);
                                break;
                            case 2:
                                easting = std::stod(line.substr(0, line.find(spc_delimiter)));
                                break;
                            case 3:
                                northing = std::stod(line.substr(0, line.find(spc_delimiter)));
                                break;
                            case 4:
                                depth = -1 * std::stod(line.substr(0, line.find(spc_delimiter)));
                                break;
                        }

                        line = line.substr(line.find(spc_delimiter) + 1, line.size());
                        pose_in_line += 1;
                    }

                    // Initialize origin with first input
                    if(first_pose_ == true){
                        first_pose_ = false;
                        time_stamp_origin = time_stamp;
                        easting_origin_ = easting;
                        northing_origin_ = northing;
                        depth_origin = 0;
                    }

                    // Store new ROV waypoint
                    rov_coord_.emplace_back(time_stamp - time_stamp_origin,
                                            easting - easting_origin_,
                                            northing - northing_origin_,
                                            depth - depth_origin);
                }
                infile.close();
            }
        }
        else{
            // Could not open directory
            ROS_ERROR("Could not check directory");
        }
        ROS_INFO_STREAM(node_name_ << ", finished reading nav files");
    }


    // HELPER METHODS

    double computeTimeNav(std::string time_str, std::string delimiter){
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
            time_str = time_str.substr(time_str.find(delimiter) + 1, time_str.size());
            time_field_cnt += 1;

            // When the hours/minutes/seconds fields have been parsed, exit
            if(time_field_cnt==3){
                break;
            }
        }

        return time_stamp;
    }


    double computeTimePing(std::string time_str){

        double time_stamp = std::stod(time_str);
        double mins = fmod(time_stamp, 10);   // Remove month and day from time
        double hours = (fmod(time_stamp, 1000) - mins)/100;

        return hours*3600 + mins*60;
    }


    std::vector<std::string> checkFilesInDir(DIR *dir){
        // Check files and directories within directory
        struct dirent *ent;
        std::vector<std::string> files;
        while ((ent = readdir (dir)) != NULL) {
            if( ent->d_type != DT_DIR ){
                // If directory, move on
                files.push_back(std::string(ent->d_name));
            }
        }
        closedir(dir);
        return files;
    }

    std::string node_name_;
    double easting_origin_, northing_origin_;

    ros::NodeHandle *nh_;
    ros::Publisher track_pub_;
    ros::Publisher fulltrack_pub_;
    ros::Publisher pcl_pub_;

    std::string map_frame_;

    bool first_pose_;
    std::vector<std::tuple<double, double, double, double>> rov_coord_;
    std::vector<mbes_ping> mbes_pings_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "mbes_parser");

    ros::NodeHandle nh;
    boost::shared_ptr<MBESParser> mbes_parser(new MBESParser(ros::this_node::getName(), nh));

    ros::waitForShutdown();

    mbes_parser.reset();

    return 0;
}
