
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <dirent.h>

#include <tuple>
#include <math.h>
#include <algorithm>

#include <eigen3/Eigen/Core>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct mbes_ping{

    mbes_ping(unsigned int id, double time_stamp, double heading, double heave, double pitch, double roll){
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
        rov_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>((node_name_ + "/rov_pose_t"), 10);
        pcl_pub_ = nh_->advertise<PointCloud> ((node_name_ + "/mbes_pcl"), 100);
        map_frame_ = "map";
        rov_frame_ = "rov_link";

        // Parse ROV track files
        first_pose_ = true;
        const char* nav_dir = "/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/NavUTM/";
        readNavFilesInDir(nav_dir);

        // Parse MBES pings files
        first_orientation_ = true;
        const char* pings_dir = "/home/nacho/catkin_ws/src/smarc-project/smarc_data_tools/mbes_data_parser/Data/Pings/";
        readMBESFilesInDir(pings_dir);

        std::cout << "Number of pings: " << mbes_pings_.size() << std::endl;
        std::cout << "Number of ROV poses: " << rov_coord_.size() << std::endl;

        // Run main loop
        double freq = 100;
        timer_run_ = nh_->createTimer(ros::Duration(1.0 / std::max(freq, 1.0)), &MBESParser::run, this);

        ros::spin();
    }


private:

    void run(const ros::TimerEvent&){

        PointCloud::Ptr pcl_msg;
        PointCloud pcl_msg_rov;
        pcl_msg_rov.header.frame_id = rov_frame_;
        tf::Transform tf_rov_map;

//      pubROVFullTrack(1);

        // Publish one MBES ping
        ROS_INFO("Publishing the MBES");
        if(i_ >= mbes_pings_.size()-1){
            i_ = 0;
        }

        // Extract beams as PCL
        pcl_msg = createMBESPcl(i_);

        // Compute tf from map to ROV pose at t given by current ping
        interpolateROVPose(tf_rov_map);
        i_ += 2;

        // BC ROV pose interpolated
        bcMapROVTF(tf_rov_map);
        ros::Duration(0.01).sleep();

        // Transform PCL from map to ROV pose at t
        pcl_ros::transformPointCloud(rov_frame_, *pcl_msg, pcl_msg_rov, tf_listener_);

        // Publish the MBES ping transformed
        pcl_conversions::toPCL(ros::Time::now(), pcl_msg_rov.header.stamp);
        pcl_pub_.publish (pcl_msg_rov);
        pcl_msg_rov.clear();
        pcl_msg->clear();

        // Publish the ROV track
        pubROVPose(tf_rov_map);

        if(j_ == rov_coord_.size()-1){
            j_ = 0;
        }
        pubROVTrack(j_);
        j_ += 1;
    }


    bool bcMapROVTF(tf::Transform &tf_rov_map){
        tf::StampedTransform tf_odom_map_stp = tf::StampedTransform(tf_rov_map,
                                               ros::Time::now(),
                                               map_frame_,
                                               rov_frame_);

        tf_odom_map_stp.getRotation().normalize();

        geometry_msgs::TransformStamped msg_odom_map;
        tf::transformStampedTFToMsg(tf_odom_map_stp, msg_odom_map);
        map_rov_bc_.sendTransform(msg_odom_map);
        bool broadcasted = true;

        return broadcasted;
    }


    void interpolateROVPose(tf::Transform &tf_rov_map){

        // Look for closest time stamp in ROV track
        double time_t = (mbes_pings_.at(i_).time_stamp_ + mbes_pings_.at(i_ + 1).time_stamp_) / 2;
        double min_t_diff = 10000.0;
        unsigned int rov_pose_id = 0;
        unsigned int cnt = 0;

        // TODO: reduce range of search
        std::for_each(rov_coord_.begin(), rov_coord_.end(), [&cnt, &time_t, &min_t_diff, &rov_pose_id](std::tuple<double, double, double, double> rov_pose){
            if(min_t_diff > std::abs(std::get<0>(rov_pose) - time_t)){
                min_t_diff = std::abs(std::get<0>(rov_pose) - time_t);
                rov_pose_id = cnt;
            }
            cnt += 1;
        });

//        rov_pose_id = cnt;
        std::cout << "Time of ROV pose: " << std::get<0>(rov_coord_.at(rov_pose_id)) << std::endl;
        std::cout << "Time of ping: " << time_t << std::endl;
        std::cout << "ROV pose: " << std::endl;
        std::cout << std::get<1>(rov_coord_.at(rov_pose_id)) << ", " << std::get<2>(rov_coord_.at(rov_pose_id)) << ", " << std::get<3>(rov_coord_.at(rov_pose_id)) << std::endl;

        // Interpolate ROV position
        int next_it = (std::get<0>(rov_coord_.at(rov_pose_id)) > time_t)? -1: 1;

        Eigen::Vector3d rov_pose_t1 = Eigen::Vector3d(std::get<1>(rov_coord_.at(rov_pose_id)),
                                                      std::get<2>(rov_coord_.at(rov_pose_id)),
                                                      std::get<3>(rov_coord_.at(rov_pose_id)));

        Eigen::Vector3d rov_pose_t2 = Eigen::Vector3d(std::get<1>(rov_coord_.at(rov_pose_id + next_it)),
                                                      std::get<2>(rov_coord_.at(rov_pose_id + next_it)),
                                                      std::get<3>(rov_coord_.at(rov_pose_id + next_it)));

        Eigen::Vector3d position = rov_pose_t1 + ((rov_pose_t2 - rov_pose_t1)/(std::get<0>(rov_coord_.at(rov_pose_id + next_it)) - std::get<0>(rov_coord_.at(rov_pose_id)))) *
                                   (time_t - std::get<0>(rov_coord_.at(rov_pose_id)));

        std::cout << "Position interpolated: " << position << std::endl;

        // And orientation
        tf::Matrix3x3 m;
//        m.setRPY(mbes_pings_.at(i_).roll_ * M_PI/180,
//                 mbes_pings_.at(i_).pitch_ * M_PI/180,
//                 mbes_pings_.at(i_).heave_ * M_PI/180);

        m.setRPY(0,0,0);
        tf_rov_map = tf::Transform(m, tf::Vector3(position(0),position(1),position(2)));
    }


    void pubROVPose(tf::Transform tf_map_rov){

        geometry_msgs::PoseStamped rov_pose;
        geometry_msgs::Quaternion q;
        tf::quaternionTFToMsg(tf_map_rov.getRotation(), q);

        rov_pose.header.frame_id = map_frame_;
        rov_pose.header.stamp = ros::Time::now();
        rov_pose.pose.position.x = tf_map_rov.getOrigin().getX();
        rov_pose.pose.position.y = tf_map_rov.getOrigin().getY();
        rov_pose.pose.position.z = tf_map_rov.getOrigin().getZ();
        rov_pose.pose.orientation = q;

        ROS_INFO("Publishing the ROV track");
        rov_pose_pub_.publish(rov_pose);

    }


    PointCloud::Ptr createMBESPcl(unsigned int i){

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

            double time_stamp_origin, roll_origin, pitch_origin, yaw_origin;
            double time_stamp, ping_id, beam_id;
            double x_pose, y_pose, z_pose;
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
//                                std::cout << "Time stamp from hours: "<< time_stamp << std::endl;
                                break;
                            case 2:
                                time_stamp += std::stod(line.substr(0, line.find(tab_delimiter)));
//                                std::cout << "Time stamp from sec: "<< time_stamp << std::endl;
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

                    // Init orientation from first ping in first file
                    if(first_orientation_){
                        first_orientation_ = false;
                        ROS_INFO("Init first orientation!");
                        time_stamp_origin = time_stamp;
                        yaw_origin = heading;
                        roll_origin = roll;
                        pitch_origin = pitch;
                    }

                    // Init caches with first beam in file
                    if(beam_num_in_file == 1){
                        beam_num_in_file = 2;
                        prev_ping_id = 0;
                    }

                    // If new beam id found, create a new ping object
                    if(prev_ping_id != ping_id){
//                        if(mbes_pings_.size() > 10){
//                            ROS_INFO("1000 pings created, exiting");
//                            break;
//                        }
//                        std::cout << "New ping: "<< ping_id << ", " << time_stamp - time_stamp_origin << std::endl;
                        mbes_pings_.emplace_back(ping_id,
                                                 time_stamp - time_stamp_origin,
                                                 heading,
                                                 heave - yaw_origin,
                                                 pitch - pitch_origin,
                                                 roll_origin - roll_origin);
                        prev_ping_id = ping_id;
                    }

                    // Store new beam in current ping
//                    std::cout << "New beam in ping "<< ping_id << ", " << time_stamp - time_stamp_origin << std::endl;
                    std::vector<double> new_beam;
                    new_beam.push_back(x_pose - easting_origin_);
                    new_beam.push_back(y_pose - northing_origin_);
                    new_beam.push_back(z_pose);
                    mbes_pings_.back().beams.push_back(new_beam);

//                    if((time_stamp - time_stamp_origin) == 120.5){
//                        break;
//                    }
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
//                    std::cout << "ROV pose: "<< time_stamp - time_stamp_origin << ", " << easting - easting_origin_ << ", " << northing - northing_origin_ << ", " <<  depth - depth_origin << std::endl;
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
    double easting_origin_, northing_origin_, yaw_origin_;

    ros::NodeHandle *nh_;
    ros::Timer timer_run_;
    ros::Publisher track_pub_;
    ros::Publisher fulltrack_pub_;
    ros::Publisher pcl_pub_;
    ros::Publisher rov_pose_pub_;

    std::string map_frame_;
    std::string rov_frame_;
    tf::TransformBroadcaster map_rov_bc_;
    tf::TransformListener tf_listener_;

    bool first_pose_;
    bool first_orientation_;
    unsigned int i_ = 0;
    unsigned int j_ = 0;

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
