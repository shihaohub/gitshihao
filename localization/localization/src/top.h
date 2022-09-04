/**
 *top.h
 *brief:top layer of localization pipeline
 *author:Chen Xiaofeng
 *date:20191028
 **/

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <iostream>

#include "common/map_frame.h"
#include "common/wgs84_to_utm.h"
#include "cyber_msgs/GPGGA_MSG.h"
#include "cyber_msgs/Heading.h"
#include "cyber_msgs/JY901.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/VehicleSteerFeedback.h"
#include "ekf/ekf_pose.h"
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/TimeReference.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose2D.h"

#ifndef TOP_H
#define TOP_H

class Top {
 public:
  Top(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  ~Top();

 private:
  bool have_inited_vel_ = false;
  bool have_inited_gps_ = false;
  bool command_no_gps_ = false;
  bool gps_with_problem_ = false;
  bool localization_debug_ = false;
  bool use_steer_ = false;
  int gps_warning_count = 0;
  int gps_error_count = 0;

  double curr_vel_ = 0;
  double curr_yaw_rate_ = 0;
  geometry_msgs::Vector3 angular_velocity_;
  geometry_msgs::Vector3 linear_acceleration_;

  All_EKF_Pose fusion_;

  // parameters for ekf
  double slam_fix_param_ = 0.3;
  double slam_error_param_ = 0.2;
  double imu_err_ = 0.004;       // rad velocity of IMU, unit: rad/s
  double speed_err_ = 0.001;     // unit: m/s
  double gps_err_fix_ = 0.0005;  // from gps, unit: m
  double gps_yaw_err_fix_ = 0.5 * M_PI / 180.0;     // from gps, unit: rad
  double gps_yaw_err_normal_ = 2.0 * M_PI / 180.0;  // from gps, unit: rad
  double slam_err_fix_ = 0.1;                       // slam error, unit: m
  double slam_err_normal_ = 1.0;                    // slam error, unit: m
  double slam_yaw_err_fix_ = 0.5 * M_PI / 180.0;    // from slam, unit: rad
  double slam_yaw_err_normal_ = 5 * M_PI / 180.0;   // from slam, unit: rad
  double car_heading_error = 0.01;
  int err_cnt = 90;

  double t_vel_ = 0.0;
  double t_gps_ = 0.0;
  double t_slam_ = 0.0;
  double t_output_ = 0.0;
  double t_diff_ = 0.0;

  double GLOBAL_ZERO_Longitude_ = 0; //经纬高参数
  double GLOBAL_ZERO_Latitude_ = 0;
  double GLOBAL_ZERO_Altitude_ = 0;

  double GLOBAL_ZERO_X_ = 0;  //UTM参数
  double GLOBAL_ZERO_Y_ = 0; 
  double GLOBAL_ZERO_Z_ = 0;
  int GLOBAL_ZERO_Band_ = 0;
  int GLOBAL_ZERO_Zone_ = 0;

  double EARTH_RAD_EQ_ = 0; //map_frame参数
  double OFFSET_X_ = 0;
  double OFFSET_Y_ = 0;
  double SCALE_ = 0;  
  bool manual_reset = false;

  double global_heading_ = 0;
  double cali_x_ = 0;
  double cali_y_ = 0;

  ros::Timer filter_timer_;

  ros::Publisher pub_localization_estimation_;
  ros::Publisher pub_localization_estimation_angle_;
  ros::Publisher pub_gps_marker_;
  ros::Publisher pub_gps_pose_;
  ros::Publisher pub_slam_marker_;
  ros::Publisher pub_slam_pose_;
  ros::Publisher pub_filter_marker_;
  ros::Publisher pub_filter_pose_;

  ros::Publisher pub_gps_angular_;
  ros::Publisher pub_slam_angular_;
  ros::Publisher pub_pose_diff_;

  ros::Subscriber vel_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber sub_imu_;
  ros::Subscriber command_sub_;

  ros::NodeHandle nh_priv_;
  geometry_msgs::Pose2D diff_pose_;
  int localization_fail_cnt_ = 0;
  int LOCALIZATION_FAIL_NUM_ = 0;
  double TRANS_THRED = 0;
  double ROTA_THRED = 0; 
  bool enable_manual_reset = 0;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, cyber_msgs::Heading, cyber_msgs::GPGGA_MSG> GpsPolicy;
  message_filters::Subscriber<sensor_msgs::NavSatFix> *sub_gps_fix_;
  message_filters::Subscriber<cyber_msgs::Heading> *sub_gps_heading_;
  message_filters::Subscriber<cyber_msgs::GPGGA_MSG> *sub_gps_status_;
  message_filters::Synchronizer<GpsPolicy> *gps_sync_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::NavSatFix, sensor_msgs::Imu>
      SlamPolicy;
  message_filters::Subscriber<sensor_msgs::NavSatFix> *sub_slam_fix_;
  message_filters::Subscriber<sensor_msgs::Imu> *sub_slam_heading_;
  message_filters::Synchronizer<SlamPolicy> *slam_sync_;

  geometry_msgs::PoseArray gps_poses_;
  visualization_msgs::Marker gps_marker_traj_;
  tf::TransformBroadcaster br_gps_;
  geometry_msgs::PoseArray slam_poses_;
  visualization_msgs::Marker slam_marker_traj_;
  tf::TransformBroadcaster br_slam_;
  geometry_msgs::PoseArray filter_poses_;
  visualization_msgs::Marker filter_marker_traj_;
  tf::TransformBroadcaster br_filter_;

  double get_time_now();
  // void command_callback(const std_msgs::Int8ConstPtr &bool_in);
  void vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in);
  void imu_callback(const geometry_msgs::Vector3StampedConstPtr &imu_in);
  void steer_callback(const cyber_msgs::VehicleSteerFeedbackConstPtr &steer_in);
  void gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_in,
                    const cyber_msgs::HeadingConstPtr &heading_in,
                    const cyber_msgs::GPGGA_MSGConstPtr &status_in);
  void slam_callback(const sensor_msgs::NavSatFixConstPtr &slam_in,
                     const sensor_msgs::ImuConstPtr &heading_in);
  void filter_callback(const ros::TimerEvent &);
};  // class Top

#endif
