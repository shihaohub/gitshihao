## localization_pipeline_e100 E100定位模块接口
- ### 输入
  - /strong/fix sensor_msgs::NavSatFix 从GPS得到的经纬度
  - /strong/heading pickman_msgs::Heading 从GPS得到的航向
  - /strong/raw_data pickman_msgs/GPGGA_MSG 从GPS得到的星数等信息
  - /mti1_msg pickman_msgs::JY901 从惯导得到的角速度
  - /e100/speed_feedback e100_msgs::VehicleSpeedFeedback 从e100底层得到的车辆速度
  - /slam/gps/fix sensor_msgs::NavSatFix 从SLAM得到的经纬度，可缺失
  - /slam/gps/heading sensor_msgs::Imu 从SLAM得到的航向，可缺失
  - /localize_mode std_msgs::Int8 （GPS定位和SLAM定位的切换，可缺失，默认为0，GPS定位.）暂时不用这个切换，有SLAM就用SLAM
- ### 输出
  - /localization/estimation cyber_msgs::LocalizationEstimate 融合后总的输出，时间戳是UTC时间，包括经纬度，UTM坐标系下位姿，速度，角速度，加速度，角加速度
  - /gps_markers visualization_msgs::Marker 原始GPS位置点显示，非定位调试需要可不输出
  - /gps_poses geometry_msgs::PoseArray 原始GPS位姿箭头显示，非定位调试需要可不输出
  - /slam_markers visualization_msgs::Marker 原始SLAM位置点显示，非定位调试需要可不输出
  - /slam_poses geometry_msgs::PoseArray 原始SLAM位姿箭头显示，非定位调试需要可不输出
  - /filter_markers visualization_msgs::Marker 融合后位置点显示，非定位调试需要可不输出
  - /filter_poses geometry_msgs::PoseArray 融合后位姿箭头显示，非定位调试需要可不输出
