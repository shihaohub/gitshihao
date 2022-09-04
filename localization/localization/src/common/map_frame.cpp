/*
 * @Author: your name
 * @Date: 2021-05-10 12:20:32
 * @LastEditTime: 2022-05-01 14:40:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /localization/localization/src/common/map_frame.cpp
 */
/**
  *map_frame.cpp
  *brief:transform between gps and map
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "map_frame.h"

MapFrame::MapFrame(double earth_rad_eq, double offset_x, double offset_y, double scale, const double x_in, const double y_in): x(x_in), y(y_in){
    EARTH_RAD_EQ_ = earth_rad_eq;  // unit: m
    OFFSET_X_ = offset_x;
    OFFSET_Y_ = offset_y;
    SCALE_ = scale;
}

sensor_msgs::NavSatFix MapFrame::MapFrame2GPS(){
    double lat = 360.0 / M_PI * atan(exp((y+OFFSET_Y_)/(SCALE_ * EARTH_RAD_EQ_)))-90.0;
    double lon = 180.0 * (x + OFFSET_X_) / (SCALE_ * EARTH_RAD_EQ_ * M_PI);
    sensor_msgs::NavSatFix gps;
    gps.latitude = lat;
    gps.longitude = lon;
    return gps;
}
void MapFrame::GPS2MapFrame(const sensor_msgs::NavSatFix &gps){
    x = SCALE_ * EARTH_RAD_EQ_ * gps.longitude * M_PI / 180.0 - OFFSET_X_;
    y = SCALE_ * EARTH_RAD_EQ_ * log(tan((90.0 + gps.latitude) * (M_PI / 360.0))) - OFFSET_Y_;
}
double MapFrame::deadReckoning(const double &travel_distance,const double &yaw_before, const double &turn_radian){
    double yaw = yaw_before + turn_radian/2;
    x += travel_distance * cos(yaw);
    y += travel_distance * sin(yaw);
    yaw += turn_radian/2;
    return yaw;
}
double MapFrame::calcDistance(const MapFrame &another){
    return sqrt(pow(x - another.x,2) + pow(y - another.y,2));
}
