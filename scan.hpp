#ifndef SCAN_HPP
#define SCAN_HPP
#include <sensor_msgs/LaserScan.h>
#include "stdio.h"
#include <ros/ros.h>
#include <math.h>

class scan
{
private:
    ros::Subscriber sub_scan;
    sensor_msgs::LaserScan latest_scan;
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg);
public:
    scan();
    float scan_val(int i);
};
#endif
