#include "scan.hpp"


void scan::cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // 受け取ったメッセージをコピーしておく
  latest_scan = *msg;
}
scan::scan()
{
  ros::NodeHandle nh("~");
  sub_scan = nh.subscribe("/scan", 5,
  &scan::cb_scan, this);
}
float scan::scan_val(int i)
{

  //ROS_INFO("Hello ROS World!");
  int flag =1;
  //ros::Rate rate(10.0);
  if(latest_scan.ranges.size() > 0)
  {
    // LaserScanメッセージをすでに受け取っている場合a

    //int i = 540;//((latest_scan.angle_max)+(-latest_scan.angle_min)) / latest_scan.angle_increment;
    //ROS_INFO("%d",i);
    if(latest_scan.ranges[i] < latest_scan.range_min || // エラー値の場合
    latest_scan.ranges[i] > latest_scan.range_max || // 測定範囲外の場合
    std::isnan(latest_scan.ranges[i])) // 無限遠の場合
    {
    // 正面に十分な距離がある (測定範囲以上)
    //ROS_INFO("front-range: measurement error");
    return 0.0;
    flag = 0;

    }
    else
    {


    //ROS_INFO("front-range: %0.3f", latest_scan.ranges[i]);
    //ROS_INFO("%d",i);
    return latest_scan.ranges[i];
    flag = 0;

    }
  }
}
