#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sstream>
#include"img_manip.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"binary_manip");
    ros::NodeHandle nh;

    BinaryImg I("image/from_cam",10,"image/binary",10);

    ros::spin();

    return 0;
}
