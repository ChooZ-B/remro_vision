#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sstream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg,"");

        ROS_INFO("resolution: %dx%d\n",cv_ptr->image.cols,cv_ptr->image.rows);
        cv::imshow("view",cv_ptr->image); //eventually window title based in topic
        cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n",e.what());
    }
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"image_viewer");
    ros::NodeHandle nh;
    cv::namedWindow("view",cv::WINDOW_AUTOSIZE/* | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED*/);

    if(argc < 2)
    {
        ROS_INFO("usage: process <subscriber_topic const char*>\n");
        return 1;
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(argv[1],5,imageCallback);

    ros::spin();
    cv::destroyWindow("view");
    
    return 0;
}
