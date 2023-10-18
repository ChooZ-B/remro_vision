#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sstream>
#include<image_transport/image_transport.h>
#include<cmath>
#include"remro_vision/RotorAngle.h"

#define PI 3.14159265

/**
 * \class AngleDetector
 *
 * \brief Detects angle of object.
 *
 */
class AngleDetector{

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber sub_;
        ros::Publisher pub_;

        /**
         * \brief Checks if Mat is binary
         *
         * \param m an opencv matrix object
         *
         */
        bool isBinary(cv::Mat& m);

    public:
        AngleDetector();
        void callback(const sensor_msgs::ImageConstPtr& msg);

        /**
         * \brief Computes object orientation using the least second moment of the binary image.
         *
         * \param bin a preferrably binary matrix
         *
         * \throws string error message if matrix isn't binary
         *
         * \return angle
         *
         */
        float get_angle(cv::Mat& bin);
};

/*
 *
 * main function
 *
 */
int main(int argc, char** argv)
{
    ros::init(argc,argv,"angle_detection");

    AngleDetector a;
    ros::spin();
    return 0;
}

AngleDetector::AngleDetector() : it_(nh_)
{
    sub_ = it_.subscribe("image/binary",5,&AngleDetector::callback, this);
    pub_ = nh_.advertise<remro_vision::RotorAngle>("motor/angle",1);
}

void AngleDetector::callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    remro_vision::RotorAngle angle_msg;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n",e.what());
    }

    try
    {
        angle_msg.ANGLE = get_angle(cv_ptr->image);
        //for debugging purposes:
        ROS_INFO("%d\n",(int)angle_msg.ANGLE);

        pub_.publish(angle_msg);
    }
    catch(std::string e)
    {
        ROS_INFO("%s\n",e.c_str());
    }

    return;
}

float AngleDetector::get_angle(cv::Mat& bin)
{
    if(!isBinary(bin))
        throw std::string("Image container isn't binary");
    cv::Moments m = moments(bin,true);
    double a = m.mu20;
    double b = m.mu11;
    double c = m.mu02;

    double theta = atan2(b, a-c)/2.0;
    
    // return minus theta since the algorithm is counting
    // the angle from -180° instead of 0
    return static_cast<float>(-theta*180/PI);
}

bool AngleDetector::isBinary(cv::Mat& m)
{
    if(m.channels()>1)
        return false;
    int i,j;
    for(i = 0; i < m.rows; i++)
        for(j = 0; j < m.cols; j++)
            if(!((m.at<uchar>(i,j) == 0) || (m.at<uchar>(i,j) == 255)))
                return false;
    return true;
}
