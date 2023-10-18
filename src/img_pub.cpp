#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sstream>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "image_publisher");
    ros::NodeHandle nh;


    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image/from_cam",10);

    int video_source = 0;
    if(argv[1] != NULL)
    {
        std::istringstream video_source_param(argv[1]);
        if(!(video_source_param >> video_source)) return 1;
    }

    cv::VideoCapture cap(video_source);
    if(!cap.isOpened())
    {
        ROS_ERROR("ERROR:unable to open video device %d\n",video_source);
        return 1;
    }
    bool w = cap.set(cv::CAP_PROP_FRAME_WIDTH,1280);
    bool h = cap.set(cv::CAP_PROP_FRAME_HEIGHT,720);
    if(!w || !h)
    {
        ROS_ERROR("WARNING: Resolution of 720p not supported\n");
    }


    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(10);
    while(nh.ok())
    {
        cap.read(frame);
        if(frame.empty())
        {
            ROS_ERROR("ERROR: blank frame grabbed\n");
            continue;
        }
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        cv::waitKey(1);

        ros::spinOnce; //unnecessary
        loop_rate.sleep();
    }
    
    return 0;
}
        
        



