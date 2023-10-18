#include<ros/ros.h>
#include<sstream>
#include<cstdlib>
#include<iostream>
#include"remro_vision/rect_dims.h"
#include"remro_vision/motor_idx.h"

void index_set(int argc, char** argv);
void manual_set(int argc, char** argv);

int main(int argc, char** argv)
{
    std::cout << "-1 to end process" << std::endl;
    index_set(argc,argv);
    return 0;
}

void index_set(int argc, char** argv){
    ros::init(argc,argv,"crop_dimensions");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<remro_vision::motor_idx>("rectangle/index");

    remro_vision::motor_idx srv;

    int idx;
    ROS_INFO("input motor index:");

    while(ros::ok())
    {
        std::cin >> idx;
        if(std::cin.fail())
        {
            std::cin.clear();
            ROS_INFO("input expected int\n");
            std::cin.ignore(1000,'\n');
            continue;
        }
        if(idx == -1)
        {
            ROS_INFO("terminating node..\n");
            break;
        }
        srv.request.INDEX = idx;

        if (client.call(srv))
        {
            ROS_INFO("Received response: %s\n",srv.response.RESPONSE.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service\n");
        }
    }
    return;
}

void manual_set(int argc, char** argv){
    ros::init(argc,argv,"crop_dimensions");
    ros::NodeHandle nh;

    //if(argc!=4){
    //    ROS_INFO("usage: process <X int> <Y int> <WIDTH int> <HEIGHT int>\n");
    //    return 1;
    //}

    ros::ServiceClient client = nh.serviceClient<remro_vision::rect_dims>("rectangle/dimensions");

    remro_vision::rect_dims srv;

    int x,y,w,h;
    ROS_INFO("input x y width height:");

    while(ros::ok())
    {
        std::cin >> x >> y >> w >> h;
        if(std::cin.fail())
        {
            std::cin.clear();
            ROS_INFO("input expected int\n");
            std::cin.ignore(1000,'\n');
            continue;
        }

        srv.request.X = x;
        srv.request.Y = y;
        srv.request.WIDTH = w;
        srv.request.HEIGHT = h;

        if (client.call(srv))
        {
            ROS_INFO("Received response: %s\n",srv.response.RESPONSE.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service\n");
        }

    }
    return;
}
