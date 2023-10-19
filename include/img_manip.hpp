/**
 *
 * \file img_manip.hpp
 *
 * \author Niels Toepler
 *
 * \date 03.10.2023
 *
 * Contains declaration for classes IManip, ManipPrototype, Manip,
 * ImgSegmentSimple, ImgSegmentSpecial, BinaryImg.
 *
 */
#ifndef IMG_MANIP_INCLUDED
#define IMG_MANIP_INCLUDED

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include"remro_vision/motor_idx.h"
#include"remro_vision/rect_dims.h"

/**
 *
 * \class IManip
 *
 * \brief Interface for number of image manipulation classes
 *
 * All derived classes make use of at least one callback function which receives an <b>image msg</b>
 * and publishes an image from within that callback function.
 */
class IManip{

    protected:
        /**
         * \brief manipulate image container
         *
         * \param image the image container to manipulate
         *
         */
        virtual void manipulate(cv::Mat& image) = 0;

    public:
        /**
         * \brief ROS callback function
         *
         * \param msg ROS sensor message containing image data
         *
         */
        virtual void callback(const sensor_msgs::ImageConstPtr& msg) = 0;
        virtual ~IManip();

};

/**
 * \class ManipPrototype
 *
 * \brief Test class with no real callback and manipulate implementation.
 *
 */
class ManipPrototype : public IManip{

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber sub_;
        image_transport::Publisher pub_;

        void manipulate(cv::Mat& image);

    public:
        /**
         * \brief constructor
         *
         * Initializes transport.
         *
         * \param s_topic subscriber topic
         *
         * \param sq_size queue size of subscriber topic
         *
         * \param p_topic publisher topic
         *
         * \param pq_size queue size of publisher topic
         *
         */
        ManipPrototype(const char* s_topic, int sq_size, const char* p_topic, int pq_size);
        void callback(const sensor_msgs::ImageConstPtr& msg);
};

/**
 * \class Manip
 *
 * \brief Fully implements callback function.
 *
 */
class Manip : public ManipPrototype{

    protected:
        void manipulate(cv::Mat& image);

    public:
        Manip(const char* s_topic, int sq_size, const char* p_topic, int pq_size);
        void callback(const sensor_msgs::ImageConstPtr& msg);
};

/**
 * \class ImgSegmentSimple
 *
 * \brief Crops received image according to coordinates from topic `rectangle/dimensions`.
 *
 */
class ImgSegmentSimple : public Manip{

    protected:
        ///
        /// Region of interest. Contains the coordinates of the cropped image.
        cv::Rect roi_;
        ros::ServiceServer srv_;

        void manipulate(cv::Mat& image);
    public:
        ImgSegmentSimple(const char* s_topic, int sq_size, const char* p_topic, int pq_size);
        /**
         * \brief callback method to set region of interest
         *
         * \param req client request for crop dimensions
         *
         * \param res server response, either `'OK'` or an error message
         *
         */
        bool set_dim(remro_vision::rect_dims::Request& req, remro_vision::rect_dims::Response& res);
};

/**
 * \class ImgSegmentSpecial
 *
 * \brief Crops received image according to motor index from topic `rectangle/index`.
 *
 */
class ImgSegmentSpecial : public ImgSegmentSimple{

    protected: 
        ///
        /// Contains image coordinates and dimensions for all 24 servo motors on the first board.
        cv::Rect motor_rack_[24];

    public:
        ImgSegmentSpecial(const char* s_topic, int sq_size, const char* p_topic, int pq_size);
        /**
         * \brief analogous to ImgSegmentSimple::set_dim()
         *
         */
        bool set_dim_by_index(remro_vision::motor_idx::Request& req, remro_vision::motor_idx::Response& res);
};

/**
 * \class BinaryImg
 *
 * \brief Converts received image to binary image.
 * 
 * This method is specialized for the servo motors used from thr university project 
 * and thus it might not give the desired results on differing objects
 *
 */
class BinaryImg : Manip{

    protected:
        void manipulate(cv::Mat& Image);
        
    public:
        BinaryImg(const char* s_topic, int sq_size, int pq_size);
};

#endif
