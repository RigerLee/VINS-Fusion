/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "pose_graph.h"
#include "utility/visualization.h"

// lee: Init both estimator and posegraph
Estimator estimator;
PoseGraph posegraph;


queue<sensor_msgs::ImageConstPtr> img_buf;
queue<sensor_msgs::ImageConstPtr> depth_buf;
std::mutex m_buf;

int frame_index  = 0;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img_buf.push(img_msg);
    m_buf.unlock();
}

void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    m_buf.lock();
    depth_buf.push(depth_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

cv::Mat getDepthFromMsg(const sensor_msgs::ImageConstPtr &depth_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (depth_msg->encoding == "16UC1")
    {
        sensor_msgs::Image img;
        img.header = depth_msg->header;
        img.height = depth_msg->height;
        img.width = depth_msg->width;
        img.is_bigendian = depth_msg->is_bigendian;
        img.step = depth_msg->step;
        img.data = depth_msg->data;
        img.encoding = "mono16";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    }
    else
        ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(RGBD)
        {
            // lee: Estimator get pose, give to posegraph
            // lee: How to give?
            Result result = estimator.inputImage(time, image, depth);
            KeyFrame* keyframe = new KeyFrame(result._timestamp, frame_index, result._T, result._R, image,
                                              result._point_3d, result._point_2d_uv, result._point_2d_normal,
                                              result._point_id, sequence);
//            start_flag = 1;
            // lee: 实时给出posegraph的位姿, 可能不是刚优化好的(为了保证实时性
            // lee: query直接访问对应的keyframe list下标即可(或者向前或向后查找?)
            posegraph.addKeyFrame(keyframe, 1);

            frame_index++;

        }
        // Mono not used
        else
        {
            estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for rgb and depth");

    registerPub(n);

    // Use message filter for rgb and depth
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    ros::Subscriber sub_depth = n.subscribe(DEPTH_TOPIC, 100, depth_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
