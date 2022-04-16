#pragma once
#ifndef _MESSAGE_H_
#define _MESSAGE_H_
#include <string>

#include "msgs_ros2struct.h"
using namespace rosMsgs;

#define LIST_LENGTH 10 // the lenth of the queue

/**
 * @brief 测试1 3
 * 测试用例：std_msgs_Header
 */

struct std_msgs_Header_List
{
    int now; // the index of the current message
    int next;
    int count; // the number of all the message
    std_msgs_Header content[LIST_LENGTH];//有没有什么办法复用？
};


void addMessage_std_msgs_Header(std_msgs_Header_List* theList, std_msgs_Header& theMessage)
{
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}

/**
 * @brief sensor_msgs_PointCloud2
 * featureExtraction.cpp
 * 
 */
struct sensor_msgs_PointCloud2_List
{
    int now; // the index of the current message
    int next;
    int count; // the number of all the message
    sensor_msgs_PointCloud2 content[LIST_LENGTH];//有没有什么办法复用？
};

/**
 * @brief sensor_msgs_PointCloud2
 * featureExtraction.cpp
 * 
 */
void addMessage_sensor_msgs_PointCloud2(sensor_msgs_PointCloud2_List* theList, sensor_msgs_PointCloud2& theMessage)
{
    //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}

/**
 * @brief lio_sam_msgs_cloud_info
 * featureExtraction.cpp
 * 
 */

struct lio_sam_msgs_cloud_info_List
{
    int now;
    int next;
    int count;
    lio_sam_msgs_cloud_info content[LIST_LENGTH];
};

/**
 * @brief lvi_sam_msgs_cloud_info
 * featureExtraction.cpp
 * 
 */
void addMessage_lio_sam_msgs_cloud_info(lio_sam_msgs_cloud_info_List* theList, lio_sam_msgs_cloud_info& theMessage)
{
        //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}

/**
 * @brief nav_msgs_Odometry
 * mapOptmization.cpp
 * 
 */

struct nav_msgs_Odometry_List
{
    int now;
    int next;
    int count;
    nav_msgs_Odometry content[LIST_LENGTH * 200];
};

/**
 * @brief lvi_sam_msgs_cloud_info
 * featureExtraction.cpp
 * 
 */
void addMessage_nav_msgs_Odometry(nav_msgs_Odometry_List* theList, nav_msgs_Odometry& theMessage)
{
        //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % (LIST_LENGTH * 200);
    theList->count++;
}


/**
 * @brief nav_msgs_Odometry
 * mapOptmization.cpp
 * 
 */

struct geometry_msgs_TransformStamped_List
{
    int now;
    int next;
    int count;
    geometry_msgs_TransformStamped content[LIST_LENGTH];
};

/**
 * @brief lvi_sam_msgs_cloud_info
 * featureExtraction.cpp
 * 
 */
void addMessage_geometry_msgs_TransformStamped(geometry_msgs_TransformStamped_List* theList, geometry_msgs_TransformStamped& theMessage)
{
        //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}

/**
 * @brief nav_msgs_Odometry
 * mapOptmization.cpp
 * 
 */

struct msgs_TransformStampedGraph_List
{
    int now;
    int next;
    int count;
    msgs_TransformStampedGraph content[LIST_LENGTH];
};

/**
 * @brief lvi_sam_msgs_cloud_info
 * 
 * 
 */
void addMessage_msgs_TransformStampedGraph(msgs_TransformStampedGraph_List* theList, msgs_TransformStampedGraph& theMessage)
{
        //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}

struct nav_msgs_Path_List
{
    int now;
    int next;
    int count;
    nav_msgs_Path content[LIST_LENGTH];
};


/**
 * @brief lvi_sam_msgs_cloud_info
 * featureExtraction.cpp
 * 
 */
void addMessage_nav_msgs_Path_List(nav_msgs_Path_List* theList, nav_msgs_Path& theMessage)
{
        //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}


struct sensor_msgs_Imu_List
{
    int now;
    int next;
    int count;
    sensor_msgs_Imu content[LIST_LENGTH];
};


/**
 * @brief lvi_sam_msgs_cloud_info
 * featureExtraction.cpp
 * 
 */
void addMessage_sensor_msgs_Imu_List(sensor_msgs_Imu_List* theList, sensor_msgs_Imu& theMessage)
{
        //改一下now的定义
    int theIndex = theList->next;
    theList->content[theIndex] = theMessage;
    theList->now = theList->next;
    theList->next = (theIndex + 1) % LIST_LENGTH;
    theList->count++;
}



#endif