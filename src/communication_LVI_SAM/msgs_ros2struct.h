#pragma once
#ifndef _MSGS_ROS2STRUCTS_H_
#define _MSGS_ROS2STRUCTS_H_

#include <stdint.h>
#include <stddef.h>


#define STRING_LENGTH 256

#define MAX_tf 10 

/**
 * @brief 将项目中用到的ros中的msg类型重新定义成结构体
 * xxx_msgs_name
 * uint32 -- uint32_t
 * int32 -- int32_t 以此类推
 * time duration -- 类 成员变量：int32 sec,int32 nsec https://www.cnblogs.com/HaoQChen/p/11048603.html https://blog.csdn.net/weixin_45590473/article/details/121289430
 * ???[]如何定义
 * sensor_msgs::imags_encodeings http://docs.ros.org/en/diamondback/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
 * 添加维护指针大小的变量
 */
namespace rosMsgs
{
    struct msgs_Time
    {
        uint32_t sec;
        uint32_t nsec;
    };
    struct msgs_duration
    {
        uint32_t sec;
        uint32_t nsec;
    };

    struct std_msgs_Header
    {
        uint32_t seq;
        msgs_Time stamp;
        char frame_id[STRING_LENGTH];
    };

    struct sensor_msgs_PointField
    {
        uint8_t INT8=1;
        uint8_t UINT8=2;
        uint8_t INT16=3;
        uint8_t UINT16=4;
        uint8_t INT32=5;
        uint8_t UINT32=6;
        uint8_t FLOAT32=7;
        uint8_t FLOAT64=8;
        char name[STRING_LENGTH];
        uint32_t offset;
        uint8_t datatype;
        uint32_t count;
    };

    struct sensor_msgs_PointCloud2
    {
        std_msgs_Header header;
        uint32_t height;
        uint32_t width;
        uint32_t fields_number;
        sensor_msgs_PointField* fields;
        uint8_t is_bigendian;
        uint32_t point_step;
        uint32_t row_step;
        uint32_t data_number;
        uint8_t* data;
        uint8_t is_dense;
    };

    struct geometry_msgs_Quaternion
    {
        double x;
        double y;
        double z;
        double w;
    };

    struct geometry_msgs_Vector3
    {
        double x;
        double y;
        double z;
    };

    struct sensor_msgs_Imu
    {
        std_msgs_Header header;
        geometry_msgs_Quaternion orientation;
        double orientation_covariance[9];
        geometry_msgs_Vector3 angular_velocity;
        double angular_velocity_covariance[9];
        geometry_msgs_Vector3 linear_acceleration;
        double linear_acceleration_covariance[9];
    };

    struct geometry_msgs_Point
    {
        double x;
        double y;
        double z;
    };

    struct geometry_msgs_Pose
    {
        geometry_msgs_Point position;
        geometry_msgs_Quaternion orientation;
    };

    struct geometry_msgs_PoseWithCovariance
    {
        geometry_msgs_Pose pose;
        double covariance[36];
    };

    struct geometry_msgs_Twist
    {
        geometry_msgs_Vector3 linear;
        geometry_msgs_Vector3 angular;
    };

    struct geometry_msgs_TwistWithCovariance
    {
        geometry_msgs_Twist twist;
        double covariance[36];
    };

    struct nav_msgs_Odometry
    {
        std_msgs_Header header;
        char child_frame_id[STRING_LENGTH];
        geometry_msgs_PoseWithCovariance pose;
        geometry_msgs_TwistWithCovariance twist;
        int number;
    };

    struct geometry_msgs_PoseStamped
    {
        std_msgs_Header header;
        geometry_msgs_Pose pose;
    };

    struct nav_msgs_Path
    {
        std_msgs_Header header;
        uint32_t poses_number;
        geometry_msgs_PoseStamped* poses;
    };

    struct std_msgs_MultiArrayDimension
    {
        char label[STRING_LENGTH];
        uint32_t size;
        uint32_t stride;
    };

    struct std_msgs_MultiArrayLayout
    {
        uint32_t dim_number;
        std_msgs_MultiArrayDimension* dim;
        uint32_t data_offset;
    };

    struct std_msgs_Float64MultiArray
    {
        std_msgs_MultiArrayLayout layout;
        uint32_t data_number;
        double* data;
    };

    struct std_msgs_ColorRGBA
    {
        float r;
        float g;
        float b;
        float a;

    };

    struct visualization_msgs_Marker
    {
        uint8_t ARROW=0;
        uint8_t CUBE=1;
        uint8_t SPHERE=2;
        uint8_t CYLINDER=3;
        uint8_t LINE_STRIP=4;
        uint8_t LINE_LIST=5;
        uint8_t CUBE_LIST=6;
        uint8_t SPHERE_LIST=7;
        uint8_t POINTS=8;
        uint8_t TEXT_VIEW_FACING=9;
        uint8_t MESH_RESOURCE=10;
        uint8_t TRIANGLE_LIST=11;
        uint8_t ADD=0;
        uint8_t MODIFY=0;
        uint8_t DELETE=2;
        uint8_t DELETEALL=3;
        std_msgs_Header header;
        char ns[STRING_LENGTH];
        int32_t id;
        int32_t type;
        int32_t action;
        geometry_msgs_Pose pose;
        geometry_msgs_Vector3 scale;
        std_msgs_ColorRGBA color;
        msgs_duration lifetime;
        uint8_t frame_locked;
        uint32_t points_number;
        geometry_msgs_Point* points;
        uint32_t colors_number;
        std_msgs_ColorRGBA* colors;
        char text[STRING_LENGTH];
        char mesh_resource[STRING_LENGTH];
        uint8_t mesh_use_embedded_materials;
    };

    /**
     * @brief 比较不好的类型
     * 指针套着指针
     * 但lidar部分没用到过这个类型 还好
     */
    struct visualization_msgs_MarkerArray
    {
        uint32_t markers_number;
        visualization_msgs_Marker* markers;
    };

    struct sensor_msgs_image
    {
        std_msgs_Header header;
        uint32_t height;
        uint32_t width;
        char encoding[STRING_LENGTH];
        uint8_t is_bigendian;
        uint32_t step;
        uint32_t data_number;
        uint8_t* data;
    };

    struct std_msgs_Bool
    {
        uint8_t data;
    };

    struct sensor_msgs_ChannelFloat32
    {
        char name[STRING_LENGTH];
        uint32_t values_number;
        float* values;
    };

    struct geometry_msgs_Point32
    {
        float x;
        float y;
        float z;
    };

    /**
     * @brief 比较不好的类型
     * 指针套着指针
     * 但lidar部分没用到过这个类型 还好
     */
    struct sensor_msgs_PointCloud
    {
        std_msgs_Header header;
        uint32_t points_number;
        geometry_msgs_Point32* pionts;
        uint32_t channels_number;
        sensor_msgs_ChannelFloat32* channels;
    };

    /**
     * @brief 这是lvi-sam项目中自定义的消息类型
     * 
     */
    struct lvi_sam_msgs_cloud_info
    {
        // Cloud Info
        std_msgs_Header header; 

        uint32_t startRingIndex_number;
        int32_t* startRingIndex;
        uint32_t endRingIndex_number;
        int32_t* endRingIndex;

        uint32_t pointColInd_number;
        int32_t*  pointColInd; // point column index in range image
        uint32_t pointRange_number;
        float* pointRange; // point range 

        int64_t imuAvailable;
        int64_t odomAvailable;

        // Attitude for lidar odometry initialization
        float imuRollInit;
        float imuPitchInit;
        float imuYawInit;

        // Odometry 
        float odomX;
        float odomY;
        float odomZ;
        float odomRoll;
        float odomPitch;
        float odomYaw;

        // Odometry reset ID
        int64_t odomResetId;

        // Point cloud messages
        sensor_msgs_PointCloud2 cloud_deskewed ; // original cloud deskewed
        sensor_msgs_PointCloud2 cloud_corner ;   // extracted corner feature
        sensor_msgs_PointCloud2 cloud_surface  ; // extracted surface feature

    };

        /**
     * @brief 这是lvi-sam项目中自定义的消息类型
     * 
     */
    struct lio_sam_msgs_cloud_info
    {
        // Cloud Info
        std_msgs_Header header; 

        uint32_t startRingIndex_number;
        int32_t* startRingIndex;
        uint32_t endRingIndex_number;
        int32_t* endRingIndex;

        uint32_t pointColInd_number;
        int32_t*  pointColInd; // point column index in range image
        uint32_t pointRange_number;
        float* pointRange; // point range 

        int64_t imuAvailable;
        int64_t odomAvailable;

        // Attitude for lidar odometry initialization
        float imuRollInit;
        float imuPitchInit;
        float imuYawInit;

        // Initial guess from imu pre-integration
        float initialGuessX;
        float initialGuessY;
        float initialGuessZ;
        float initialGuessRoll;
        float initialGuessPitch;
        float initialGuessYaw;

        // Point cloud messages
        sensor_msgs_PointCloud2 cloud_deskewed;  // original cloud deskewed
        sensor_msgs_PointCloud2 cloud_corner;    // extracted corner feature
        sensor_msgs_PointCloud2 cloud_surface;   // extracted surface feature

    };



    /**
     * @brief TF中用到的类型
     * 
     */


    struct geometry_msgs_Transform
    {
        geometry_msgs_Vector3 translation;
        geometry_msgs_Quaternion rotation;
    };
    
    struct geometry_msgs_TransformStamped
    {
        std_msgs_Header header;
        char child_frame_id[STRING_LENGTH];
        geometry_msgs_Transform transform;
    };

    struct msgs_TransformStampedGraph
    {
        geometry_msgs_TransformStamped tf_graph[MAX_tf][MAX_tf];
        //name 顺序代表id
        char names[MAX_tf][STRING_LENGTH];
        //标记，记录每个id是否被占用
        uint8_t flags[MAX_tf];
        //标记每个对应关系
        uint8_t tf_graph_flags[MAX_tf][MAX_tf];


    };






}

#endif