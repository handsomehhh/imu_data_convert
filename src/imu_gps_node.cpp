#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <list>
#include <pthread.h>
#include <string.h>
#include <iostream>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include <termios.h>//speed1、write()
#include <fcntl.h>//open
#include <sys/ipc.h>
#include <sys/msg.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#include <fstream>

#include "./communication_LVI_SAM/communicator1.h"
#include "time.h"

using namespace std;
    
    sensor_msgs::Imu imu_data;
    sensor_msgs::NavSatFix gps_data;

    sensor_msgs_Imu imu_data_struct;

    
    



    #define MAXLINE 4096
    #define PI 3.1415926

//   std::ofstream file1;


    
speed_t getBaudrate(int baudrate)
{
    switch (baudrate)
    {
    case 0:
        return B0;
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}

int open_tty(char const *path, int baudrate, int flags)
{
    int fd;
    speed_t speed;

    if ((speed = getBaudrate(baudrate)) == -1)
    {
        printf("FAILED  to open %s\n", path);
        return -1;
    }
    if ((fd = open(path, O_RDWR|flags)) == -1)
    {
        printf("FAILED  to open %s\n", path);
        return -1;
    }

    struct termios cfg;
    if (tcgetattr(fd, &cfg))
    {
        close(fd);
        printf("FAILED  to open %s\n", path);
        return -1;
    }
    cfmakeraw(&cfg);
    cfsetispeed(&cfg, speed);
    cfsetospeed(&cfg, speed);
    cfg.c_cc[VMIN] = 1;
    cfg.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &cfg))
    {
        close(fd);
        printf("FAILED  to open %s\n", path);
        return -1;
    }

    printf("open %s ok,fd=%d\n", path, fd);
    return fd;
}


int BBC1(char *buf)

{   
    int  ret = buf[1];
    // printf("ret %x\n",ret);
    int ind = strlen(buf);
    // cout<<ind<<endl;
    // printf("ret1 %c\n",buf[ind-5]);
    // printf("ret1 %c\n",buf[ind-6]);
    
    for(int i = 2;buf[i] != '*';i++)
    {
        ret ^= buf[i];
    }
    return ret;

}



long timestamp()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000000 + tv.tv_usec);
    cout<<"微秒： "<<tv.tv_sec * 1000000 + tv.tv_usec<<endl;
}
// long getTime(void)
// {
//   const auto t = std::chrono::system_clock::now();
//   const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
//   return (long)t_sec.count();
// }
        

bool nan_handle(char *data_buf)
{
    int check= BBC1(data_buf);

    

    // printf("check %x\n",check);
    
    
char *cmdtype;
int GPSWeek;
float GPSTime, yaw, pitch, roll, latitude, longitude, height, east_speed, north_speed, sky_speed;
float x_angle_velocity, y_angle_velocity, z_angle_velocity, x_acceleration, y_acceleration,z_acceleration;
int NSV1,NSV2,pose_type,heading_type;
int state,checksum;



//    char check1 = check;
//    printf("check1 %x\n",check);



        cmdtype = strtok(data_buf, ",");//buf+1?
        cout<<*cmdtype<<endl;


        if(strcmp(cmdtype, "$BDFPDL") == 0 )
        {
            

                

        sscanf(strtok(NULL, ","), "%x", &GPSWeek);
        sscanf(strtok(NULL, ","), "%f", &GPSTime);
        sscanf(strtok(NULL, ","), "%f", &yaw);
        sscanf(strtok(NULL, ","), "%f", &pitch);
        sscanf(strtok(NULL, ","), "%f", &roll);
        
        sscanf(strtok(NULL, ","), "%f", &latitude);
        sscanf(strtok(NULL, ","), "%f", &longitude);
        sscanf(strtok(NULL, ","), "%f", &height);
        sscanf(strtok(NULL, ","), "%f", &east_speed);
        sscanf(strtok(NULL, ","), "%f", &north_speed);
        sscanf(strtok(NULL, ","), "%f", &sky_speed);
        
        sscanf(strtok(NULL, ","), "%f", &x_angle_velocity);
        sscanf(strtok(NULL, ","), "%f", &y_angle_velocity);
        sscanf(strtok(NULL, ","), "%f", &z_angle_velocity);
        sscanf(strtok(NULL, ","), "%f", &x_acceleration);
        sscanf(strtok(NULL, ","), "%f", &y_acceleration);
        sscanf(strtok(NULL, ","), "%f", &z_acceleration);
        sscanf(strtok(NULL, ","), "%x", &NSV1);
        
        sscanf(strtok(NULL, ","), "%x", &NSV2);
        sscanf(strtok(NULL, ","), "%x", &pose_type);
            
        sscanf(strtok(NULL, ","), "%x", &heading_type);
        
        sscanf(strtok(NULL, "*"), "%x", &state);
        
        sscanf(strtok(NULL, "*"), "%x", &checksum);
        
        
        
        
        
        //状态码
        if(state == 0x00)
        {
            printf("imu+GNSS Standby\n");
        }
        else if(state == 0x10)
        {
            printf("DONT MOVE ,Coarse alignment，waiting for BDS...\n");
        }
        else if(state == 0x20)
        {
            printf("Fine alignment，waiting...\n");
        }
        else if(state == 0x30 || state == 0x31)
        {
            printf("imu+ GNSS is already....");
        }

        // memset(data_buf, 0, sizeof(data_buf));

        yaw=yaw/180*PI;
        pitch=pitch/180*PI;
        roll =roll/180*PI;
        
        Eigen::Vector3f eulerAngle(yaw,pitch,roll);
        Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitZ())); 
        Eigen::Quaternionf quaternion ;
        quaternion=yawAngle*pitchAngle*rollAngle;


        imu_data.header.stamp = ros::Time::now();
        
        //获取当前时间戳（使用与lidar驱动一样的时间获取方式）
        imu_data_struct.header.stamp = system_time::toTimeFromSec(system_time::getNowTimestamp());


        imu_data.header.frame_id = "imu_link";
        string temp_imu_name = "imu_link";
        strcpy(imu_data_struct.header.frame_id, temp_imu_name.c_str());

        // long timesatmp = timestamp(); // 应该跟我获取的time一样

        // cout<<"timeStamp: "<<timesatmp<<endl;
        


        imu_data.orientation.x=quaternion.x();
        imu_data.orientation.y=quaternion.y();
        imu_data.orientation.z=quaternion.z();
        imu_data.orientation.w=quaternion.w();

        imu_data.orientation_covariance[0]=0.01;
        imu_data.orientation_covariance[1]=0;
        imu_data.orientation_covariance[2]=0;
        imu_data.orientation_covariance[3]=0;
        imu_data.orientation_covariance[4]=0.01;
        imu_data.orientation_covariance[5]=0;
        imu_data.orientation_covariance[6]=0;
        imu_data.orientation_covariance[7]=0;
        imu_data.orientation_covariance[8]=0.01;
        //   imu_data.orientation_covariance[]=;

        imu_data.angular_velocity.x= x_angle_velocity/180*PI;
        imu_data.angular_velocity.y= y_angle_velocity/180*PI;
        imu_data.angular_velocity.z= z_angle_velocity/180*PI;

        imu_data.angular_velocity_covariance[0] =0.01;
        imu_data.angular_velocity_covariance[1] =0;
        imu_data.angular_velocity_covariance[2] =0;
        imu_data.angular_velocity_covariance[3] =0;
        imu_data.angular_velocity_covariance[4] =0.01;
        imu_data.angular_velocity_covariance[5] =0;
        imu_data.angular_velocity_covariance[6] =0;
        imu_data.angular_velocity_covariance[7] =0;
        imu_data.angular_velocity_covariance[8] =0.01;
        //   imu_data.angular_velocity_covariance[] =;

        imu_data.linear_acceleration.x = x_acceleration;
        imu_data.linear_acceleration.y = y_acceleration;
        imu_data.linear_acceleration.z = z_acceleration;

        imu_data.linear_acceleration_covariance[0] =0.01;
        imu_data.linear_acceleration_covariance[1] =0;
        imu_data.linear_acceleration_covariance[2] =0;
        imu_data.linear_acceleration_covariance[3] =0;
        imu_data.linear_acceleration_covariance[4] =0.01;
        imu_data.linear_acceleration_covariance[5] =0;
        imu_data.linear_acceleration_covariance[6] =0;
        imu_data.linear_acceleration_covariance[7] =0;
        imu_data.linear_acceleration_covariance[8] =0.01;





        imu_data_struct.orientation.x=quaternion.x();
        imu_data_struct.orientation.y=quaternion.y();
        imu_data_struct.orientation.z=quaternion.z();
        imu_data_struct.orientation.w=quaternion.w();

        imu_data_struct.orientation_covariance[0]=0.01;
        imu_data_struct.orientation_covariance[1]=0;
        imu_data_struct.orientation_covariance[2]=0;
        imu_data_struct.orientation_covariance[3]=0;
        imu_data_struct.orientation_covariance[4]=0.01;
        imu_data_struct.orientation_covariance[5]=0;
        imu_data_struct.orientation_covariance[6]=0;
        imu_data_struct.orientation_covariance[7]=0;
        imu_data_struct.orientation_covariance[8]=0.01;
        //   imu_data.orientation_covariance[]=;

        imu_data_struct.angular_velocity.x= x_angle_velocity/180*PI;
        imu_data_struct.angular_velocity.y= y_angle_velocity/180*PI;
        imu_data_struct.angular_velocity.z= z_angle_velocity/180*PI;

        imu_data_struct.angular_velocity_covariance[0] =0.01;
        imu_data_struct.angular_velocity_covariance[1] =0;
        imu_data_struct.angular_velocity_covariance[2] =0;
        imu_data_struct.angular_velocity_covariance[3] =0;
        imu_data_struct.angular_velocity_covariance[4] =0.01;
        imu_data_struct.angular_velocity_covariance[5] =0;
        imu_data_struct.angular_velocity_covariance[6] =0;
        imu_data_struct.angular_velocity_covariance[7] =0;
        imu_data_struct.angular_velocity_covariance[8] =0.01;
        //   imu_data.angular_velocity_covariance[] =;

        imu_data_struct.linear_acceleration.x = x_acceleration;
        imu_data_struct.linear_acceleration.y = y_acceleration;
        imu_data_struct.linear_acceleration.z = z_acceleration;

        imu_data_struct.linear_acceleration_covariance[0] =0.01;
        imu_data_struct.linear_acceleration_covariance[1] =0;
        imu_data_struct.linear_acceleration_covariance[2] =0;
        imu_data_struct.linear_acceleration_covariance[3] =0;
        imu_data_struct.linear_acceleration_covariance[4] =0.01;
        imu_data_struct.linear_acceleration_covariance[5] =0;
        imu_data_struct.linear_acceleration_covariance[6] =0;
        imu_data_struct.linear_acceleration_covariance[7] =0;
        imu_data_struct.linear_acceleration_covariance[8] =0.01;
        //   imu_data.linear_acceleration_covariance[] =;
    //    cout<<"imu_data: "<<imu_data<<endl;
        
        
        //pubImu.publish(imu_data);
    
        // sensor_msgs::NavSatFix gps_data;

        gps_data.header.stamp =  ros::Time::now();

        gps_data.header.frame_id = "base_link";

        gps_data.latitude = latitude;
        gps_data.longitude = longitude;
        gps_data.altitude = height;


        // gps_data.position_covariance[0]=0.2;
        // gps_data.position_covariance[1]=;                
        // gps_data.position_covariance[2]=
        // gps_data.position_covariance[3]=
        // gps_data.position_covariance[4]=       
        // gps_data.position_covariance[5]=        
        // gps_data.position_covariance[6]=
        // gps_data.position_covariance[7]=
        // gps_data.position_covariance[8]=        

        if(isnanf(yaw)||isnanf(pitch)||isnanf(roll)||isnanf(x_angle_velocity)||isnanf(y_angle_velocity)
                        ||isnanf(z_angle_velocity)||isnanf(x_acceleration)||isnanf(y_acceleration)||isnanf(z_acceleration))
        {
                
                cout<<"nan"<<endl;
                return 0;
        }
        
        else{
            
            cout<<"----------------------------------------"<<endl;
        }
                    
                            
        //         // file1<<"yaw: "<<yaw<<" pitch: "<<pitch<<" raw: "<<roll
        //         //  <<" x_angle_velocity: "<<x_angle_velocity<<" y_angle_velocity: "
        //         // <<y_angle_velocity<<" z_angle_velocity: "<<z_angle_velocity
        //         // <<" x_acceleration: "<<x_acceleration<<" y_acceleration: "<<y_acceleration
        //         // <<" z_acceleration: "<<z_acceleration<<endl;

        //         return 1;     
        //         }////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        return check == checksum ;
        


        // gps_data.position_covariance= ;
        // pubGps.publish(gps_data);
        }
        else
        {
            printf("wrong frame!");

            return 0;
        }
    
}





int main(int argc, char **argv)
{

    MemoryDefinition* pub_imu = createShareMemory("lio-sam-imu",sizeof(sensor_msgs_Imu_List));
    
    ros::init(argc,argv,"imu_gps");//定义节点
    ros::NodeHandle n;//定义句柄;
    ros::Publisher pubImu = n.advertise<sensor_msgs::Imu>("/imu_raw",2000);
    ros::Publisher pubGps = n.advertise<sensor_msgs::NavSatFix>("/gps_data",2000);
    
    /////////////////   

ros::Rate loop_rate(100);

    char dev_name[200]="/dev/ttyUSB0";
    int imu_gps_fd = -1;
    imu_gps_fd = open_tty(dev_name, 460800, 0);
    ROS_INFO("imu_gps start\n");
    
char buf[0];


//    file1.open("imu_gps.txt",std::ofstream::app);


char data_buf[MAXLINE];
int new_frame=0;
    int buf_size =0;


    while(ros::ok())
{    
    
    int ret = read(imu_gps_fd,buf,1);

    

    //printf("buf: %s\n",buf);
    
    if(ret>0)
    {
        
        if(new_frame == 0)
        {
        
        if(buf[0]=='$')
        {
            
            new_frame = 1;
        }
        
        }
    if(new_frame == 1)
    {

        
        char tem_buf[MAXLINE];
        //memset(tem_buf, 0, sizeof(tem_buf));
        tem_buf[buf_size]=buf[0];

        if(buf_size > 140  && tem_buf[buf_size]=='\n' && tem_buf[buf_size-1]=='\r')
            {
                buf_size = -1;
                if(nan_handle(tem_buf))
                {
                    pubImu.publish(imu_data);

                    //发布自定义类型
                    addMessage_sensor_msgs_Imu_List((sensor_msgs_Imu_List*)pub_imu->point, imu_data_struct);
                    pubGps.publish(gps_data);
                
                    
                cout<<"imu_data: "<<imu_data<<endl;
                cout <<"gps_data: "<< gps_data<<endl;
                }
                
                memset(tem_buf, 0, sizeof(tem_buf));
                new_frame=0;
                
            }
                buf_size++;
//                if(buf_size >155){
//                    buf_size =0;
//                }
//                cout<< "buf_size: "<<buf_size <<endl;

            
    }
    }
    else{
        cout<<"ret: "<<ret<<endl;
        cout<<"imu wrong..."<<endl;
    }
}
    
        


    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    // ros::spin();
    //ros::MultiThreadedSpinner spinner(4);
// spinner.spin();
// ??
    ros::spinOnce(); 
    loop_rate.sleep(); 

    return 0;

}
