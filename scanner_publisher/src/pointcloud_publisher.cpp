#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <deque>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>

using namespace std;


///#include "livox_sdk.h"

float setting_speed;
float setting_angle;
bool save_points = false;


using namespace std;
typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudTI;
ros::Publisher pointcloud_pub;
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t intensity;
} __attribute__((packed)) lidar_point;

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;

    float ref;    // target value
    float fdb;    // feedback value
    float err[2]; // error and last error

    float p_out;
    float i_out;
    float d_out;
    float output;
} pid_struct_t;

typedef struct
{
    int16_t target_volt;
    uint16_t fdb_enc;
    int16_t fdb_rpm;
    int16_t fdb_current;
} motor_ctrl_t;

pid_struct_t motor_pid;
motor_ctrl_t glb_motor;

int s;
float glb_angle;
mutex glb_angle_lock;

vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_vector;

static void pid_init(pid_struct_t *pid, float kp, float ki, float kd,
                     float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
}

static float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->p_out = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out = pid->kd * pid->err[0] - pid->err[1];
    LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
    return pid->output;
}

static void *can_send_thread(void *arg)
{

    float target_rpm = setting_speed;

    pid_init(&motor_pid, 10, 2, 0, 30000, 30000);

    glb_motor.target_volt = 0;

    int nbytes;
    struct can_frame frame;

    frame.can_id = 0x2FF;
    frame.can_dlc = 8;
    int i=0;

    while (1)
    {
        if(i++%10==0)
        {
            frame.data[4] = (glb_motor.target_volt >> 8) & 0xff;
            frame.data[5] = (glb_motor.target_volt) & 0xff;

            nbytes = write(s, &frame, sizeof(frame));
            if (nbytes != sizeof(frame))
            {
                printf("Send Error frame\n!");
                break;
            }
        }
        glb_angle_lock.lock();
        
        if (glb_angle > 177 + setting_angle/2) //357
            target_rpm = -setting_speed;

        if (glb_angle < 183 - setting_angle/2) //3
            target_rpm = setting_speed;
            
        glb_angle_lock.unlock();

        glb_motor.target_volt = pid_calc(&motor_pid, target_rpm, glb_motor.fdb_rpm);
        
        usleep(1000);
    }

    return NULL;
}

static void *can_recv_thread(void *arg)
{
    struct can_frame frame;
    struct timeval tv;
    fd_set rset;

    int nbytes, ret;

    while (1)
    {
        tv.tv_sec = 0;
        tv.tv_usec = 200;

        FD_ZERO(&rset);
        FD_SET(s, &rset);
        ret = select(s + 1, &rset, NULL, NULL, NULL);

        if (0 == ret)
        {
            return NULL;
        }

        nbytes = read(s, &frame, sizeof(frame));

        if (nbytes > 0)
        {
            glb_motor.fdb_enc = ((frame.data[0]) << 8 | frame.data[1]);
            glb_motor.fdb_rpm = ((frame.data[2]) << 8 | frame.data[3]);
            glb_motor.fdb_current = ((frame.data[4]) << 8 | frame.data[5]);

            glb_angle_lock.lock();
            glb_angle = glb_motor.fdb_enc * 360 / 8192.0;
            glb_angle_lock.unlock();
            //printf("enc:%d rpm:%d angle:%.2lf\n", glb_motor.fdb_enc, glb_motor.fdb_rpm, glb_angle);
        }
    }

    return NULL;
}

PointCloudTI::Ptr pc_cache_ptr(new PointCloudTI);
mutex pc_cache_ptr_lock;

void GetLidarData(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    pcl::PointCloud<pcl::PointXYZI> lidarPoints;
    pcl::fromROSMsg(*cloud_msg,lidarPoints);
    int dnum=lidarPoints.points.size();
    glb_angle_lock.lock();
    float rotate_angle=glb_angle;
    glb_angle_lock.unlock();
    
    PointCloudTI::Ptr pc_temp_ptr(new PointCloudTI);    
    for (int i = 0; i < dnum; i++)
    {
        pcl::PointXYZRGB temp_point;
        temp_point.x = lidarPoints.points[i].x;
        temp_point.y = lidarPoints.points[i].y;
        temp_point.z = lidarPoints.points[i].z;
        int reflection_map = (int)lidarPoints.points[i].intensity;
        if (reflection_map < 30)
        {
            int green = (reflection_map * 255 / 30);
            temp_point.r = 0x0;
            temp_point.g = green & 0xff;
            temp_point.b = 0xff;
        }
        else if (reflection_map < 90)
        {
            int blue = (((90 - reflection_map) * 255) / 60);
            temp_point.r = 0x0;
            temp_point.g = 0xff;
            temp_point.b = blue & 0xff;
        }
        else if (reflection_map < 150)
        {
            int red = ((reflection_map - 90) * 255) / 60;
            temp_point.r = red & 0xff;
            temp_point.g = 0xff;
            temp_point.b = 0x0;
        }
        else
        {
            int green = ((255 - reflection_map) * 255) / (256 - 150);
            temp_point.r = 0xff;
            temp_point.g = green & 0xff;
            temp_point.b = 0x0;
        }
        pc_temp_ptr->push_back(temp_point);
    }

    Eigen::Isometry3f tran_isometry = Eigen::Isometry3f::Identity();
    Eigen::Matrix3f rotate_matrix;
    Eigen::Matrix4f tran_matrix;

    Eigen::AngleAxisf rotate_angle_axis(rotate_angle/360*2*M_PI, Eigen::Vector3f(0, 0, 1));
    rotate_matrix = rotate_angle_axis.toRotationMatrix();
    tran_isometry.rotate(rotate_matrix);
    tran_matrix=tran_isometry.matrix();
    
    
    pc_cache_ptr->clear();
    pcl::transformPointCloud(*pc_temp_ptr,*pc_cache_ptr,tran_matrix);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pc_cache_ptr,msg);

    //save points
    if(save_points){
        string save_name = "./pointscloud2.csv";
        ofstream outfile;
        outfile.open(save_name,ios::app);
        for(int pid=0;pid<dnum;pid++){
            outfile<<pc_cache_ptr->points[pid].x<<','
                <<pc_cache_ptr->points[pid].y<<','
                <<pc_cache_ptr->points[pid].z<<','
                <<lidarPoints.points[pid].intensity
                <<"\n";
        }
        outfile.close();
    }
    msg.header.frame_id = "livox_frame";
    msg.header.stamp.sec = cloud_msg->header.stamp.sec;
    msg.header.stamp.nsec = cloud_msg->header.stamp.nsec;
    pointcloud_pub.publish(msg);
    ROS_INFO("rotation_angle:=%f,topic_num:= %d",rotate_angle,dnum);

}

int main(int argc, char **argv)
{
    ifstream infile;

    infile.open("./speed_config.txt");

    
    if(!infile)
    {
        cout<<"No config file, use default param"<<endl;
        setting_speed = 2;
        setting_angle = 100;
    }
    else
    {
        vector<string> param_ver;
        
        while (!infile.eof())
        {
            string buf;
            getline(infile, buf, '\n');
            param_ver.push_back(buf);
        }
        
        cout<<"line0: "<<param_ver[0]<<endl;
        cout<<"line1: "<<param_ver[1]<<endl;

        setting_speed = atof(param_ver[0].c_str());
        setting_angle = atof(param_ver[1].c_str());
        
        LIMIT_MIN_MAX(setting_speed, 0, 10);
        LIMIT_MIN_MAX(setting_angle, 30, 360);
    }
    
    cout<<"speed: "<<setting_speed<<endl;
    cout<<"angle: "<<setting_angle<<endl;

    infile.close();
    
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[1];

    pthread_t thread_send;
    pthread_t thread_recv;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    //can0
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    //set up filter rules
    rfilter[0].can_id = 0x20B;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    pthread_create(&thread_send, NULL, can_send_thread, NULL);
    pthread_create(&thread_recv, NULL, can_recv_thread, NULL);

    cout<<"good0"<<endl;
    ros::init(argc, argv, "pointcloud_publisher");
    ros::NodeHandle n;
    ros::Subscriber customCloud = n.subscribe<sensor_msgs::PointCloud2>("/livox/lidar",10,GetLidarData);
    pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("lidar_pointcloud", 10);
    ros::spin();
    return 0;
}
