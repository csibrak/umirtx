#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <sensor_msgs/PointField.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/SolidPrimitive.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char * argv[]){

    ros::init(argc, argv, "count_test_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    ROS_INFO("initialized cloud");
    ros::Rate loop_rate(10);
    //uint64_t i=0;

    ros::Publisher col_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
        ROS_INFO("initialized advertiser");
    moveit_msgs::CollisionObject co;
        ROS_INFO("initialized co");
    shape_msgs::SolidPrimitive sph;
        ROS_INFO("initialized sphere");
    sph.type=shape_msgs::SolidPrimitive::SPHERE;
    sph.dimensions.resize(1);
    sph.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.01;
    //sph.SPHERE_RADIUS=0.1;
    ROS_INFO("initialized sphere type");
    //sph.dimensions[0]=0.05;
    ROS_INFO("initialized sphere dimension");
    geometry_msgs::Pose pose;
        ROS_INFO("initialized pose");
    pose.orientation.w=1.0;
    pose.position.x=0.4;
    pose.position.y=0.0;
    pose.position.z=0.5;


    int j=0;
    while (nh.ok())
    {
        j++;
        ROS_INFO("cucc %d \r\n ",ros::Time::now().toNSec());
        cloud.width=4;
        cloud.height=1;
        cloud.points.resize(cloud.width * cloud.height);
        for (size_t i =0; i < cloud.points.size();++i)
        {
            cloud.points[i].x=(rand() / (RAND_MAX +1.0f))-0.3;
            cloud.points[i].y=(rand() / (RAND_MAX +1.0f))-0.5f;
            cloud.points[i].z=(rand() / (RAND_MAX +1.0f));
        }
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "base_link";
        output.header.seq=j;
        output.header.stamp.nsec=ros::Time::now().toNSec();
        output.header.stamp.sec=ros::Time::now().toSec();
//        pub.publish(output);

        co.primitives.push_back(sph);
            ROS_INFO("initialized pushed sphere");
        co.primitive_poses.push_back(pose);
            ROS_INFO("initialized pushed pose");
        co.operation =co.ADD;
            ROS_INFO("initialized sphere");
        //collision obj.
        co.header.frame_id = "base_link";
        co.header.seq=j;
        co.header.stamp.nsec=ros::Time::now().toNSec();
        co.header.stamp.sec=ros::Time::now().toSec();
        //co.header.frame_id="";
        col_pub.publish(co);


        ros::spinOnce();
        int c = getch();
        if(c=='w'){pose.position.x+=0.01;}
        if(c=='s'){pose.position.x-=0.01;}
        if(c=='a'){pose.position.y+=0.01;}
        if(c=='d'){pose.position.y-=0.01;}
        if(c=='+'){pose.position.z+=0.01;}
        if(c=='-'){pose.position.z-=0.01;}
        loop_rate.sleep();
    }
    return 0;
}
