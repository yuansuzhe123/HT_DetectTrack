/*******************************************************************/
/*
 * @Author: Yuan
 * @Date: 2022-10-14 16:34:41
 * @Email:824100902@qq.com
 * @Description: 
 * @Beta:2.0
 */
/*******************************************************************/

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <sstream>
#include <math.h>
#include <chrono>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "msg_obj_perception/Obj.h"
#include "module4/fusionmap.h"
#include "msg_lidar_obj/msg_lidar_obj.h"
#include "obj_visualization.h"
#include "convex_hull_granham.h"
#include "tracker.h"
//#include "common.hpp"
#include "msg_box/Box.h"
#include "obj_visualization.h"
#include <queue>
#define InitTimer(timer)     vector<double> timer;\
    timer.push_back(ros::Time::now().toSec());

#define PrintWithTimeConsumption(timer,content)     timer.push_back(ros::Time::now().toSec());\
    std::cout << "--T-- " << content << " cost time: " < \
    (timer[timer.size()-1] - timer[timer.size()-2])*1000 << "ms" << std::endl; 

struct TrackResult {
  int oriid, trackid;
  double vx, vy;
  double tracking_time;
  int m_flag;
};
// using namespace cv;
// using namespace pcl;
using namespace std;
ObjVisualization* obj_visualizer_move;

void convertPCLpc2sensorMsgs(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc,sensor_msgs::PointCloud &senorPtCloud)
{
    senorPtCloud.channels.resize(3);

    geometry_msgs::Point32  ros_point;
    for(auto &pt:pc->points)
    {
        ros_point.x = pt.x;
        ros_point.y = pt.y;
        ros_point.z = pt.z;
        senorPtCloud.points.push_back(ros_point);
    }
}


void convertPCLpc2sensorMsgs(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc, 
const std::vector<pcl::PointIndices> &cluster_indices_vector,sensor_msgs::PointCloud &ros_cloud)
{
    ros_cloud.channels.resize(2);
    ros_cloud.channels[0].name = "cluster_id";
    ros_cloud.channels[1].name = "certainty";
    geometry_msgs::Point32  ros_point;
    for (int c_index = 0; c_index < cluster_indices_vector.size();c_index++)
    {
        auto& indices = cluster_indices_vector.at(c_index);
        if(indices.indices.empty()){continue;}
        for(int p_index = 0;p_index < indices.indices.size();++p_index)
        {
            auto& point = pc->at(indices.indices.at(p_index));
            ros_point.x = point.x;
            ros_point.y = point.y;            
            ros_point.z = point.z;
            ros_cloud.points.emplace_back(ros_point);
            ros_cloud.channels[0].values.push_back(c_index);
            // ros_cloud.channels[1].values.push_back(v.intensity);
        }
    }
}




void PubTest(sensor_msgs::PointCloud& cloud, ros::Publisher& pub)
{
    cloud.header.frame_id = "rslidar";
    cloud.header.stamp = ros::Time::now();
    pub.publish(cloud);
    cloud.points.clear();
    if (cloud.channels.size() != 0)
    {
        cloud.channels.clear();
    }
}


void SubObjCallback(const msg_lidar_obj::msg_lidar_obj::ConstPtr msg,
                                              std::vector<Tracker*>& v_tracker,
                                              msg_lidar_obj::msg_lidar_obj new_obj,
                                              msg_lidar_obj::msg_lidar_obj track_obj,
                                              msg_lidar_obj::msg_lidar_obj move_obj,
                                              queue<double> &Qtime
                                              )
{
              double deltT,lastT,newT,sumT;
              newT = ros::Time::now().toSec();
               std::cout<<"ysztime    newT = "<<newT<<std::endl;
               std::cout<<"ysztime    Qtimesize() = "<<Qtime.size()<<std::endl;
               if(Qtime.size()>1)
               {
                    sumT = Qtime.back() - Qtime.front();
                    
               }
               if(Qtime.size()>0)
               {
                lastT = Qtime.back(); 
               }

               if(Qtime.size() >= 30 || sumT>1)
                    {
                        Qtime.pop();
                        std::cout<<"ysztime    pop = "<<sumT<<std::endl;
                        Qtime.push(newT);
                    }
               else
                    {
                        Qtime.push(newT);
                         std::cout<<"ysztime     push newT = "<<newT<<std::endl;
                    }
               if(Qtime.size()>1)
               {
                   deltT = newT - lastT;
                   if(deltT<0.001)deltT=0.1;
                    std::cout<<"ysztime     delt = "<<deltT<< "   newT  =  "<<newT<<"   q lastT== "<< lastT<<std::endl;
               }
               std::cout<<"ysztime     delt = "<<deltT<< "   sumt  =  "<<sumT<<"   q size== "<< Qtime.size()<<std::endl;
                std::cout<<"ysz   msg_lidar_obj input_obj size== "<<msg->lidar_obj.size()<<std::endl;
                // std_msgs::Header header = msg.header;
                // std::cout << msg.header << std::endl;
                //std::vector<std::vector<cv::Rect>> v_dets;
                std::vector<std::map<int, Track>> v_tracks;
                std::vector<std::vector<msg_box::Box>> v_result;
                msg_box::Box box;
                std::vector<TrackResult> v_tr;
                new_obj.lidar_obj.clear();
                track_obj.lidar_obj.clear();
                move_obj.lidar_obj.clear();
                const int CLASS_NUM = 1;
                for (int i = 0; i < CLASS_NUM; ++i)
                    {
                        //ysz放不同种类的目标框，每个种类分开
                        //v_dets.push_back(std::vector<cv::Rect>());
                        v_result.push_back(std::vector<msg_box::Box>());
                    }
                 for (auto o : msg->lidar_obj)
                    {
                        //std::cout << "project class: " << boxes.type << " rect: " << boxes.x << boxes.y << boxes.w << boxes.h << " sensor id: " << boxes.sensor_id << std::endl; 
                        msg_box::Box box;
                        box.x = o.gx;
                        box.y = o.gy;
                        box.z = o.gz;
                         //std::cout<<"yszoooooo    vxy = "<<box.x<<" , " << box.y<<std::endl;
                        box.l = o.length;
                        box.w = o.width;
                        box.h = o.height;
                        box.sensor_id = o.id;
                        box.time = newT;
                        //以boxes.type进行划分
                        //v_dets[0].push_back(obj);
                        v_result[0].push_back(box);
                    }

                 // 跟踪
                for (int i = 0; i < v_result.size(); ++i)
                    {
                        //v_dets[i]跟踪前的目标,是同一类型的多个目标
                        v_tracker[i]->dt = deltT;
                        v_tracker[i]->Run(v_result[i]);
                        //跟踪结果
                        v_tracks.push_back(v_tracker[i]->GetTracks());
                    }
                        msg_box::Box box_result;
                        
                        //v_tracks存放类别，size目前等于1
                        for (int i = 0; i < v_tracks.size(); ++i)
                             {
                               //v_tracks[i]每一类里的目标
                               std::cout<<"ysz v_tracks[i]"<<v_tracks[i].size()<<std::endl;
                                for (auto& trk : v_tracks[i]) 
                                        {
                                            //bbox -> Eigen::VectorXd
                                            const auto& bbox = trk.second.GetStateAsBbox();
                                            //if (trk.second.coast_cycles_ < 1 &&  (trk.second.hit_streak_ >= 5 ) && bbox[4]>0.03 || bbox[5]>0.03) 
                                            //连续预测5帧才输出
                                            //if (trk.second.coast_cycles_ < 1 &&  (trk.second.hit_streak_ >=20 )) 
                                             //连续跟踪超过0.5s , 须添加0.5s内位移
                                            if (trk.second.coast_cycles_ < 1 &&  (trk.second.tracking_time >=0.3 )) 
                                            {
                                                // Print to terminal for debugging
                                                // if(bbox[4]>0.1 || bbox[5]>0.1 && bbox[1] >-7 && bbox[1]<-1)
                                                // std::cout<<"ysz res"<<"  c=="<<bbox[0]<<" , "<<bbox[1]<<"    vxy="<<bbox[4]<<" , "<<bbox[5]<<std::endl;
                                                // std::cout<<"ysz    trk id== "<<trk.first<<std::endl;
                                                // std::cout<<"ysz    obj id== "<<trk.second.track_id<<std::endl;
                                                TrackResult tr;
                                                tr.oriid = trk.second.track_id;
                                                tr.trackid = trk.first;
                                                tr.vx = bbox[4];
                                                tr.vy = bbox[5];
                                                tr.tracking_time=bbox[6];
                                                tr.m_flag = bbox[7];
                                                v_tr.push_back(tr);
                                                // if( bbox[1] >-7 && bbox[1]<-1)
                                                // {
                                                //     std::cout<<"ysz res"<<"  c=="<<bbox[0]<<" , "<<bbox[1]<<"    vxy="<<bbox[4]<<" , "<<bbox[5]<<std::endl;
                                                //     std::cout<<"ysz    trk id== "<<trk.first<<std::endl;
                                                // }
                                            }
                                        }
                            }
                    for (auto o : msg->lidar_obj)
                        {
                                o.relative_motion = 0;
                                bool track_flag = false;
                                for(int i = 0 ; i<v_tr.size() ; i++ )
                                {
                                    if(o.id == v_tr[i].oriid)
                                    {
                                        track_flag = true;
                                        o.vx = v_tr[i].vx;
                                        o.vy = v_tr[i].vy;
                                        // std::cout<<"yszoooooo    vxy = "<<o.vx<<" , " << o.vy<<std::endl;
                                        //0不显示，所以可视化id = 跟踪id+1
                                        o.id_track = v_tr[i].trackid+1;
                                        //判断动静不能以一帧来判定，存一个目标1s内的观测位置，求最远两点，/delt，
                                        //大于2才认为是移动目标
                                        //if(fabs(v_tr[i].vx)>1 || fabs(v_tr[i].vy)>1)
                                        {
                                          //if(v_tr[i].tracking_time>0.5 &&v_tr[i].m_flag == 1)
                                           {
                                                o.relative_motion = 1;
                                                move_obj.lidar_obj.push_back(o);
                                           }  
                                        }
                                        track_obj.lidar_obj.push_back(o);
                                        //std::cout<<"yszrrrrr"<<o.id_track<<std::endl;
                                        break;
                                    }
                                }
                                if(!track_flag)
                                {
                                    new_obj.lidar_obj.push_back(o);
                                }
                        }
                        std::cout<<"yszttt new_obj size="<<new_obj.lidar_obj.size()<<std::endl;
                        std::cout<<"yszttt track_obj size="<<track_obj.lidar_obj.size()<<std::endl;
                        std::cout<<"yszttt move_obj size="<<move_obj.lidar_obj.size()<<std::endl;
                        obj_visualizer_move->ObjVisualizationMain(move_obj.lidar_obj);
}


int main(int argc,char **argv)
{

    cout << "begin track" << endl;
    ros::init(argc,argv,"n_obj_track");
    //obj_visualizer_before_track = new ObjVisualization("/rs_lidar_object_before_track");
    const int CLASS_NUM = 1;
    std::vector<Tracker*> v_tracker;
    msg_lidar_obj::msg_lidar_obj  new_obj;
    msg_lidar_obj::msg_lidar_obj  track_obj;
    msg_lidar_obj::msg_lidar_obj  move_obj;
    queue<double> Qtime;
    int a , b;
    for (int i = 0; i < CLASS_NUM; ++i)
    {
        v_tracker.push_back(new Tracker());
    }
    time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss<<std::put_time(std::localtime(&t), "%F %X");
    //创建节点句柄
    ros::NodeHandle n;
    // ring_cluster_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster", 10);    
    // org_RoI_pub = n.advertise<sensor_msgs::PointCloud>("/org_RoI", 10);
    // ring_cluster_grid_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster_grid", 10);
    // ring_cluster_hull_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster_hull", 10);
    // ring_cluster_hull_grid_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster_hull_grid", 10);
    // pub_lidar_obj_ = n.advertise<msg_lidar_obj::msg_lidar_obj>("/topic_pub_lidar_obj", 10);
    // pub_fusionmap_ = n.advertise<module4::fusionmap>("/FUSIONMAP_MSG", 10);
    // obj_visualizer_before_track->SetColor(255, 255, 255, 1);
    obj_visualizer_move = new ObjVisualization("/move_obj_after_track");
    ros::Publisher pub_lidar_obj_ = n.advertise<msg_lidar_obj::msg_lidar_obj>("/topic_pub_lidar_obj_track", 10);
    ros::Subscriber sub_obj = n.subscribe<msg_lidar_obj::msg_lidar_obj>("/topic_pub_lidar_obj", 
                                                    1,boost::bind(&SubObjCallback, _1, v_tracker,new_obj,
                                                    track_obj,move_obj,Qtime));
    obj_visualizer_move->SetColor(255, 255, 255, 0.3);
    // ros::Subscriber sub_obj = n.subscribe<msg_lidar_obj::msg_lidar_obj>("/topic_pub_lidar_obj", 
    //                                                 1,boost::bind(&SubObjCallback1, _1, a,b));
    // ros::Subscriber prep_sub_inn = n.subscribe("/topic_pub_lidar_obj",10,SubObjCallback1);
    ros::spin(); 
}
