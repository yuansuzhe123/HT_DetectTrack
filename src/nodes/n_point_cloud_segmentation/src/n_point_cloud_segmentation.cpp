/*******************************************************************/
/*
 * @Author: Yuan
 * @Date: 2022-10-14 16:34:41
 * @Email:824100902@qq.com
 * @Description1: 
 * @Beta:2.0
 */
/*******************************************************************/

#include <iostream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <sstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/vfh.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/normal_space.h>


#include "nanoflann.hpp"
#include "utils.h"
#include "c_object_state.h"

#include "msg_obj_perception/Obj.h"
#include "module4/fusionmap.h"
#include "msg_lidar_obj/msg_lidar_obj.h"
#include "obj_visualization.h"
#include "convex_hull_granham.h"
#include "multiagent/TargetVehicles.h"
#include "multiagent/WAYPOINT.h"
#include "multiagent/DetectedVehicle.h"
#include "multiagent/SpeedSliceInfo.h"
#include "globalplan/localpose.h"
#include "vehicle/insMsg.h"
#define InitTimer(timer)     vector<double> timer;\
    timer.push_back(ros::Time::now().toSec());

#define PrintWithTimeConsumption(timer,content)     timer.push_back(ros::Time::now().toSec());\
    std::cout << "--T-- " << content << " cost time: " << \
    (timer[timer.size()-1] - timer[timer.size()-2])*1000 << "ms" << std::endl; 
vehicle::insMsg GlobalPoseData;
globalplan::localpose LocalPoseData;
multiagent::TargetVehicles Vehiclesdata;

typedef struct POSE2_
{
   double x = 0; 
   double y = 0 ;
   double theta = M_PI/2;
};
typedef struct GPScar
{
   double x ; 
   double y ;
   double p1x,p2x,p3x,p4x;
   double p1y,p2y,p3y,p4y;
   int id;
};
std::vector<GPScar> V_GPScar;
// using namespace cv;
// using namespace pcl;
using namespace std;


void mixed_condition_euclidean_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc, const pcl::PointIndices::Ptr &indices, std::vector<pcl::PointIndices> &cluster_indices_vector)
{
    InitTimer(vecTimes)

    int nnK = 7; // n neighbors + 1 self
    double distThres = 0.5;
    int countThres = 6;
    vector<int*> flagTab(pc->size(),NULL);
    pcl::search::KdTree <pcl::PointXYZI>::Ptr  tree(new pcl::search::KdTree<pcl::PointXYZI>);
    // float resolution = 2;
    // pcl::search::Octree<pcl::PointXYZI>::Ptr tree(new pcl::search::Octree<pcl::PointXYZI>(resolution));
    tree->setInputCloud(pc);
    cout << __FUNCTION__ << "input point cloud size:" << pc->size() << endl;
    PrintWithTimeConsumption(vecTimes,"tree set input")
    // tree->addPointsFromInputCloud();
    vector<int> IdxStack; // fill ground seed in stack
    int clusterID = 0;
    int searchCnt = 0;
    for(int i = 0;i < pc->size();i++)
    {
        if(NULL != flagTab[i]) // if clustered
        {
            continue;
        }

        IdxStack.push_back(i);
        while(!IdxStack.empty())
        { 
            int searchid = IdxStack.back();
            IdxStack.pop_back();
            pcl::PointXYZI &searchPt = pc->points[searchid];

            // if new cluster
            if(NULL == flagTab[searchid])
            {
                flagTab[searchid] = new int(clusterID++);
                pcl::PointIndices tmpIndices;
                tmpIndices.indices.push_back(searchid);
                cluster_indices_vector.push_back(tmpIndices);
            }

            auto &curClusterIndices = cluster_indices_vector[*flagTab[searchid]];
            vector<int> rangIdx;
            vector<float> vecDist;
            searchCnt++;
            if(tree->nearestKSearch(searchPt,nnK,rangIdx,vecDist) > 0)
            {
                // #pragma omp parallel for
                for(int j = 0;j < rangIdx.size();j++)
                {
                    auto &idx = rangIdx[j];
                    auto &curPt = pc->at(idx);
                    // if in range 
                    if(vecDist[j] < distThres)
                    {
                        if(NULL == flagTab[idx])// if around point not be clustered,set the same shell
                        {
                            flagTab[idx] = flagTab[searchid];
                            curClusterIndices.indices.push_back(idx);
                            IdxStack.push_back(idx);
                        }
                        else if(*flagTab[idx] != *flagTab[searchid]) // don't need to merge
                        {
                            if(*flagTab[idx] > *flagTab[searchid]) // set shells the same cluster ID,the little one
                            {
                                *flagTab[idx] = *flagTab[searchid];
                            }
                            else
                            {
                                *flagTab[searchid] = *flagTab[idx];
                            }
                        }
                        else // the search point self
                        {
                            continue;
                        }
                    }

                }                
            }
        }
    }
    cout << "----- search count:" << searchCnt << endl;
    PrintWithTimeConsumption(vecTimes,"flood fill cluster")

    // merge clusters 
    for(int i = 0;i < cluster_indices_vector.size();i++)
    {
        int checkPos = cluster_indices_vector.size()-1-i;
        auto &curIds = cluster_indices_vector[checkPos].indices;
        int cluster_id = *flagTab[curIds[0]];
        auto &clusterIds = cluster_indices_vector[cluster_id].indices;
        
        if(checkPos != cluster_id)
        {
            clusterIds.insert(clusterIds.end(),curIds.begin(),curIds.end());
            curIds.clear();
            cluster_indices_vector.erase(cluster_indices_vector.begin()+checkPos);
        }
    }
    PrintWithTimeConsumption(vecTimes,"merge clusters")

    // clean empty cluster,and the cluster which's count less than countThres 
    int count = 0;
    for(int i = 0;i < cluster_indices_vector.size();i++)
    {
        int checkPos = cluster_indices_vector.size()-1-i;
        count += cluster_indices_vector[checkPos].indices.size();
        if(cluster_indices_vector[checkPos].indices.size() < countThres)
        {
            cluster_indices_vector.erase(cluster_indices_vector.begin()+checkPos);
        }
    }
    
}

bool InsideJudge(const double pointx , const double pointy , double box[])
{
    double cross_p1_p2_p = (box[2]-box[0])*(pointy-box[1])
                        -(pointx-box[0])*(box[3]-box[1]);
    double cross_p3_p4_p = (box[6]-box[4])*(pointy-box[5])
                        -(pointx-box[4])*(box[7]-box[5]);
    double cross_p2_p3_p = (box[4]-box[2])*(pointy-box[3])
                        -(pointx-box[2])*(box[5]-box[3]);
    double cross_p4_p1_p = (box[0]-box[6])*(pointy-box[7])
                        -(pointx-box[6])*(box[1]-box[7]);
    return (cross_p1_p2_p*cross_p3_p4_p >= 0 && cross_p2_p3_p*cross_p4_p1_p>=0);
}
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


void mixed_cluster( pcl::PointCloud<pcl::PointXYZI>::Ptr pc, 
std::vector<pcl::PointIndices> &cluster_indices_output,    
    int nnK , // n neighbors + 1 self
    double distThres,
    int countThres)
{
    InitTimer(vecTimes)

    std::vector<pcl::PointIndices> cluster_indices_vector;
    PointCloud<double> cloud;
    cloud.pts.resize(pc->size());    
    for(int i = 0;i < pc->size();i++)
    {
        auto &p = pc->points[i];
        cloud.pts[i].x = p.x;
        cloud.pts[i].y = p.y;
        cloud.pts[i].z = p.z;
    }

    vector<int*> flagTab(pc->size(),NULL);
    vector<int*> cleanList;

    using namespace nanoflann;
    	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<double,PointCloud<double> > ,
		PointCloud<double>,
		3 /* dim */
		> my_kd_tree_t;

    my_kd_tree_t   index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(nnK /* max leaf */) );
    index.buildIndex();
    PrintWithTimeConsumption(vecTimes,"buildIndex")

    // 剔除地面
    vector<int> seedIndx;
    seedIndx.reserve(pc->size());
    int groundType = -1;
    float maxDeltaZThres = 0.1;
    for(int i = 0;i < pc->size();i++)
    {
        int searchid = i;
        pcl::PointXYZI &searchPt = pc->points[searchid];
        std::vector<size_t>   rangIdx(nnK);
        std::vector<double> vecDist(nnK);
        const double searchPtAr[3] = {searchPt.x,searchPt.y,searchPt.z};
        if(searchPt.z < 0.3)
        //if(0)
        {
            continue;
            if(index.knnSearch(&searchPtAr[0], nnK, &rangIdx[0], &vecDist[0]))
            {
                // z轴周边变化阈值卡掉地面点
                float maxDeltaZ = 0;
                for(int j = 0;j < rangIdx.size();j++)
                {
                    auto &idx = rangIdx[j];
                    auto &curPt = pc->at(idx);
                    float DeltaZ = fabs(curPt.z-searchPt.z);
                    if(DeltaZ > maxDeltaZ)
                    {
                        maxDeltaZ = DeltaZ;
                    }
                    if(maxDeltaZ > maxDeltaZThres)
                    {   
                        break;
                    }
                }
                if(maxDeltaZ < maxDeltaZThres)
                {
                    // flagTab[idx]
                    flagTab[searchid] = &groundType;
                    // continue;
                }
                else
                {
                    seedIndx.emplace_back(i);
                }
            }
    
        }
        else
        {
            seedIndx.emplace_back(i);
        }
        
    }
    PrintWithTimeConsumption(vecTimes,"filter ground points")

    vector<int> IdxStack; // fill ground seed in stack
    int clusterID = 0;
    vector<vector<int *>> VecClusterIdUnits; // 每个聚类簇 下面的点集合 队列,for merge
    int searchCnt = 0;
    // for(int i = 0;i < pc->size();i++)
    cout << "seed count :" << seedIndx.size() << endl;
    for(auto &i : seedIndx)
    {
        if(NULL != flagTab[i] or flagTab[i] == &groundType) // if clustered
        {
            continue;
        }

        IdxStack.push_back(i);
        while(!IdxStack.empty())
        {
            int searchid = IdxStack.back();
            IdxStack.pop_back();
            pcl::PointXYZI &searchPt = pc->points[searchid];

            // if new cluster
            if(NULL == flagTab[searchid])
            {
                flagTab[searchid] = new int(clusterID++);
                cleanList.push_back(flagTab[searchid]);
                VecClusterIdUnits.push_back({flagTab[searchid]});
                pcl::PointIndices tmpIndices;
                tmpIndices.indices.push_back(searchid);
                cluster_indices_vector.push_back(tmpIndices);
            }

            double searchPtDist = sqrt(pow(searchPt.x,2)+pow(searchPt.y,2)+pow(searchPt.z,2));
            auto &curClusterIndices = cluster_indices_vector[*flagTab[searchid]];
            std::vector<size_t>   rangIdx(nnK);
            std::vector<double> vecDist(nnK);
            searchCnt++;
            const double searchPtAr[3] = {searchPt.x,searchPt.y,searchPt.z};
            if(index.knnSearch(&searchPtAr[0], nnK, &rangIdx[0], &vecDist[0]))
            {
                // #pragma omp parallel for
                for(int j = 0;j < rangIdx.size();j++)
                {
                    auto &idx = rangIdx[j];
                    auto &curPt = pc->at(idx);
                    
                    // double dynamicThres = (distThres/50.0)*searchPtDist;
                    // if(dynamicThres < distThres/2){dynamicThres=distThres/2;}
                    // if(dynamicThres > distThres*2){dynamicThres=distThres*2;}
                    double dynamicThres = 1;
                    if(searchPtDist>20)
                    {
                        dynamicThres =distThres +  distThres*(searchPtDist-20)/20;
                    }
                   dynamicThres = dynamicThres>2?2:dynamicThres;

                    // if in range 
                    if(vecDist[j] < dynamicThres && vecDist[j]!=0)
                    {
                        //std::cout<<"yszkkkk"<<vecDist[j]<<std::endl;
                        if(flagTab[idx] == &groundType)
                        {
                            continue;
                        }                        
                        else if(NULL == flagTab[idx])// if around point not be clustered,set the same shell
                        {
                            flagTab[idx] = flagTab[searchid];
                            curClusterIndices.indices.push_back(idx);
                            IdxStack.push_back(idx);
                        }
                        else if(*flagTab[idx] != *flagTab[searchid]) // don't need to merge
                        {
                            if(*flagTab[idx] > *flagTab[searchid]) // set shells the same cluster ID,the little one
                            {
                                for(auto &flagPtr:VecClusterIdUnits[*flagTab[idx]])
                                {
                                    *flagPtr = *flagTab[searchid];
                                    VecClusterIdUnits[*flagTab[searchid]].push_back(flagPtr);//merge 
                                }


                                // *flagTab[idx] = *flagTab[searchid];
                            }
                            else
                            {
                                for(auto &flagPtr:VecClusterIdUnits[*flagTab[searchid]])
                                {
                                    
                                    *flagPtr = *flagTab[idx];
                                    VecClusterIdUnits[*flagTab[idx]].push_back(flagPtr);//merge 
                                }

                                // *flagTab[searchid] = *flagTab[idx];
                            }
                        }
                        else // the search point self
                        {
                            continue;
                        }
                    }

                }                
            }
        }
    }
    cout << "----- search count:" << searchCnt << endl;
    PrintWithTimeConsumption(vecTimes,"flood fill cluster")
    index.freeIndex(index);

    groundType = cluster_indices_vector.size() + 5;
    // merge clusters 
    vector<int> invalidIdxVec;
    for(int i = 0;i < cluster_indices_vector.size();i++)
    {
        int checkPos = cluster_indices_vector.size()-1-i;
        auto &curIds = cluster_indices_vector[checkPos].indices;
        int cluster_id = *flagTab[curIds[0]];
        auto &clusterIds = cluster_indices_vector[cluster_id].indices;
        
        if(checkPos != cluster_id)
        {
            clusterIds.insert(clusterIds.end(),curIds.begin(),curIds.end());
            curIds.clear();
            invalidIdxVec.push_back(i);
            // cluster_indices_vector.erase(cluster_indices_vector.begin()+checkPos);
        }
    }
    PrintWithTimeConsumption(vecTimes,"merge clusters")

    // clean empty cluster,and the cluster which's count less than countThres 
    int count = 0;
    for(auto &cluster :cluster_indices_vector)
    {
        if(cluster.indices.size() >= countThres)
        {
            cluster_indices_output.push_back(cluster);
        }
    }

    for(auto &pt:cleanList)
    {
        delete pt;
    }
}


ros::Publisher  ring_cluster_pub;
ros::Publisher  ring_cluster_grid_pub;
ros::Publisher  vehicle_conner_pc_pub;
ros::Publisher  ring_cluster_hull_pub;
ros::Publisher  ring_cluster_hull_grid_pub;
ros::Publisher  pub_lidar_obj_;  // ROS pub obj for final results;
ros::Publisher  pub_fusionmap_;
ros::Publisher  org_RoI_pub;  // ROS pub obj for final results;
ObjVisualization* obj_visualizer_before_track;

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

inline void GlobalCoor2LocalCoor(double *transVec, double *rotateMat, double glbX, double glbY, double &lcX, double &lcY)
{
    lcX = rotateMat[0] * (glbX - transVec[0]) + rotateMat[1] * (glbY - transVec[1]);
    lcY = rotateMat[2] * (glbX - transVec[0]) + rotateMat[3] * (glbY - transVec[1]);
}


inline void LocalCoor2GlobalCoor(double *transVec, double *rotateMat, double lcX, double lcY, double &glbX, double &glbY)
{
    glbX = rotateMat[3] * lcX - rotateMat[1] * lcY + transVec[0];
    glbY = -rotateMat[2] * lcX + rotateMat[0] * lcY + transVec[1];
}


inline void ComputeMat(double selfX, double selfY, double glbCoorToLcCoorDeg, double *transVec, double *rotateMat)
{
    transVec[0] = selfX; transVec[1] = selfY;
    double rotateRad = M_PI * glbCoorToLcCoorDeg / 180.;
    rotateMat[0] =  cos(rotateRad); rotateMat[1] = sin(rotateRad);
    rotateMat[2] = -sin(rotateRad); rotateMat[3] = cos(rotateRad);
}


double uniAngle(double ang){
    if (fabs(ang) > 1e6)
        return 0;
    double R_2PI = 2 * M_PI;
    while (ang < -M_PI)
    {
        if (ang < -R_2PI)
            ang += (ang / -R_2PI) * R_2PI;
        else
            ang += R_2PI;
    }

    while (ang > M_PI)
    {
        if (ang > R_2PI)
            ang -= (ang / R_2PI) * R_2PI;
        else
            ang -= R_2PI;
    }
    double res = ang;
    return res;
}

void gaussToLocalPose(POSE2_ vehicle_gaussPos, POSE2_ vehicle_localPos, POSE2_ gaussPosPt, POSE2_ &localPosPt){
    double c1 = cos(vehicle_gaussPos.theta);
    double s1 = sin(vehicle_gaussPos.theta);
    double c2 = cos(vehicle_localPos.theta);
    double s2 = sin(vehicle_localPos.theta);
    localPosPt.x = gaussPosPt.x - vehicle_gaussPos.x;
    localPosPt.y = gaussPosPt.y - vehicle_gaussPos.y;
    double temp_x = localPosPt.x * c1 + localPosPt.y * s1;
    double temp_y = localPosPt.y * c1 - localPosPt.x * s1;
    localPosPt.x = temp_x * c2 - temp_y * s2 + vehicle_localPos.x;
    localPosPt.y = temp_x * s2 + temp_y * c2 + vehicle_localPos.y;
}

//VehiclesCallback
void CallbackGlobePos(const vehicle::insMsg msg)
{
    GlobalPoseData = msg;
}

void CallbackLocalPos(const globalplan::localpose msg)
{
    LocalPoseData = msg;
}

void VehiclesCallback(const multiagent::TargetVehicles input)
{
    std::cout<<"VehiclesCallback   "<<std::endl;
    POSE2_ vehicle_gaussPos; //本车gauss坐标
    double imuToFront = 3.83;
    double imuToBack =  0.87;
    double ve_length =4.7;
    double ve_halflength =2.35;
    double ve_width = 2.0;
    double ve_halfwidth = 1.0;
    vehicle_gaussPos.x = 0.01 * GlobalPoseData.gaussPos[0];
    vehicle_gaussPos.y = 0.01 * GlobalPoseData.gaussPos[1];
    vehicle_gaussPos.theta = uniAngle(0.01 * GlobalPoseData.azimuth * M_PI / 180);
    POSE2_ vehicle_localPos;  //本车localpose坐标
    vehicle_localPos.x = LocalPoseData.dr_x * 0.01;
    vehicle_localPos.y = LocalPoseData.dr_y * 0.01;
    vehicle_localPos.theta = uniAngle(LocalPoseData.dr_heading * 0.01 * M_PI / 180);
    int index = 0;

    


    if(V_GPScar.size()>0)V_GPScar.clear();
    for(int i = 0 ; i < 4 ; i ++)
    {
        POSE2_ gaussPosPt;
        gaussPosPt.x = input.otherVehicle[i].gpsX * 0.01;        
        gaussPosPt.y = input.otherVehicle[i].gpsY * 0.01;
        POSE2_ localPosPt;
        gaussToLocalPose(vehicle_gaussPos, vehicle_localPos, gaussPosPt, localPosPt);

        std::cout<<"id    "<<int(input.otherVehicle[i].id)<<std::endl;
        std::cout<<"x y     "<<std::to_string(input.otherVehicle[i].gpsX)<<"  "<<std::to_string(input.otherVehicle[i].gpsY)<<std::endl;
        std::cout<<"heading     "<<input.otherVehicle[i].heading<<std::endl;
        std::cout<<"speed   "<<input.otherVehicle[i].speed<<std::endl;
        double transVecL[2], rotateMatL[4];
        gaussToLocalPose(vehicle_gaussPos, vehicle_localPos, gaussPosPt, localPosPt);

        double centerx = localPosPt.x;
        double centery = localPosPt.y;
        double lcx1,lcy1,glbX1, glbY1;
        double lcx2,lcy2,glbX2, glbY2;
        double lcx3,lcy3,glbX3, glbY3;
        double lcx4,lcy4,glbX4, glbY4;
        double area_compen = 1.2;
        double theta = LocalPoseData.dr_heading*0.01*M_PI  /180;
        glbX1 = centerx+ area_compen*imuToFront * cos(theta) - area_compen*ve_halfwidth * sin(theta);
        glbY1 = centery+ area_compen*imuToFront * sin(theta)  + area_compen*ve_halfwidth * cos(theta);
        glbX2 = centerx+ area_compen*imuToFront * cos(theta) + area_compen*ve_halfwidth * sin(theta);
        glbY2 = centery+ area_compen*imuToFront * sin(theta)  - area_compen*ve_halfwidth * cos(theta);
        glbX3 = centerx - area_compen*imuToBack  * cos(theta) + area_compen*ve_halfwidth * sin(theta);
        glbY3= centery - area_compen*imuToBack  * sin(theta)  - area_compen*ve_halfwidth * cos(theta);
        glbX4 = centerx - area_compen*imuToBack  * cos(theta) - area_compen*ve_halfwidth * sin(theta);
        glbY4 = centery - area_compen*imuToBack  * sin(theta) + area_compen*ve_halfwidth * cos(theta);

        // ComputeMat(LocalPoseData.dr_x*0.01, LocalPoseData.dr_y*0.01, LocalPoseData.dr_heading*0.01 - 90., transVecL, rotateMatL);
        ComputeMat(LocalPoseData.dr_x*0.01, LocalPoseData.dr_y*0.01, LocalPoseData.dr_heading*0.01 - 90, transVecL, rotateMatL);
        double lcx0,lcy0;
        GlobalCoor2LocalCoor(transVecL, rotateMatL, centerx, centery, lcx0, lcy0);
        GlobalCoor2LocalCoor(transVecL, rotateMatL, glbX1, glbY1, lcx1, lcy1);
        GlobalCoor2LocalCoor(transVecL, rotateMatL, glbX2, glbY2, lcx2, lcy2);
        GlobalCoor2LocalCoor(transVecL, rotateMatL, glbX3, glbY3, lcx3, lcy3);
        GlobalCoor2LocalCoor(transVecL, rotateMatL, glbX4, glbY4, lcx4, lcy4);

        //GlobalCoor2LocalCoor(transVecL, rotateMatL,  mapEntity->vehicleObj[i].center.x,  mapEntity->vehicleObj[i].center.y, lcX, lcY);
         std::cout<<"Local x y     "<<(lcx0)<<"  "<<(lcy0)<<std::endl;
         GPScar gpscar;
         gpscar.id = int(input.otherVehicle[i].id);
         gpscar.x = lcy0;
         gpscar.y = -lcx0;
         gpscar.p1x =lcy1; 
         gpscar.p1y =-lcx1;
         gpscar.p2x =lcy2; 
         gpscar.p2y =-lcx2;
         gpscar.p3x =lcy3; 
         gpscar.p3y =-lcx3;
         gpscar.p4x =lcy4; 
         gpscar.p4y =-lcx4;
        V_GPScar.push_back(gpscar);
    }
}
void HandlePcdCallback(const sensor_msgs::PointCloud2ConstPtr input)
{
    InitTimer(vecTimes)
    // pcl::PCLPointCloud2 pclPc2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd0(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZI>);   
    pcl::PointCloud<pcl::PointXYZI>::Ptr org_roi(new pcl::PointCloud<pcl::PointXYZI>);  
    pcl::PointCloud<pcl::PointXYZI>::Ptr org_clouds(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_groundori_clouds(new pcl::PointCloud<pcl::PointXYZI>);  
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_clouds(new pcl::PointCloud<pcl::PointXYZI>);  
    pcl::fromROSMsg(*input,*pcd0);
    PrintWithTimeConsumption(vecTimes,"convert fromROSMsg")
    // pcl::copyPointCloud(pcd0,*pcd);       

    pcd->reserve(pcd0->size());

    //提取ROI区域点
    for(int i=pcd0->size()-1;i > 0;i--)
    {
        if(!(isnan(pcd0->points[i].x) or isnan(pcd0->points[i].y) or isnan(pcd0->points[i].z)))
        {
            auto &pt = pcd0->at(i);
            pcl::PointXYZI tmpPt;
            // 以下标定参数为五号车
            double par_imu = 3.7;
            // //51-1
            // tmpPt.x = pt.x * 0.99989 + pt.y *0 -pt.z * 0.0148346+par_imu;
            // tmpPt.y = -pt.x * 0.000362443 + pt.y * 0.999701  -pt.z *0.0244295;
            // tmpPt.z = pt.x * 0.0148302 + pt.y *0.0244322 + pt.z * 0.999592 + 1.58;
            //51-3
            tmpPt.x = pt.x * 0.999936 + pt.y *0 +pt.z * 0.0113444+par_imu;
            tmpPt.y = pt.x * 0.000227681 + pt.y * 0.999799  -pt.z *0.0200686;
            tmpPt.z = -pt.x * 0.0113421 + pt.y *0.0200699 + pt.z * 0.999734 + 1.6;
            // tmpPt.x = pt.x * 1 + pt.y *0 - pt.z * 0+par_imu;
            // tmpPt.y = -pt.x * 0 + pt.y * 1  -pt.z *0;
            // tmpPt.z = pt.x * 0 + pt.y *0 + pt.z * 1 + 1.54;
            // tmpPt.x = pt.x ;
            // tmpPt.y = pt.y ;
            // tmpPt.z =  pt.z * 0.999656 + 2.16;
            // ROI： 潜在地面点提取
            //csc
           // if(tmpPt.y<-25 || tmpPt.y>25 || tmpPt.x>80-par_imu|| tmpPt.z>3 || tmpPt.x<-20-par_imu)continue;
           if(tmpPt.y<-25 || tmpPt.y>25 || tmpPt.x>80|| tmpPt.z>1.8 || tmpPt.x<-20)continue;
            {
                //if((tmpPt.x<0.2 +par_imu&& tmpPt.x>-4.5 +par_imu&& fabs(tmpPt.y) <1) )
                if((tmpPt.x<0.5 +par_imu&& tmpPt.x>-5 +par_imu&& fabs(tmpPt.y) <1.2) )
                continue;
            }
            org_clouds->points.emplace_back(tmpPt);
            org_roi->points.emplace_back(tmpPt);
            //pcd->points.emplace_back(tmpPt);
            // cout << "not a number !!!" <<  i << endl;
            // pcd->points.erase(pcd->points.begin()+i);
        }
    }
std::cout<<"ysz org_roi_clouds size== "<<org_roi->points.size()<<std::endl;
//提取潜在地面点
    for(int i=org_roi->size()-1;i > 0;i--)
    {
        if(!(isnan(org_roi->points[i].x) or isnan(org_roi->points[i].y) or isnan(org_roi->points[i].z)))
        {
            auto &pt = org_roi->at(i);
            pcl::PointXYZI tmpPt;
            tmpPt.x = pt.x ;
            tmpPt.y = pt.y ;
            tmpPt.z = pt.z ;
            if( tmpPt.z>0.2)
            {
                continue;
            }

            p_groundori_clouds->points.emplace_back(tmpPt);
            
        }
    }
    //计算地平面
    double grounddata[4];
    bool ground_normal_flag = true;
    std::cout<<"ysz p_groundori_clouds size== "<<p_groundori_clouds->points.size()<<std::endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        {
            pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.1);
            seg.setInputCloud(p_groundori_clouds);
            seg.segment(*ground_indices,*coefficients);
            for(int i = 0 ; i<4 ; ++i)
                {
                    grounddata[i] = coefficients->values[i];
                }
              std::cerr << "ysz coefficients: " << grounddata[0] << " "
                << grounddata[1] << " "
                << grounddata[2] << " "
                << grounddata[3] << std::endl;
             //判定地面是否正常
            if(grounddata[2]>0.99)
                {
                    std::cout<<"ysz ground is horizontal and normal"<<std::endl;
                }
             else if(grounddata[2]>0.85)
                {
                    std::cout<<"ysz ground is not  horizontal but normal"<<std::endl;
                }
            else
                {
                        std::cout<<"ysz ground is unnormal"<<std::endl;
                        ground_normal_flag = false;
                }
        }
    
//滤除地面点
int sum_groud_points = 0;
 for(int i=org_roi->size()-1;i > 0;i--)
    {
        if(!(isnan(org_roi->points[i].x) or isnan(org_roi->points[i].y) or isnan(org_roi->points[i].z)))
        {
           
            auto &pt = org_roi->at(i);
            pcl::PointXYZI tmpPt;
            tmpPt.x = pt.x ;
            tmpPt.y = pt.y ;
            tmpPt.z = pt.z ;
            double plane_slope = sqrt(pow(grounddata[0],2)+pow(grounddata[1],2)+pow(grounddata[2],2));
            double dist = fabs(tmpPt.x*grounddata[0]+tmpPt.y*grounddata[1]+tmpPt.z*grounddata[2]+grounddata[3])/plane_slope;
            double point_dis = sqrt(pow(tmpPt.x,2)+pow(tmpPt.y,2));
            double height_thres = 0.1+0.006* point_dis;
            if(ground_normal_flag)
                {
                    if(dist<height_thres || tmpPt.z <0.15)
                        {
                            //26为目前车辆雷达安装能打到的最远地面点
                            if(point_dis<26)
                            sum_groud_points++;
                            continue;
                        }
                }
            else
                {
                
                    if(tmpPt.z<height_thres)
                    {
                        continue;  
                          sum_groud_points++;
                    }
                }
            

            
            pcd->points.emplace_back(tmpPt);
            // cout << "not a number !!!" <<  i << endl;
            // pcd->points.erase(pcd->points.begin()+i);
        }
    }
    std::cout<<"ysz sum ground points ="<< sum_groud_points<<std::endl;
    PrintWithTimeConsumption(vecTimes,"Filter nan points")
    sensor_msgs::PointCloud orgCloud;
    convertPCLpc2sensorMsgs(org_clouds,orgCloud);
    org_clouds.reset();
    PubTest(orgCloud,org_RoI_pub);
    //voxel
    if(0)
        {
            std::cout << "--"<<__LINE__ <<"-----before voxel PC data size :" << pcd->size() << endl;
            pcl::VoxelGrid<pcl::PointXYZI> sor0;
            sor0.setInputCloud(pcd);
            // lt float leafSize0 = 0.14;
            float leafSize0 = 0.05;
            sor0.setLeafSize(leafSize0,leafSize0,leafSize0);
            sor0.setFilterLimits(0,1);
            sor0.filter(*pcd);
            PrintWithTimeConsumption(vecTimes,"VoxelGrid")
            std::cout << "--"<<__LINE__ <<"-----pc0_filtered size :" << pcd->size() << endl;
        }
    // pcl::fromROSMsg(*input,*pcd);
    cout << "recieve pcd size: " << pcd->size() << endl;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    // std::vector<pcl::PointIndices> cluster_indices_vector;
    std::vector<pcl::PointIndices> cluster_indices_vector;
    mixed_cluster(pcd, cluster_indices_vector,20,1.5,10);
    //mixed_cluster(pcd, cluster_indices_vector,10,1.0,3);
    sensor_msgs::PointCloud clusterCloud;
    convertPCLpc2sensorMsgs(pcd,cluster_indices_vector,clusterCloud);
    PubTest(clusterCloud,ring_cluster_pub);
    PrintWithTimeConsumption(vecTimes,"mixed_cluster")
    msg_lidar_obj::msg_lidar_obj ret;
    module4::fusionmap fm;
    //module4::EntityMap em;
    msg_lidar_obj::msg_lidar_obj v_objs;
    ret.time.sec = input->header.stamp.sec;
    ret.time.msec = input->header.stamp.nsec/1000000;    
    cout << "time stamp: " << ret.time.sec << " s " << ret.time.msec << " ms" << endl;

    struct timeval tv;
    gettimeofday(&tv,NULL);
    printf("system second:%ld\n",tv.tv_sec); 
    printf("millisecond:%ld\n",tv.tv_sec*1000 + tv.tv_usec/1000); 

    // estimate state
    std::vector<msg_lidar_shape::DetectedObject> vec_lidar_shape;
    lidar::detect::ObjectState      box_model;
    algorithm::convex::ConvexHull hull_cal;
    int num_of_obj = 0;
    sensor_msgs::PointCloud gridcloud  , hullcloud , gridhullpoint;
    int colori=1;
    sensor_msgs::PointCloud vehicle_conner_pc;
    for(int k = 0 ; k < V_GPScar.size() ; k ++)
                {
                        geometry_msgs::Point32  ros_point;
                        ros_point.x = V_GPScar[k].p1x;
                        ros_point.y= V_GPScar[k].p1y;
                        ros_point.z = 0;
                        vehicle_conner_pc.points.push_back(ros_point);
                        ros_point.x = V_GPScar[k].p2x;
                        ros_point.y= V_GPScar[k].p2y;
                        ros_point.z = 0;
                        vehicle_conner_pc.points.push_back(ros_point);
                        ros_point.x = V_GPScar[k].p3x;
                        ros_point.y = V_GPScar[k].p3y;
                        ros_point.z = 0;
                        vehicle_conner_pc.points.push_back(ros_point);
                        ros_point.x = V_GPScar[k].p4x;
                        ros_point.y= V_GPScar[k].p4y;
                        ros_point.z = 0;
                        vehicle_conner_pc.points.push_back(ros_point);
                        
                }
    for(int i = 0;i < cluster_indices_vector.size();i++)
    {
        msg_obj_perception::Obj obj;
        // module4::Dynamic_Object d_obj;
        // module4::Pedestrian_Object p_obj;
        num_of_obj++;
        obj.id = num_of_obj;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointIndices::Ptr tmpInds(new pcl::PointIndices);
        tmpInds->indices.swap(cluster_indices_vector[i].indices);

        {      
            pcl::ExtractIndices<pcl::PointXYZI> extract;   //点提取对象
            extract.setInputCloud(pcd);
            extract.setIndices(tmpInds);
            extract.setNegative(false);//设置成true是保存滤波后剩余的点，false是保存在区域内的点,
            extract.filter(*cloud_temp);     
        }

        vector<pcl::PointXYZI> _edge;
        vector<pcl::PointXYZI> contour_point;
        contour_point.insert(contour_point.begin(), cloud_temp->points.begin(),cloud_temp->points.end());
        sensor_msgs::PointCloud hole_msg;
        msg_lidar_shape::DetectedObject shape_output;
        // for(int i = 0 ; i < contour_point.size() ; i++)
        // {
        //     std::cout<<"yszkkk  x=   "<<contour_point[i].x <<"  "<< contour_point[i].y<<"   "<<contour_point[i].z<<std::endl;
        // }
        box_model.estimate1(contour_point, shape_output, _edge,hole_msg,false);

        vec_lidar_shape.push_back(shape_output);

        // pull info into obj
        {
            obj.x = shape_output.pose.position.x;
            obj.y = shape_output.pose.position.y;
            obj.z = shape_output.pose.position.z;
            obj.cx = shape_output.cx;
            obj.cy = shape_output.cy;
            obj.cz = shape_output.cz;
            obj.orientation = shape_output.angle * 180 / M_PI;

            //        obj.absolute_motion =
            //        std::sqrt(std::pow(obj.x, 2) + std::pow(obj.y, 2));

            obj.length = static_cast<float>(shape_output.dimensions.x);
            obj.width = static_cast<float>(shape_output.dimensions.y);
            obj.height = static_cast<float>(shape_output.dimensions.z); 

            // if(obj.height <0.3  &&  (obj.z+0.5*obj.height) < 0.2)continue;
            // if((obj.z-0.5*obj.height>0.8  &&  obj.length+obj.width<5)|| obj.height<0.4 || obj.z-0.5*obj.height>1)continue;
            // if(obj.z +0.5*obj.height <1   &&   obj.length+obj.width+ obj.length < 4)continue;

            double dis_obj= sqrt(obj.x*obj.x + obj.y*obj.y);
            if(dis_obj>40)continue;
            if(dis_obj<26)
            {
                if(obj.height <0.3  &&  (obj.z+0.5*obj.height) < 0.2)continue;
                if(obj.z-0.5*obj.height>1.0    )continue;
                //if(obj.z +0.5*obj.height <0.5   &&   obj.length+obj.width+ obj.height < 4)continue;
            }
            else
            {                    
                if(contour_point.size()<15   &&   (obj.length+obj.width+ obj.height < 3 || obj.height<0.5))continue;
            }
            //std::cout<<"yszmmm"<<contour_point.size()<<"   xy=   "<< obj.x<<"  ,   "<<obj.y<<std::endl;
            //std::cout<<"yszmmm"<<contour_point.size()<<"   xy=   "<< obj.x<<"  ,   "<<obj.y<<std::endl;
            // set all origin points !!!
            obj.contour_point.reserve(cloud_temp->size());
            obj.raw_points.reserve(cloud_temp->size());
            msg_common::GridPoint grid;
            for(auto pt:cloud_temp->points)
            {
                grid.x = pt.x;
                grid.y = pt.y;
                grid.z = pt.z;
                
                obj.contour_point.push_back(grid);
                obj.raw_points.push_back(grid);              
            }
            /////////////////////////////////test
            double transVec[2], rotateMat[4]; 
            double gaussPos_0 = 0.01 * GlobalPoseData.gaussPos[0];
            double gaussPos_1 = 0.01 * GlobalPoseData.gaussPos[1];
            //double azimuth_temp = 0.01 * GlobalPoseData.azimuth-90;
            double gaussPos_a = uniAngle(0.01 * GlobalPoseData.azimuth * M_PI / 180);
            gaussPos_a = gaussPos_a*180/M_PI;
            // double gaussPos_0 =LocalPoseData.dr_x*0.01;
            // double gaussPos_1 = LocalPoseData.dr_y*0.01;
            // double gaussPos_a =  LocalPoseData.dr_heading*0.01 - 90;
           // std::cout<<"yszuuuuuu"<<"     gpsxy = "<<std::to_string(gaussPos_0)<<" , "<<std::to_string(gaussPos_1)<<std::endl;
            //std::cout<<"yszuuuuuu"<<"     uniAngle = "<<uniAngle(0.01 * GlobalPoseData.azimuth * M_PI / 180)<<std::endl;
            //ComputeMat(LocalPoseData.dr_x*0.01, LocalPoseData.dr_y*0.01, LocalPoseData.dr_heading*0.01 - 90, transVec, rotateMat);
            ComputeMat(gaussPos_0, gaussPos_1, gaussPos_a, transVec, rotateMat);
            double glbX, glbY;
            LocalCoor2GlobalCoor(transVec, rotateMat, obj.cx, obj.cy, glbX, glbY);
            obj.gx = glbX;
            obj.gy = glbY;
            obj.gz = 0;
            // if(obj.y<-7)
            // {
            //         std::cout<<"yszuuuuuu"<<"     V coor xy before = "<<std::to_string(obj.cx)<<" , "<<std::to_string(obj.cx)<<std::endl;
            //         std::cout<<"yszuuuuuu"<<"     G xy = "<<std::to_string(glbX)<<" , "<<std::to_string(glbY)<<std::endl;
            //         // double transVecL[2], rotateMatL[4];
            //         // ComputeMat(-gaussPos_0, -gaussPos_1, -gaussPos_a, transVecL, rotateMatL);
            //         double V_X, V_Y;
            //         //LocalCoor2GlobalCoor(transVecL, rotateMatL, glbX, glbY, V_X, V_Y);
            //         GlobalCoor2LocalCoor(transVec, rotateMat, glbX, glbY, V_X, V_Y);
            //         //GlobalCoor2LocalCoor(transVecL, rotateMatL, centerx, centery, lcx0, lcy0);
            //         std::cout<<"yszuuuuuu"<<"     V coor xy after = "<<std::to_string(V_X)<<" , "<<std::to_string(V_Y)<<std::endl; 
            // }
            //std::cout<<"yszlll"<<obj.contour_point.size()<<std::endl;
            //栅格化obj点集
            pcl::PointXYZI grid2;
            pcl::PointCloud<pcl::PointXYZI>::Ptr grid2_pointclouds(new pcl::PointCloud<pcl::PointXYZI>);
            bool temp_flag = true;
             int tempk=0;
            for(auto pt:cloud_temp->points)
            {
                //std::cout<<"ysz111 pt.x== "<< pt.x << "pt.y= "<<pt.y<<std::endl;
                grid2.x = round(pt.x*5);
                grid2.y = round(pt.y*5);
                 //std::cout<<"ysz111 grid2.x== "<< grid2.x << "grid2.y= "<<grid2.y<<std::endl;
                grid2.z = 0;
               // std::cout<<"grid2.x== "<< grid2.x << "grid2.y= "<<grid2.y <<std::endl;
                if(temp_flag)
                    {
                        
                         grid2_pointclouds->points.push_back(grid2);
                         temp_flag = false;
                    }
                else{
                        bool temp_flag2 = true;
                        for(auto pt2: grid2_pointclouds->points)
                            {
                                // std::cout<<"ysz 44444  pt2.x== "<< pt2.x << "pt2.y= "<<pt2.y<<std::endl;
                                // std::cout<<"ysz 44444  grid2.x== "<< grid2.x << "grid2.y= "<<grid2.y<<std::endl;
                                if(grid2.x ==  pt2.x && grid2.y ==  pt2.y)
                                {
                                    //std::cout<<"ysz2222222删除一个点 "<<grid2.y<<std::endl;
                                    temp_flag2 = false;
                                    break;
                                }
                            }
                            if(temp_flag2 )
                            {
                                  grid2_pointclouds->points.push_back(grid2);
                            }
                }
                
            }
           // std::cout<<"ysz555 before grid size == "<<cloud_temp->points.size() <<"  after = "<<grid2_pointclouds->points.size()<<std::endl;
            bool vehicleflag = false;
            if(obj.length + obj.width > 5  || obj.length >1.5 || obj.width>1.5)
            {
                double compeny =  cos(std::atan2(obj.x , obj.y))*0.8;
                double compenx =  sin(std::atan2(obj.x , obj.y))*1.9-1.5;
                //std::cout<<"yszuuuu  com"<<compenx <<" ,  " << compeny<<std::endl;
                //std::cout<<"yszuuuu  obj"<<obj.x+compenx <<" ,  " << obj.y+compeny<<std::endl;

                double min_dis = 9999;
               // std::cout<<"yszuuuu  V_GPScar"<<V_GPScar.size()<<std::endl;
                for(int k = 0 ; k < V_GPScar.size() ; k ++)
                {
                      //std::cout<<"yszuuuu  gps"<<V_GPScar[k].x <<" ,  " << V_GPScar[k].y<<std::endl;
                      double dis_2ve = sqrt((V_GPScar[k].x - obj.x - compenx)*(V_GPScar[k].x - obj.x - compenx)    +   (V_GPScar[k].y - obj.y - compeny)*(V_GPScar[k].y - obj.y - compeny)) ;
                      if(dis_2ve<min_dis)
                      {
                        min_dis = dis_2ve;
                      }
                }

                 // std::cout<<"yszuuuu  mindis"<<min_dis<<std::endl;
                    if(min_dis <2.5)
                    {
                                vehicleflag = true;
                    }
            }
            
            {
                for(int k = 0 ; k < V_GPScar.size() ; k ++)
                    {
                            double gps_ve_box[8]={V_GPScar[k].p1x,V_GPScar[k].p1y,V_GPScar[k].p2x,V_GPScar[k].p2y,
                            V_GPScar[k].p3x,V_GPScar[k].p3y,V_GPScar[k].p4x,V_GPScar[k].p4y};
                            if(fabs(obj.x+1) <1 &&fabs(obj.y+5) <1 )
                            {
                                std::cout<<"id=  "<< V_GPScar[k].id<<"center"<<V_GPScar[k].x<<"   "<<V_GPScar[k].y<<std::endl;
                                std::cout<<V_GPScar[k].p1x<<"   "<<V_GPScar[k].p1y<<std::endl;
                                std::cout<<V_GPScar[k].p2x<<"   "<<V_GPScar[k].p2y<<std::endl;
                                std::cout<<V_GPScar[k].p3x<<"   "<<V_GPScar[k].p3y<<std::endl;
                                std::cout<<V_GPScar[k].p4x<<"   "<<V_GPScar[k].p4y<<std::endl;
                            }
                            if(InsideJudge(obj.x , obj.y , gps_ve_box))
                            {
                                vehicleflag = true;
                                break;
                            }
                    }

                        
            }
            {
                msg_common::GridPoint grid;
                geometry_msgs::Point32  ros_point;
                if(!vehicleflag)
                {
                        for(auto pt4:grid2_pointclouds->points)
                        {
                            grid.x = pt4.x/5.0;
                            grid.y = pt4.y/5.0;
                            grid.z = pt4.z/5.0;
                            ros_point.x = pt4.x/5.0;
                            ros_point.y = pt4.y/5.0;
                            ros_point.z = pt4.z/5.0;
                            int obj_color = (i*17)%255; 
                            colori++;
                            gridcloud.points.push_back(ros_point);
                            gridcloud.channels.resize(1);
                            gridcloud.channels[0].name = "cluster_id";
                            gridcloud.channels[0].values.push_back(obj_color);
                            obj.grid_points.push_back(grid);
                        }
                }
                
            }
            

            //计算凸包点
           //hull_cal.GetConvetHullPts(obj );   
            geometry_msgs::Point32  ros_point2;
            // int size_map = sizeof(fm.positive_map);
            // //std::cout<<"yszer"<<size_map<<std::endl;
            // for(int i = 0 ; i<size_map ; i++)
            // {
            //     std::cout<<"yszer"<<int(fm.positive_map[i])<<std::endl;
            // }
            if(!vehicleflag)
                {
                    for(auto pt:obj.grid_points)
                        {
                            ros_point2.x = pt.x;
                            ros_point2.y = pt.y;
                            ros_point2.z = pt.z;
                            int obj_color = (i*17)%255; 
                            colori++;
                            hullcloud.points.push_back(ros_point2);
                            hullcloud.channels.resize(1);
                            hullcloud.channels[0].name = "cluster_id";
                            hullcloud.channels[0].values.push_back(obj_color);

                            int u_map = pt.x/0.2;
                            int v_map = pt.y/0.2;
                            u_map = 400-u_map;
                            if(u_map>499)u_map = 499;
                            if(u_map<0)u_map = 0;
                            v_map = 125-v_map;
                            if(v_map>250)u_map = 250;
                            if(v_map<0)u_map = 0;
                            int position = u_map*250 + v_map;
                            if(position<0 || position>125000)continue;
                            fm.positive_map[position] = uint8_t(12);
                            //std::cout<<"yszuv="<< u_map << " , "<<v_map<<std::endl;
                        }
                 }
            
            

        }
        v_objs.lidar_obj.push_back(obj);
    }
    PubTest(vehicle_conner_pc,vehicle_conner_pc_pub);
    PubTest(gridcloud,ring_cluster_grid_pub);
    PubTest(hullcloud,ring_cluster_hull_pub);
    PubTest(gridhullpoint,ring_cluster_hull_grid_pub);
    //box_model.EstimateShapeFuse(v_objs);
    for (auto& obj_temp : v_objs.lidar_obj)
    {
        //std::cout << "v_objs.lidar_obj======"<<obj_3.id << std::endl;
        if(obj_temp.id != 0 )
            {   

                ret.lidar_obj.push_back(obj_temp);
                
            }
    }
    PrintWithTimeConsumption(vecTimes,"estimate1 state")
 
    obj_visualizer_before_track->ObjVisualizationMain(ret.lidar_obj);
    // todo . publish msg_lidar_obj
    pub_lidar_obj_.publish(ret);
    pub_fusionmap_.publish(fm);

    std::cout << "All  segmentationAndClusteringPcl processing cost time: " << 
    (ros::Time::now().toSec() - vecTimes[0])*1000 << "ms" << std::endl;
}


int main(int argc,char **argv)
    {
        cout << "begin" << endl;
        ros::init(argc,argv,"n_point_cloud_segmentation");
        obj_visualizer_before_track = new ObjVisualization("/rs_lidar_object_before_track");
        //创建节点句柄
        ros::NodeHandle n;
        ring_cluster_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster", 10);    
        org_RoI_pub = n.advertise<sensor_msgs::PointCloud>("/org_RoI", 10);
        ring_cluster_grid_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster_grid", 10);
        ring_cluster_hull_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster_hull", 10);
        ring_cluster_hull_grid_pub = n.advertise<sensor_msgs::PointCloud>("/ring_cluster_hull_grid", 10);
        vehicle_conner_pc_pub= n.advertise<sensor_msgs::PointCloud>("/vehicle_conner_pc_pub", 10);
        pub_lidar_obj_ = n.advertise<msg_lidar_obj::msg_lidar_obj>("/topic_pub_lidar_obj", 10);
        pub_fusionmap_ = n.advertise<module4::fusionmap>("/FUSIONMAP_MSG", 10);
        obj_visualizer_before_track->SetColor(255, 255, 255, 1);
        ros::Subscriber sub_gps_vehicles = n.subscribe("/TARGETVEHICLES",10,VehiclesCallback);
        ros::Subscriber prep_sub_inn = n.subscribe("/rslidar_points",10,HandlePcdCallback);
        ros::Subscriber SubGlobePos = n.subscribe("/insMsg",10,CallbackGlobePos);
        ros::Subscriber SubLocalPos = n.subscribe("/LocalPosData",10,CallbackLocalPos);
        ros::spin(); 
    }
