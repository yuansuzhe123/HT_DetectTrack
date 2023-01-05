/*******************************************************************/
/*
 * @Author: Yuan
 * @Date: 2022-10-14 16:34:41
 * @Email:824100902@qq.com
 * @Description: 
 * @Beta:2.0
 */
/*******************************************************************/
// #ifndef CLASS_BOUNDING_BOX_H
// #define CLASS_BOUNDING_BOX_H

#ifndef CLASS_OBJECT_STATE_H
#define CLASS_OBJECT_STATE_H
// include personal lib.
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>


#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#include <algorithm>

#include <Eigen/Core>
#include "omp.h"
#include "msg_lidar_obj/msg_lidar_obj.h"
#include "msg_lidar_prep/msg_lidar_prepare.h"
#include "msg_common/GridPoint.h"
#include "msg_lidar_prep/LidarPoints.h"
#include "msg_lidar_shape/DetectedObject.h"
#include "rotated_iou.h"

namespace lidar
{
namespace detect
{

#define EIGEN_MPL2_ONLY

class ObjectState
{
 private:
	double CalcClosenessCriterion(const std::vector<double>& c_1,
								  const std::vector<double>& c_2);

	  double CalcVarianceCriterion(const std::vector<double>& C_1,
                                  const std::vector<double>& C_2);
	double CalcVarianceCriterion1(const std::vector<double> &C_1,
										  const std::vector<double> &C_2);
    double CalcVarianceCriterion3(const std::vector<double> &C_1,
    const std::vector<double> &C_2);
	double CalcClosenessMCriterion(const std::vector<double>& C_1,
                                  const std::vector<double>& C_2);
	//    ros::NodeHandle nh_line;
	//    ros::Publisher pub_line;
	//    ros::Subscriber sub_line;

 public:
	ObjectState() {}

	~ObjectState() {}
    bool EstimateShapeFuse(msg_lidar_obj::msg_lidar_obj& v_objs);
    bool InsideJudge(const double pointx , const double pointy , double box[]);
		double CalculateBestAngleArea(const std::vector<double>& C_1,
                                  const std::vector<double>& C_2);
	/*
	 * minimum cluster size is 2.
	 */
	// bool estimate(const std::vector<msg_common::GridPoint>& cluster,
	// 			  msg_lidar_shape::DetectedObject&			output,
	// 			  std::vector<msg_common::GridPoint>&		edge);

template <class pointType>
bool estimate1(
    const std::vector<pointType>& cluster,
    msg_lidar_shape::DetectedObject&          output,
    std::vector<pointType>&       edge,
    sensor_msgs::PointCloud& hole_msg,
    bool hole_flag)
{
    msg_lidar_prep::LidarPoints centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;
    for (const auto& lidar_point : cluster)
    {
        centroid.x += lidar_point.x;
        centroid.y += lidar_point.y;
        centroid.z += lidar_point.z;
    }
    centroid.x = centroid.x / ( double )cluster.size();
    centroid.y = centroid.y / ( double )cluster.size();
    centroid.z =centroid.z/ ( double )cluster.size();  
    output.cx = centroid.x;
    output.cy = centroid.y;
    output.cz = centroid.z;
    // Note that centroid.z isn't center.z,centroid.z
    //is height at box,not plane.
    // calc min and max z for bounding_box length
    double min_z = 0;
    double max_z = 0;
    for (size_t i = 0; i < cluster.size(); ++i)
    {
        if (cluster.at(i).z < min_z || i == 0)  //点云簇第i个点的z值
            min_z = cluster.at(i).z;
        if (max_z < cluster.at(i).z || i == 0) max_z = cluster.at(i).z;
    }

    //计算最佳角度
    std::vector<std::pair<double /*theta*/, double /*q*/>> q_vector,q_vector_acc;
    // std::vector<std::pair<double /*theta*/, double /*q*/>> max_q_vector;
    std::vector<std::pair<double /*theta*/, double /*q*/>> slid_q_vector;
    //111111111111111111111
    const double angle_step = 1.0;
    const double max_angle = M_PI / 2.0;
    //const double min_angle = M_PI / 4.0;
    //const double angle_reso = M_PI /2/180.0;
    const double angle_reso = M_PI / (angle_step*180.0);
    const double angle_reso_acc = M_PI / (2*180.0);
    const double edge_res = M_PI / (1 * 180);
    const int slid_win_num = 1/angle_step;
    const double slid_win_ang = slid_win_num*M_PI / 180.0;
    //111111111111111111111
    double temp_max_q,temp_theta_star,total_max_q,total_theta_star,acc_max_q;
    double theta_star,max_q;
    if(cluster.size()>5)
    {
        #pragma omp parallel for
        for (double theta = 0; theta < max_angle; theta += angle_reso)
        {
            Eigen::Vector2d e_1;
            e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
            Eigen::Vector2d e_2;
            e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
            std::vector<double> C_1;                   // col.5, Algo.2
            std::vector<double> C_2;                   // col.6, Algo.2
            for (const auto& point : cluster)
            {
                C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
                C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
            }
        // double q = CalcClosenessCriterion(C_1, C_2);   // col.7, Algo.2
            double q = CalcVarianceCriterion3(C_1, C_2); 
            //double q = CalcVarianceCriterion(C_1, C_2); 
            //std::cout<<"theta===   "<<theta*180.0 /M_PI <<"q====   "<<q<<std::endl;
            q_vector.push_back(std::make_pair(theta, q));  // col.8, Algo.2
        }
        for (size_t i = 0; i < q_vector.size(); ++i)
            {

                if (total_max_q < q_vector.at(i).second
                    || i == 0)  // max_q isn't given initial value,but when
                                // i==0,Q.at(0).second is max_q.
                {
                    total_max_q = q_vector.at(i).second;
                    total_theta_star = q_vector.at(i).first;
                }
                
            }
        theta_star =total_theta_star;   // col.10, Algo.2
        max_q = total_max_q;
    }
    else
    {
        theta_star = 0;
    }
//theta_star = 80*M_PI/180;
//std::cout<<"theta_star===   "<<theta_star*180.0 /M_PI <<"max_q====   "<<max_q<<std::endl;




    Eigen::Vector2d e_1_star;  // col.11, Algo.2
    Eigen::Vector2d e_2_star;
    //geometry_msgs::Polygon hole_msg;
    geometry_msgs::Point32 hole_points_1 , hole_points_2;
    e_1_star << std::cos(theta_star), std::sin(theta_star);
    e_2_star << -std::sin(theta_star), std::cos(theta_star);
    std::vector<double> C_1_star;  // col.11, Algo.2 存储X坐标
    std::vector<double> C_2_star;  // col.11, Algo.2    存储Y坐标
    for (const auto& point : cluster)
    {
        C_1_star.push_back(point.x * e_1_star.x()
                           + point.y * e_1_star.y());  // col.11, Algo.2
        C_2_star.push_back(point.x * e_2_star.x()
                           + point.y * e_2_star.y());  // col.11, Algo.2
    }
    double ell_a = 0 , ell_b = 0,cent_x = 0,cent_y = 0;
    // col.12, Algo.2
    const double min_C_1_star =
        *std::min_element(C_1_star.begin(), C_1_star.end());
    const double max_C_1_star =
        *std::max_element(C_1_star.begin(), C_1_star.end());
    const double min_C_2_star =
        *std::min_element(C_2_star.begin(), C_2_star.end());
    const double max_C_2_star =
        *std::max_element(C_2_star.begin(), C_2_star.end());
    double area_para = sqrt(2);
    if(hole_flag)
    {
        ell_a = area_para*fabs(max_C_1_star - min_C_1_star)*0.5;
        ell_b = area_para*fabs(max_C_2_star - min_C_2_star)*0.5;
        cent_x = 0.5*(max_C_1_star + min_C_1_star);
        cent_y = 0.5*(max_C_2_star + min_C_2_star);
        // std::cout<<"cent_x===  "<<cent_x<<std::endl;
        // std::cout<<"cent_y===  "<<cent_y<<std::endl;
        for(int i =0 ;i<360;i++)
        {
            double temp_x = cent_x+ell_a*std::cos(i*M_PI/180);
            double temp_y = cent_y+ell_b*std::sin(i*M_PI/180);

            hole_points_1.x = (temp_x * std::cos(theta_star)- temp_y * std::sin(theta_star));
            hole_points_1.y = (temp_x * std::sin(theta_star)+ temp_y * std::cos(theta_star));
            hole_points_1.z = 0.5*(min_z+max_z);
            //hole_msg.points.push_back(hole_points_1);
            hole_msg.header.frame_id = "my_frame";
            hole_msg.points.push_back(hole_points_1);
        }
    }
    
    //std::cout<<"ggggggggggggggggg==="<<hole_msg.polygon.points.size()<<std::endl;
    // rectangle edges is 4 sides expression: a[i]*x + b[i]*y = c[i], i=1,2,3,4
    const double a_1 = std::cos(theta_star);
    const double b_1 = std::sin(theta_star);
    const double c_1 = min_C_1_star;
    const double a_2 = -1.0 * std::sin(theta_star);
    const double b_2 = std::cos(theta_star);
    const double c_2 = min_C_2_star;
    const double a_3 = std::cos(theta_star);
    const double b_3 = std::sin(theta_star);
    const double c_3 = max_C_1_star;
    const double a_4 = -1.0 * std::sin(theta_star);
    const double b_4 = std::cos(theta_star);
    const double c_4 = max_C_2_star;

    // calc center of bounding box
    // double intersection_x_1 =
    //     (b_1 * c_2 - b_2 * c_1)
    //     / (a_2 * b_1 - a_1 * b_2);  // edge 1 & edge 2 intersection point
    // double intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
    // double intersection_x_2 =
    //     (b_3 * c_4 - b_4 * c_3)
    //     / (a_4 * b_3 - a_3 * b_4);  // edge 3 & edge 4 intersection point
    // double intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);
    // double intersection_x_1_4 =
    //     (b_1 * c_4 - b_4 * c_1)
    //     / (a_4 * b_1 - a_1 * b_4);  // edge 1 & edge 4 intersection point
    // double intersection_y_1_4 =
    //     (a_1 * c_4 - a_4 * c_1) / (a_1 * b_4 - a_4 * b_1);
    // double intersection_x_2_3 =
    //     (b_2 * c_3 - b_3 * c_2)
    //     / (a_3 * b_2 - a_2 * b_3);  // edge 2 & edge 3 intersection point
    // double intersection_y_2_3 =
    //     (a_2 * c_3 - a_3 * c_2) / (a_2 * b_3 - a_3 * b_2);
    double intersection_x_1 = min_C_1_star* std::cos(theta_star)-min_C_2_star*std::sin(theta_star);// edge 1 & edge 2 intersection point
    double intersection_y_1 = min_C_1_star* std::sin(theta_star)+min_C_2_star*std::cos(theta_star);

    double intersection_x_2 =max_C_1_star* std::cos(theta_star)-max_C_2_star*std::sin(theta_star); // edge 3 & edge 4 intersection point
    double intersection_y_2 = max_C_1_star* std::sin(theta_star)+max_C_2_star*std::cos(theta_star);

    double intersection_x_1_4 =min_C_1_star* std::cos(theta_star)-max_C_2_star*std::sin(theta_star); // edge 1 & edge 4 intersection point
    double intersection_y_1_4 =min_C_1_star* std::sin(theta_star)+max_C_2_star*std::cos(theta_star);

    double intersection_x_2_3 =max_C_1_star* std::cos(theta_star)-min_C_2_star*std::sin(theta_star);// edge 2 & edge 3 intersection point
    double intersection_y_2_3 =max_C_1_star* std::sin(theta_star)+min_C_2_star*std::cos(theta_star);

    // calculate rectangle edge intersection for lines markerArray
    geometry_msgs::Point intersect_1_2, intersect_3_4, intersect_1_4,intersect_2_3;
    intersect_1_2.x = intersection_x_1;
    intersect_1_2.y = intersection_y_1;
    intersect_1_2.z = centroid.z;
    intersect_3_4.x = intersection_x_2;
    intersect_3_4.y = intersection_y_2;
    intersect_3_4.z = centroid.z;
    intersect_1_4.x = intersection_x_1_4;
    intersect_1_4.y = intersection_y_1_4;
    intersect_1_4.z = centroid.z;
    intersect_2_3.x = intersection_x_2_3;
    intersect_2_3.y = intersection_y_2_3;
    intersect_2_3.z = centroid.z;
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/base_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "L-shape_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    //  line_list.points.push_back(intersect_1_2);
    //  line_list.points.push_back(intersect_2_3);
    //  pub_line.publish(line_list);
    output.line_list.points.push_back(intersect_1_2);
    output.line_list.points.push_back(intersect_2_3);

    // calc dimention/dimension of bounding box
    Eigen::Vector2d e_x;
    Eigen::Vector2d e_y;
    e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)),
        b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
    e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)),
        b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
    Eigen::Vector2d diagonal_vec;
    // intersection_x_1 - intersection_x_2 may be negetive.so at
    // output.dimensions.x call fabs.
    diagonal_vec << intersection_x_1 - intersection_x_2,
    intersection_y_1 - intersection_y_2;
    // calc yaw
    tf2::Quaternion quat;
    quat.setEuler(/* roll */ 0, /* pitch */ 0,
                  /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));
    /********************************************************************************************/
    //  std::cout
    //    << "*******************here is calcucating x,y,z,yaw****************"
    //    << std::endl;
    //输出左下角点
    // output.pose.position.x = intersection_x_1 ;
    // output.pose.position.y = intersection_y_1;
    //
    output.pose.position.x =  (intersection_x_1 + intersection_x_2) / 2.0;
    //  std::cout << "position.x is :" << output.pose.position.x << std::endl;
    output.pose.position.y = (intersection_y_1 + intersection_y_2) / 2.0;
    if(theta_star < M_PI*45/180)
    {
        output.x_bottom_right= intersect_1_2.x;//min_C_1_star;//min_C_1_star*e_x_star.x()+max_C_2_star*e_x_star.y();
        output.y_bottom_right =  intersect_1_2.y;//min_C_1_star*e_y_star.x()+max_C_2_star*e_y_star.y();

        output.x_bottom_left= intersect_1_4.x;
        output.y_bottom_left= intersect_1_4.y;
    }
    else
    {
        /* code */
        output.x_bottom_right= intersect_1_4.x;//min_C_1_star;//min_C_1_star*e_x_star.x()+max_C_2_star*e_x_star.y();
        output.y_bottom_right =  intersect_1_4.y;//min_C_1_star*e_y_star.x()+max_C_2_star*e_y_star.y();
        output.x_bottom_left= intersect_3_4.x;
        output.y_bottom_left= intersect_3_4.y;
    }
    output.x_bottom_cent =  output.x_bottom_left*0.5+output.x_bottom_right*0.5;
    output.y_bottom_cent=  output.y_bottom_left*0.5+output.y_bottom_right*0.5;
    //  std::cout << "position.y is :" << output.pose.position.y << std::endl;
    output.pose.position.z = 0.5*(min_z+max_z);
    //  std::cout << "position.z is :" << output.pose.position.z << std::endl;
    output.pose.orientation = tf2::toMsg(quat);
    //  std::cout << "orientation is :" << output.pose.orientation << std::endl;
    output.angle = std::atan2(e_1_star.y(), e_1_star.x());
    //  std::cout << "the yaw angle is :" << output.angle << std::endl;
    constexpr double ep = 0.001;
    // e_x.dot(diagonal_vec): both vector are e_x & diagonal_vec. dot product.
    //(a, b).dot((c, d)) = a*c + b*d. here difined the dimension of box.
    output.dimensions.x =std::fabs(e_x.dot(diagonal_vec));
    output.dimensions.y =std::fabs(e_y.dot(diagonal_vec));
    output.dimensions.z = std::max((max_z - min_z), ep);
    output.pose_reliable = true;
    //  std::cout << "output.dimensions is :" << output.dimensions << std::endl;
    // check wrong output
    if (output.dimensions.x < ep && output.dimensions.y < ep) return false;
    output.dimensions.x = std::max(output.dimensions.x, ep);
    output.dimensions.y = std::max(output.dimensions.y, ep);
    output.valid = true;
    output.pose_reliable = true;
    output.label = "truck";
    return true;
}
	

// void CalculateHullPoints(sensor_msgs::PointCloud gridcloud  , sensor_msgs::PointCloud hullclouds )
// {




// }

};
}  // namespace detect
}  // namespace lidar
#endif  // CLASS_BOUNDING_BOX_H
