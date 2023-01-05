/*******************************************************************/
/*
 * @Author: Yuan
 * @Date: 2022-10-14 16:34:41
 * @Email:824100902@qq.com
 * @Description: 
 * @Beta:2.0
 */
/*******************************************************************/

#ifndef CLASS_OBJ_VISUALIZATION_H
#define CLASS_OBJ_VISUALIZATION_H

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "msg_lidar_shape/DetectedObject.h"
#include "msg_lidar_shape/DetectedObjectArray.h"

#include "msg_obj_perception/Obj.h"
#include "msg_obj/Obj.h"

#define __APP_NAME__ "visualize_detected_objects"

enum VisualClassfication
{
    UNKNOWN_SMALL,  //未知小障碍物
    CAR,            //汽车
    TRUCK,          //卡车
    PEDESTRIAN,     //行人
    MOTOR_BIKE,     //摩托车
    BIKE,           //自行车
    UNKNOWN_BIG,    //未知大障碍物
    UNCLASSIFIED    //未分类
};

class ObjVisualization
{

 public:
    ObjVisualization(const std::string& topic_name);

    void ObjVisualizationMain(
        const msg_lidar_shape::DetectedObjectArray& in_objects);

    void ObjVisualizationMain(const std::vector<msg_obj::Obj>& in_objects);
    void ObjVisualizationMain(const std::vector<msg_obj_perception::Obj> &in_objects);

    void SetColor(float r, float g, float b, float a);

    void SetDuration(double seconds);

 private:
    const double arrow_height_;
    const double label_height_;
    const double object_max_linear_size_ = 100.;
    double       object_speed_threshold_;
    double       arrow_speed_threshold_;
    double       marker_display_duration_;

    int marker_id_;

    std_msgs::ColorRGBA label_color_, label_color_two, label_color_three,box_color_, hull_color_, arrow_color_,
        centroid_color_, model_color_;

    std::string input_topic_, ros_namespace_;
    std::string markers_out_topic;

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_detected_objects_;

    ros::Publisher publisher_markers_;

    void CommonObj2VisualObj(const std::vector<msg_obj::Obj>&      in,
                             msg_lidar_shape::DetectedObjectArray& out);

    void CommonObj2VisualObj(const std::vector<msg_obj_perception::Obj> &in,
                             msg_lidar_shape::DetectedObjectArray &out);

    visualization_msgs::MarkerArray
    ObjectsToLabels(const msg_lidar_shape::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray
    ObjectsToVelocity(const msg_lidar_shape::DetectedObjectArray& in_objects);
    visualization_msgs::MarkerArray
    ObjectsToID(const msg_lidar_shape::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray
    ObjectsToArrows(const msg_lidar_shape::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray
    ObjectsToBoxes(const msg_lidar_shape::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray
    ObjectsToModels(const msg_lidar_shape::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray
    ObjectsToHulls(const msg_lidar_shape::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray
    ObjectsToCentroids(const msg_lidar_shape::DetectedObjectArray& in_objects);

    std::string ColorToString(const std_msgs::ColorRGBA& in_color);

    bool IsObjectValid(const msg_lidar_shape::DetectedObject& in_object);

    float CheckColor(double value);

    float CheckAlpha(double value);

    std_msgs::ColorRGBA ParseColor(float r, float g, float b, float a);

    visualization_msgs::Marker CreateWireframeMarker(const float& center_x,
                                                     const float& center_y,
                                                     const float& center_z,
                                                     float size_x, float size_y,
                                                     const float& size_z);
};

#endif  // CLASS_OBJ_VISUALIZATION_H
