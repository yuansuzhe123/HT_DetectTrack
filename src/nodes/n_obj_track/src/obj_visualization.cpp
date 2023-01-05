/*******************************************************************/
/*
 * @Author: Yuan
 * @Date: 2022-10-14 16:34:41
 * @Email:824100902@qq.com
 * @Description: 
 * @Beta:2.0
 */
/*******************************************************************/

#include "obj_visualization.h"

ObjVisualization::ObjVisualization(const std::string& topic_name)
    : arrow_height_(0.5), label_height_(0.5), markers_out_topic(topic_name)
{
    ros::NodeHandle private_nh_("~");

    ros_namespace_ = ros::this_node::getNamespace();

    if (ros_namespace_.substr(0, 2) == "//")
    {
        ros_namespace_.erase(ros_namespace_.begin());
    }

    std::string object_src_topic;
    private_nh_.param<std::string>("objects_src_topic", object_src_topic,
                                   "/objects_shape");
    object_src_topic = "/objects_shape";

    private_nh_.param<double>("object_speed_threshold", object_speed_threshold_,
                              0.1);

    private_nh_.param<double>("arrow_speed_threshold", arrow_speed_threshold_,
                              0.25);

    private_nh_.param<double>("marker_display_duration",
                              marker_display_duration_, 0.5);

    std::vector<double> color;
    private_nh_.param<std::vector<double>>("label_color", color,
    { 255., 255., 255., 1 });
    label_color_ = ParseColor(255., 255., 255., 1);
    label_color_two = ParseColor(127., 255., 0., 1); // 黄绿色
    label_color_three = ParseColor(176., 23., 31., 1); // 酒红色

    private_nh_.param<std::vector<double>>("arrow_color", color,
    { 255, 0, 0., 0.8 });
    arrow_color_ = ParseColor(255, 0, 0., 0.8);

    private_nh_.param<std::vector<double>>("hull_color", color,
    { 51., 204., 51., 0.8 });
    hull_color_ = ParseColor(51., 204., 51., 0.8);

    private_nh_.param<std::vector<double>>("box_color", color,
    { 255, 255, 0, 0.2 });
    box_color_ = ParseColor(255, 255, 0, 0.2);

    private_nh_.param<std::vector<double>>("model_color", color,
    { 190., 190., 190., 0.5 });
    model_color_ = ParseColor(190., 190., 190., 0.5);

    private_nh_.param<std::vector<double>>("centroid_color", color,
    { 81., 255., 0., 0.8 });
    centroid_color_ = ParseColor(81., 255., 0., 0.8);

    publisher_markers_ =
            node_handle_.advertise<visualization_msgs::MarkerArray>(
                markers_out_topic, 1);
}

void ObjVisualization::ObjVisualizationMain(
        const std::vector<msg_obj::Obj>& in_objects)
{
    msg_lidar_shape::DetectedObjectArray tem;
    CommonObj2VisualObj(in_objects, tem);
    ObjVisualizationMain(tem);
}

void ObjVisualization::ObjVisualizationMain(
    const std::vector<msg_obj_perception::Obj> &in_objects)
{
  msg_lidar_shape::DetectedObjectArray tem;
  CommonObj2VisualObj(in_objects, tem);
  ObjVisualizationMain(tem);
}

void ObjVisualization::ObjVisualizationMain(
        const msg_lidar_shape::DetectedObjectArray& in_objects)
{

    //	std::cout << "obj visualization is starting." << std::endl;
    visualization_msgs::MarkerArray label_markers, arrow_markers,
            centroid_markers, polygon_hulls, bounding_boxes, object_models,velocity_markers,id_markers;

    visualization_msgs::MarkerArray visualization_markers;

    marker_id_ = 0;

    label_markers = ObjectsToLabels(in_objects);

    arrow_markers = ObjectsToArrows(in_objects);

    polygon_hulls = ObjectsToHulls(in_objects);

    bounding_boxes = ObjectsToBoxes(in_objects);

    centroid_markers = ObjectsToCentroids(in_objects);

    velocity_markers = ObjectsToVelocity(in_objects);

    id_markers = ObjectsToID(in_objects);

    int n_marker_size = 0;
    n_marker_size += label_markers.markers.size();
    n_marker_size += arrow_markers.markers.size();
    n_marker_size += polygon_hulls.markers.size();
    n_marker_size += arrow_markers.markers.size();
    n_marker_size += bounding_boxes.markers.size();
    n_marker_size += centroid_markers.markers.size();
    n_marker_size += velocity_markers.markers.size();
    n_marker_size += id_markers.markers.size();


    visualization_markers.markers.reserve(n_marker_size);//为了避免出现多次内存分配

    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                         label_markers.markers.begin(),
                                         label_markers.markers.end());
    // visualization_markers.markers.insert(visualization_markers.markers.end(),
    // 									 arrow_markers.markers.begin(),
    // 									 arrow_markers.markers.end());
    // visualization_markers.markers.insert(visualization_markers.markers.end(),
    // 									 polygon_hulls.markers.begin(),
    // 									 polygon_hulls.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                         bounding_boxes.markers.begin(),
                                         bounding_boxes.markers.end());

    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                         velocity_markers.markers.begin(),
                                         velocity_markers.markers.end());

    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                         id_markers.markers.begin(),
                                         id_markers.markers.end());

    publisher_markers_.publish(visualization_markers);
}

// 融合节点调用
void ObjVisualization::CommonObj2VisualObj(
        const std::vector<msg_obj::Obj>&      in,
        msg_lidar_shape::DetectedObjectArray& out)
{
    if (in.empty())
    {
        return;
    }
    msg_lidar_shape::DetectedObject detected_obj;
    int                             i = 0;
    for (auto& raw : in)
    {
        detected_obj.valid = true;
        switch (raw.type) // 2021-07-22 赋予类别
        {
        case UNKNOWN_SMALL:
          detected_obj.label = "unknown";
          break;
        case CAR:
          detected_obj.label = "car";
          break;
        case TRUCK:
          detected_obj.label = "truck";
          break;
        case PEDESTRIAN:
          detected_obj.label = "person";
          break;
        case MOTOR_BIKE:
          detected_obj.label = "obstacle";
          break;
        case BIKE:
          detected_obj.label = "stone";
          break;
        case UNKNOWN_BIG:
          detected_obj.label = "unknown";
          break;
        case 8:
          detected_obj.label = "out-fov";
          break;
        case 9:
          detected_obj.label = "no-assis";
          break;
        case UNCLASSIFIED:
        default:
          detected_obj.label = "unknown";
          break;
        }
        detected_obj.pose.position.x = raw.x;
        detected_obj.pose.position.y = raw.y;
        detected_obj.pose.position.z = raw.z;

        detected_obj.velocity_reliable = true;
        detected_obj.velocity.linear.x = raw.vx;
        detected_obj.velocity.linear.y = raw.vy;
        detected_obj.velocity.linear.z = raw.vz;

        tf::Quaternion quaternion =
                tf::createQuaternionFromYaw(raw.orientation * M_PI / 180);
        detected_obj.pose.orientation.x = quaternion.x();
        detected_obj.pose.orientation.y = quaternion.y();
        detected_obj.pose.orientation.z = quaternion.z();
        detected_obj.pose.orientation.w = quaternion.w();

        //detected_obj.id = i++;
        detected_obj.id = raw.id;//2020-12-14 赵国春修改

        detected_obj.pose_reliable = true;

        detected_obj.color = box_color_; // 2021-07-22 tjf:每一种box赋予一种颜色

        detected_obj.dimensions.x = raw.length;
        detected_obj.dimensions.y = raw.width;
        detected_obj.dimensions.z = raw.height;

        detected_obj.x = raw.x;
        detected_obj.y = raw.y;
        detected_obj.angle = raw.orientation;
        detected_obj.width = raw.width;
        detected_obj.height = raw.height;

        detected_obj.score = ( float )raw.obj_certainty;
        detected_obj.absolute_motion = ( int )raw.absolute_motion;

        out.objects.push_back(detected_obj);
    }
}

// 毫米波节点调用
void ObjVisualization::CommonObj2VisualObj(
    const std::vector<msg_obj_perception::Obj> &in,
    msg_lidar_shape::DetectedObjectArray &out)
{
  if (in.empty())
  {
    return;
  }
  msg_lidar_shape::DetectedObject detected_obj;
  int i = 0;
  for (auto &raw : in)
  {
    detected_obj.valid = true;
    switch (raw.type) // 2021-07-22 赋予类别
    {
    case UNKNOWN_SMALL:
      detected_obj.label = "unknown";
      break;
    case CAR:
      detected_obj.label = "car";
      break;
    case TRUCK:
      detected_obj.label = "truck";
      break;
    case PEDESTRIAN:
      detected_obj.label = "person";
      break;
    case MOTOR_BIKE:
      detected_obj.label = "obstacle";
      break;
    case BIKE:
      detected_obj.label = "stone";
      break;
    case UNKNOWN_BIG:
      detected_obj.label = "unknown";
      break;
    case 8:
      detected_obj.label = "out-fov";
      break;
    case 9:
      detected_obj.label = "no-assis";
      break;
    case UNCLASSIFIED:
    default:
      detected_obj.label = "unknown";
      break;
    }
    detected_obj.pose.position.x = raw.x;
    detected_obj.pose.position.y = raw.y;
    detected_obj.pose.position.z = raw.z;

    detected_obj.velocity_reliable = true;
    detected_obj.velocity.linear.x = raw.vx;
    detected_obj.velocity.linear.y = raw.vy;
    detected_obj.velocity.linear.z = raw.vz;

    tf::Quaternion quaternion =
        tf::createQuaternionFromYaw(raw.orientation * M_PI / 180);
    detected_obj.pose.orientation.x = quaternion.x();
    detected_obj.pose.orientation.y = quaternion.y();
    detected_obj.pose.orientation.z = quaternion.z();
    detected_obj.pose.orientation.w = quaternion.w();

    //detected_obj.id = i++;
    detected_obj.id = raw.id_track; //2020-12-14 赵国春修改
    if(raw.relative_motion == 1 )
    detected_obj.pose_reliable = true;

    detected_obj.color = box_color_; // 2021-07-22 tjf:每一种box赋予一种颜色

    detected_obj.dimensions.x = raw.length;
    detected_obj.dimensions.y = raw.width;
    detected_obj.dimensions.z = raw.height;

    detected_obj.x = raw.x;
    detected_obj.y = raw.y;
    detected_obj.angle = raw.orientation;
    detected_obj.width = raw.width;
    detected_obj.height = raw.height;

    detected_obj.score = (float)raw.obj_certainty;
    detected_obj.absolute_motion = (int)raw.absolute_motion;

    out.objects.push_back(detected_obj);
  }
}

//设置可视化颜色
void ObjVisualization::SetColor(float r, float g, float b, float a)
{
    box_color_ = ParseColor(r, g, b, a);
}

//设置可视化持续时间 sec: 单位秒
void ObjVisualization::SetDuration(double seconds)
{
    marker_display_duration_ = seconds;
}

//颜色值归一化【0 255】归一化到【0 1】
float ObjVisualization::CheckColor(double value)
{
    float final_value;
    if (value > 255.)
        final_value = 1.f;
    else if (value < 0)
        final_value = 0.f;
    else
        final_value = value / 255.f;
    return final_value;
}

//将std_msgs::ColorRGBA转换为字符串
std::string ObjVisualization::ColorToString(const std_msgs::ColorRGBA& in_color)
{
    std::stringstream stream;

    stream << "{R:" << std::fixed << std::setprecision(1) << in_color.r * 255
           << ", ";
    stream << "G:" << std::fixed << std::setprecision(1) << in_color.g * 255
           << ", ";
    stream << "B:" << std::fixed << std::setprecision(1) << in_color.b * 255
           << ", ";
    stream << "A:" << std::fixed << std::setprecision(1) << in_color.a << "}";
    return stream.str();
}

//Alpha值有效性检查
float ObjVisualization::CheckAlpha(double value)
{
    float final_value;
    if (value > 1.)
        final_value = 1.f;
    else if (value < 0.1)
        final_value = 0.1f;
    else
        final_value = value;
    return final_value;
}

std_msgs::ColorRGBA ObjVisualization::ParseColor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA color;
    color.r = CheckColor(r);
    color.g = CheckColor(g);
    color.b = CheckColor(b);
    color.a = CheckAlpha(a);
    return color;
}

visualization_msgs::MarkerArray ObjVisualization::ObjectsToCentroids(
    const msg_lidar_shape::DetectedObjectArray& in_objects)
{
    visualization_msgs::MarkerArray centroid_markers;
    for (auto const& object : in_objects.objects)
    {
        if (IsObjectValid(object))
        {
            visualization_msgs::Marker centroid_marker;
            centroid_marker.lifetime = ros::Duration(marker_display_duration_);
            //      object.header.frame_id = "my_frame";
            //      object.header.stamp = ros::Time::now();
            //      centroid_marker.header = in_objects.header;
            centroid_marker.header.frame_id = "rslidar";
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.pose = object.pose;
            centroid_marker.ns = ros_namespace_ + "/centroid_markers";

            centroid_marker.scale.x = 0.4;
            centroid_marker.scale.y = 0.4;
            centroid_marker.scale.z = 0.4;

            if (object.color.a == 0)
            {
                centroid_marker.color = centroid_color_;
            }
            else
            {
                centroid_marker.color = object.color;
            }
            centroid_marker.id = marker_id_++;
            
            centroid_markers.markers.push_back(centroid_marker);
        }
    }
    return centroid_markers;
}  // ObjectsToCentroids

visualization_msgs::MarkerArray ObjVisualization::ObjectsToBoxes(
        const msg_lidar_shape::DetectedObjectArray& in_objects)
{
    visualization_msgs::MarkerArray object_boxes;

    for (auto const& object : in_objects.objects)
    {
        if (IsObjectValid(object)
                && (object.pose_reliable || object.label != "unknown")
                && (object.dimensions.x + object.dimensions.y + object.dimensions.z)
                < object_max_linear_size_)
        {
            visualization_msgs::Marker box = CreateWireframeMarker(
                        object.pose.position.x, object.pose.position.y,
                        object.pose.position.z, object.dimensions.x,
                        object.dimensions.y, object.dimensions.z);

            box.lifetime = ros::Duration(marker_display_duration_);
            //      box.header = in_objects.header;
            box.header.frame_id = "rslidar";
            box.type = visualization_msgs::Marker::LINE_LIST;
            box.action = visualization_msgs::Marker::ADD;
            box.ns = ros_namespace_ + "/box_markers";
            box.id = marker_id_++;
            //            box.scale = object.dimensions;
            box.frame_locked = false;
            // box.scale.x = 0.2;
            // box.scale.y = 0.2;
            // box.scale.z = 0.2;
            box.scale.x = 0.1; // 2021-07-22 marker线条的粗细
            box.scale.y = 0.1;
            box.scale.z = 0.1;
            box.header.stamp = ros::Time::now();

            box.pose.position = object.pose.position;

            if (object.pose_reliable)
                box.pose.orientation = object.pose.orientation;

            if (object.color.a == 0)
            {
                box.color = box_color_;
            }
            else
            {
                box.color = object.color;
            }

            object_boxes.markers.push_back(box);
        }
    }
    return object_boxes;
}  // ObjectsToBoxes

visualization_msgs::Marker ObjVisualization::CreateWireframeMarker(
        const float& center_x, const float& center_y, const float& center_z,
        float size_x, float size_y, const float& size_z)
{
    visualization_msgs::Marker box_marker;
    box_marker.pose.position.x = center_x;
    box_marker.pose.position.y = center_y;
    box_marker.pose.position.z = center_z;
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    float                half_x = (0.5) * size_x;
    float                half_y = (0.5) * size_y;
    float                half_z = (0.5) * size_z;
    p1.x = half_x;
    p1.y = half_y;
    p1.z = half_z;
    p2.x = half_x;
    p2.y = -half_y;
    p2.z = half_z;
    p3.x = -half_x;
    p3.y = -half_y;
    p3.z = half_z;
    p4.x = -half_x;
    p4.y = half_y;
    p4.z = half_z;
    p5 = p1;
    p5.z = -half_z;
    p6 = p2;
    p6.z = -half_z;
    p7 = p3;
    p7.z = -half_z;
    p8 = p4;
    p8.z = -half_z;
    box_marker.points.reserve(24);
    box_marker.points.push_back(p1);
    box_marker.points.push_back(p2);
    box_marker.points.push_back(p2);
    box_marker.points.push_back(p3);
    box_marker.points.push_back(p3);
    box_marker.points.push_back(p4);
    box_marker.points.push_back(p4);
    box_marker.points.push_back(p1);
    box_marker.points.push_back(p1);
    box_marker.points.push_back(p5);
    box_marker.points.push_back(p2);
    box_marker.points.push_back(p6);
    box_marker.points.push_back(p3);
    box_marker.points.push_back(p7);
    box_marker.points.push_back(p4);
    box_marker.points.push_back(p8);
    box_marker.points.push_back(p5);
    box_marker.points.push_back(p6);
    box_marker.points.push_back(p6);
    box_marker.points.push_back(p7);
    box_marker.points.push_back(p7);
    box_marker.points.push_back(p8);
    box_marker.points.push_back(p8);
    box_marker.points.push_back(p5);
    return box_marker;
}

visualization_msgs::MarkerArray ObjVisualization::ObjectsToModels(
        const msg_lidar_shape::DetectedObjectArray& in_objects)
{
    visualization_msgs::MarkerArray object_models;

    for (auto const& object : in_objects.objects)
    {
        if (!IsObjectValid(object))
        {
            std::cout << "object is invalid" << std::endl;
        }
        if (object.label == "unknow")
        {
            //      std::cout << "object.label is unknow" << std::endl;
        }
        if (!(object.dimensions.x + object.dimensions.y + object.dimensions.z)
                < object_max_linear_size_)
        {
            //      std::cout << "object.dimensions.x + object.dimensions.y + "
            //                   "object.dimensions.z < object_max_linear_size_"
            //                << std::endl;
        }
        if (IsObjectValid(object) && object.label != "unknown"
                && (object.dimensions.x + object.dimensions.y + object.dimensions.z)
                < object_max_linear_size_)
        {
            visualization_msgs::Marker model;

            model.lifetime = ros::Duration(marker_display_duration_);
            //      model.header = in_objects.header;
            model.header.frame_id = "rslidar";
            model.type = visualization_msgs::Marker::MESH_RESOURCE;
            model.action = visualization_msgs::Marker::ADD;
            model.ns = ros_namespace_ + "/model_markers";
            model.mesh_use_embedded_materials = false;
            //      std::cout << "model.header.frame_id is :" <<
            //      model.header.frame_id
            //                << std::endl;
            //      std::cout << "model.type is :" << model.type << std::endl;
            model.color = model_color_;
            if (object.label == "car")
            {
                model.mesh_resource = "package://n_lidar_obj/models/car.dae";
            }
            else if (object.label == "person")
            {
                model.mesh_resource = "package://n_lidar_obj/models/person.dae";
            }
            else if (object.label == "bicycle" || object.label == "bike")
            {
                model.mesh_resource = "package://n_lidar_obj/models/bike.dae";
            }
            else if (object.label == "bus")
            {
                model.mesh_resource = "package://n_lidar_obj/models/bus.dae";
            }
            else if (object.label == "truck")
            {
                model.mesh_resource = "package://n_lidar_obj/models/truck.dae";
            }
            else
            {
                model.mesh_resource = "package://n_lidar_obj/models/box.dae";
            }
            model.scale.x = 1;
            model.scale.y = 1;
            model.scale.z = 1;
            model.id = marker_id_++;
            model.pose.position = object.pose.position;
            model.pose.position.z -= object.dimensions.z / 2;

            if (object.pose_reliable)
                model.pose.orientation = object.pose.orientation;

            object_models.markers.push_back(model);
        }
    }
    return object_models;
}  // ObjectsToModels

visualization_msgs::MarkerArray ObjVisualization::ObjectsToHulls(
        const msg_lidar_shape::DetectedObjectArray& in_objects)
{
    visualization_msgs::MarkerArray polygon_hulls;

    for (auto const& object : in_objects.objects)
    {
        if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty()
                && object.label == "unknown")
        {
            visualization_msgs::Marker hull;
            hull.lifetime = ros::Duration(marker_display_duration_);
            //      hull.header = in_objects.header;
            hull.header.frame_id = "rslidar";
            hull.type = visualization_msgs::Marker::LINE_STRIP;
            hull.action = visualization_msgs::Marker::ADD;
            hull.ns = ros_namespace_ + "/hull_markers";
            hull.id = marker_id_++;
            hull.scale.x = 0.2;

            for (auto const& point : object.convex_hull.polygon.points)
            {
                geometry_msgs::Point tmp_point;
                tmp_point.x = point.x;
                tmp_point.y = point.y;
                tmp_point.z = point.z;
                hull.points.push_back(tmp_point);
            }

            if (object.color.a == 0)
            {
                hull.color = hull_color_;
            }
            else
            {
                hull.color = object.color;
            }

            polygon_hulls.markers.push_back(hull);
        }
    }
    return polygon_hulls;
}

visualization_msgs::MarkerArray ObjVisualization::ObjectsToArrows(
        const msg_lidar_shape::DetectedObjectArray& in_objects)
{
    visualization_msgs::MarkerArray arrow_markers;
    for (auto const& object : in_objects.objects)
    {
        if (IsObjectValid(object) && object.pose_reliable)
        {
            double velocity = object.velocity.linear.x;

            if (abs(velocity) >= arrow_speed_threshold_)
            {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.lifetime = ros::Duration(marker_display_duration_);

                tf::Quaternion q(
                            object.pose.orientation.x, object.pose.orientation.y,
                            object.pose.orientation.z, object.pose.orientation.w);
                double roll, pitch, yaw;

                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                // in the case motion model fit opposite direction
                if (velocity < -0.1)
                {
                    yaw += M_PI;
                    // normalize angle
                    while (yaw > M_PI) yaw -= 2. * M_PI;
                    while (yaw < -M_PI) yaw += 2. * M_PI;
                }

                tf::Matrix3x3  obs_mat;
                tf::Quaternion q_tf;

                obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
                obs_mat.getRotation(q_tf);

                //        arrow_marker.header = in_objects.header;
                arrow_marker.header.frame_id = "rslidar";
                arrow_marker.ns = ros_namespace_ + "/arrow_markers";
                arrow_marker.action = visualization_msgs::Marker::ADD;
                arrow_marker.type = visualization_msgs::Marker::ARROW;

                // green
                if (object.color.a == 0)
                {
                    arrow_marker.color = arrow_color_;
                }
                else
                {
                    arrow_marker.color = object.color;
                }
                arrow_marker.id = marker_id_++;

                // Set the pose of the marker.  This is a full 6DOF pose
                // relative to the frame/time specified in the header
                arrow_marker.pose.position.x = object.pose.position.x;
                arrow_marker.pose.position.y = object.pose.position.y;
                arrow_marker.pose.position.z = arrow_height_;

                arrow_marker.pose.orientation.x = q_tf.getX();
                arrow_marker.pose.orientation.y = q_tf.getY();
                arrow_marker.pose.orientation.z = q_tf.getZ();
                arrow_marker.pose.orientation.w = q_tf.getW();

                // Set the scale of the arrow -- 1x1x1 here means 1m on a side
                arrow_marker.scale.x = 3;
                arrow_marker.scale.y = 0.1;
                arrow_marker.scale.z = 0.1;

                arrow_markers.markers.push_back(arrow_marker);
            }  // velocity threshold
        }      // valid object
    }          // end for
    return arrow_markers;
}  // ObjectsToArrows

visualization_msgs::MarkerArray ObjVisualization::ObjectsToLabels(
        const msg_lidar_shape::DetectedObjectArray& in_objects)
{
    visualization_msgs::MarkerArray label_markers;
    for (auto const& object : in_objects.objects)
    {

        if (IsObjectValid(object))
        {
            visualization_msgs::Marker certainty_marker;
            visualization_msgs::Marker abs_motion_marker;            
            certainty_marker.lifetime = ros::Duration(marker_display_duration_);
            //      certainty_marker.header = in_objects.header;
            certainty_marker.header.frame_id = "rslidar";
            certainty_marker.ns = ros_namespace_ + "/certainty_markers";
            certainty_marker.action = visualization_msgs::Marker::ADD;
            certainty_marker.type =
                    visualization_msgs::Marker::TEXT_VIEW_FACING;
            certainty_marker.scale.x = 2.0; // 2021-07-22 字体大小 tjf
            certainty_marker.scale.y = 2.0;
            certainty_marker.scale.z = 2.0;

            abs_motion_marker = certainty_marker;
            abs_motion_marker.ns = ros_namespace_ + "/abs_motion";
            abs_motion_marker.color = label_color_;
            // certainty_marker.color = label_color_;
            certainty_marker.color = label_color_two;
            certainty_marker.id = marker_id_++;
            abs_motion_marker.id = marker_id_++;



            if (!object.label.empty() && object.label != "unknown")
            {
                certainty_marker.text =
                        object.label + " ";  // Object Class if available
            }

            std::stringstream distance_stream;
            distance_stream
                    << std::fixed << std::setprecision(1)
                    << sqrt((object.pose.position.x * object.pose.position.x)
                            + (object.pose.position.y * object.pose.position.y));
            // std::string distance_str = distance_stream.str() + " m";
            // certainty_marker.text += distance_str;

            if (object.velocity_reliable)
            {
                double velocity = object.velocity.linear.x;
                if (velocity < -0.1)
                {
                    velocity *= -1;
                }

                if (abs(velocity) < object_speed_threshold_)
                {
                    velocity = 0.0;
                }

                tf::Quaternion q(
                            object.pose.orientation.x, object.pose.orientation.y,
                            object.pose.orientation.z, object.pose.orientation.w);

                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                // convert m/s to km/h
                std::stringstream kmh_velocity_stream;
                kmh_velocity_stream << std::fixed << std::setprecision(1)
                                    << (velocity * 3.6);
                // std::string text = "\n<" + std::to_string(object.id) + "> "
                // 				   + kmh_velocity_stream.str() + " km/h";
                // std::string text = "x:" + std::to_string(object.pose.position.x)+";y:" + std::to_string(object.pose.position.y)+";z:" + std::to_string(object.pose.position.z);
                // certainty_marker.text += text;
                std::string motion_text =
                        "abs_motion:" + std::to_string(object.absolute_motion);
                abs_motion_marker.text = motion_text;
            }

            certainty_marker.pose.position.x = object.pose.position.x;
            certainty_marker.pose.position.y = object.pose.position.y;
            certainty_marker.pose.position.z = label_height_ + 2; // 2021-07-22 类别字体间距 tjf
            // certainty_marker.scale.z = 0.5;

            abs_motion_marker.pose = certainty_marker.pose;
            abs_motion_marker.pose.position.x += 0.5;


            if (!certainty_marker.text.empty())
            {
                label_markers.markers.push_back(certainty_marker);
                label_markers.markers.push_back(abs_motion_marker);                
            }
        }
    }  // end in_objects.objects loop

    return label_markers;
}

visualization_msgs::MarkerArray ObjVisualization::ObjectsToVelocity(const msg_lidar_shape::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray label_markers;
    for (auto const& object : in_objects.objects)
    {
        if (IsObjectValid(object))
        {
            visualization_msgs::Marker velocity_marker;

            velocity_marker.lifetime = ros::Duration(marker_display_duration_);
            //      certainty_marker.header = in_objects.header;
            velocity_marker.header.frame_id = "rslidar";
            velocity_marker.ns = ros_namespace_ + "/velocity_markers";
            velocity_marker.action = visualization_msgs::Marker::ADD;
            velocity_marker.type =
                    visualization_msgs::Marker::TEXT_VIEW_FACING;
            velocity_marker.scale.x = 0.5;
            velocity_marker.scale.y = 0.5;
            velocity_marker.scale.z = 0.5;
            velocity_marker.color = label_color_;
            velocity_marker.id = marker_id_++;

            velocity_marker.text += (object.velocity.linear.x != 0) ? ("vx:" + std::to_string(object.velocity.linear.x)) : "";
            velocity_marker.text += (object.velocity.linear.y != 0) ? ("vy:" + std::to_string(object.velocity.linear.y)) : "";
            velocity_marker.text += (object.velocity.linear.z != 0) ? ("vz:" + std::to_string(object.velocity.linear.z)) : "";

            velocity_marker.pose.position.x = object.pose.position.x;
            velocity_marker.pose.position.y = object.pose.position.y;
            velocity_marker.pose.position.z = label_height_;
            velocity_marker.scale.z = 0.5;

            if (!velocity_marker.text.empty())
            {
                label_markers.markers.push_back(velocity_marker);
            }
        }
    }  // end in_objects.objects loop

    return label_markers;
}

visualization_msgs::MarkerArray ObjVisualization::ObjectsToID(const msg_lidar_shape::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray label_markers;
    for (auto const& object : in_objects.objects)
    {
        if (IsObjectValid(object))
        {
            visualization_msgs::Marker trace_id_marker;

            trace_id_marker.lifetime = ros::Duration(marker_display_duration_);
            //      certainty_marker.header = in_objects.header;
            trace_id_marker.header.frame_id = "rslidar";
            trace_id_marker.ns = ros_namespace_ + "/trace_id";
            trace_id_marker.action = visualization_msgs::Marker::ADD;
            trace_id_marker.type =
                    visualization_msgs::Marker::TEXT_VIEW_FACING;
            trace_id_marker.scale.x = 1.0; // 2021-07-22 tjf 改变字体的大小
            trace_id_marker.scale.y = 1.0;
            trace_id_marker.scale.z = 1.0;
            trace_id_marker.color = label_color_three;
            trace_id_marker.id = marker_id_++;
            //ysz1020
            trace_id_marker.text += (object.id != 0) ? ("id:" + std::to_string(object.id)) : "";
            trace_id_marker.pose.position.x = object.pose.position.x;
            trace_id_marker.pose.position.y = object.pose.position.y;
            trace_id_marker.pose.position.z = label_height_;
            //trace_id_marker.scale.z = 0.5;

            if (!trace_id_marker.text.empty())
            {
                label_markers.markers.push_back(trace_id_marker);
            }
        }
    }  // end in_objects.objects loop

    return label_markers;
}

bool ObjVisualization::IsObjectValid(
        const msg_lidar_shape::DetectedObject& in_object)
{
    if (!in_object.valid || std::isnan(in_object.pose.orientation.x)
            || std::isnan(in_object.pose.orientation.y)
            || std::isnan(in_object.pose.orientation.z)
            || std::isnan(in_object.pose.orientation.w)
            || std::isnan(in_object.pose.position.x)
            || std::isnan(in_object.pose.position.y)
            || std::isnan(in_object.pose.position.z)
            || (in_object.pose.position.x == 0.)
            || (in_object.pose.position.y == 0.) || (in_object.dimensions.x <= 0.)
            || (in_object.dimensions.y <= 0.) || (in_object.dimensions.z <= 0.))
    {
        return false;
    }
    return true;
}  // end IsObjectValid
