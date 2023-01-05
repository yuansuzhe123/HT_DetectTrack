/*******************************************************************/
/*
 * @Author: Yuan
 * @Date: 2022-10-14 16:34:41
 * @Email:824100902@qq.com
 * @Description: 
 * @Beta:2.0
 */
/*******************************************************************/

#include "c_object_state.h"

namespace lidar
{
namespace detect
{

bool ObjectState::EstimateShapeFuse(msg_lidar_obj::msg_lidar_obj& v_objs)
    {
        msg_lidar_shape::DetectedObject shape_output;
        //std::vector<double> fuse_parameter;
        double obj_main_l_min = 0,obj_main_w_min = 0,obj_main_h_min = 0;
        double obj_main_l_max = 0,obj_main_w_max = 0,obj_main_h_max = 0;
        double obj_search_area = 0;
        double obj_fuse_height = 0;
        double point_search_area = 0;
        double dilate_parameter = 0.5;
        for(int k = 0 ; k < v_objs.lidar_obj.size() ; k ++)
        {
             float roi_x_min =  0.5;
             float roi_x_max  = 4;
             float roi_y_min  = -22;
             float roi_y_max = -10;
             float roi_z_min = 2.8;
             if(std::fabs(v_objs.lidar_obj[k].x - 9.8) <1 && std::fabs(v_objs.lidar_obj[k].y - 32.0)<1)
             {
                  if(std::fabs(v_objs.lidar_obj[k].length - 0.23) <1 &&  std::fabs(v_objs.lidar_obj[k].width - 0.74) <1 && std::fabs(v_objs.lidar_obj[k].height - 0.49) <0.8)
                  {
                         v_objs.lidar_obj[k].id = 0;
                  }
             }
             if(v_objs.lidar_obj[k].x >roi_x_min&& v_objs.lidar_obj[k].x< roi_x_max&& v_objs.lidar_obj[k].y >  roi_y_min &&  v_objs.lidar_obj[k].y< roi_y_max &&v_objs.lidar_obj[k].z >  roi_z_min )
             {
                //  std::cout<<"roi8888888888888888888"<<std::endl;
                //  std::cout<<"center_x"<<v_objs.lidar_obj[k].x <<std::endl;
                //  std::cout<<"center_y"<<v_objs.lidar_obj[k].y<<std::endl;
                //  std::cout<<"center_z"<<v_objs.lidar_obj[k].z <<std::endl;
               v_objs.lidar_obj[k].id = 0;
               continue;
             }
            if(v_objs.lidar_obj[k].id == 0)
            {
                continue;
            }
                bool whether_fuse = false;
                //std::cout << "进入判断的主目标id号~~~~~~~~~~~~~~~~~~~~~~~~~~~ " << v_objs.lidar_obj[k].id<< std::endl;
              //  for (auto& obj_2 : v_objs.lidar_obj)
                for(int j = 0;j<v_objs.lidar_obj.size();j++)
                {  
                    // if(obj_2.id == 0 || obj_2.id ==v_objs.lidar_obj[k].id)
                    // {
                    //     continue;
                    // }
                    if (j <= k)
                    {
                        continue;
                    }
                    if(v_objs.lidar_obj[j].id == 0)
                    {
                        continue;
                    }
                    {
                        double obj_to_obj_distance = std::sqrt(std::pow((v_objs.lidar_obj[j].x - v_objs.lidar_obj[k].x), 2) + std::pow(v_objs.lidar_obj[j].y - v_objs.lidar_obj[k].y, 2));
                       double  norm_max_obj_to_obj_distaance  =   std::sqrt(std::pow( v_objs.lidar_obj[k].length,2) + std::pow(v_objs.lidar_obj[k].width,2) )   +  std::sqrt(std::pow(v_objs.lidar_obj[j].length,2) +  std::pow(v_objs.lidar_obj[j].width,2)  );
                      if(obj_to_obj_distance > (norm_max_obj_to_obj_distaance +1) )
                      {
                        continue;
                      }
                        double obj1_l = v_objs.lidar_obj[k].length;
                        double obj1_w = v_objs.lidar_obj[k].width;
                        // double obj2_l = obj_2.length;
                        // double obj2_w = obj_2.width;
                        double obj2_l =  v_objs.lidar_obj[j].length;
                        double obj2_w = v_objs.lidar_obj[j].width;
                        double orientation_first = v_objs.lidar_obj[k].orientation/180*M_PI;
                        // double orientation_second = obj_2.orientation/180*M_PI;
                        double orientation_second = v_objs.lidar_obj[j].orientation/180*M_PI;
                        // double orientation_first = v_objs.lidar_obj[k].orientation<45?
                        // v_objs.lidar_obj[k].orientation/180*3.1415926:(v_objs.lidar_obj[k].orientation)/180*3.1415926;
                        // double orientation_second = obj_2.orientation<45?
                        // obj_2.orientation/180*3.1415926:(obj_2.orientation-90)/180*3.1415926;
                        //主obj的四点坐标，旋转前
                        double obj1_ori_p1_x = v_objs.lidar_obj[k].x+dilate_parameter*obj1_l;
                        double obj1_ori_p1_y = v_objs.lidar_obj[k].y+dilate_parameter*obj1_w;
                        double obj1_ori_p2_x = v_objs.lidar_obj[k].x-dilate_parameter*obj1_l;
                        double obj1_ori_p2_y = v_objs.lidar_obj[k].y+dilate_parameter*obj1_w;
                        double obj1_ori_p3_x = v_objs.lidar_obj[k].x-dilate_parameter*obj1_l;
                        double obj1_ori_p3_y = v_objs.lidar_obj[k].y-dilate_parameter*obj1_w;
                        double obj1_ori_p4_x = v_objs.lidar_obj[k].x+dilate_parameter*obj1_l;
                        double obj1_ori_p4_y = v_objs.lidar_obj[k].y-dilate_parameter*obj1_w;
                        //进行旋转
                        double obj1_rot_p1_x = (obj1_ori_p1_x - v_objs.lidar_obj[k].x)*cos(orientation_first)
                        -(obj1_ori_p1_y - v_objs.lidar_obj[k].y)*sin(orientation_first)+v_objs.lidar_obj[k].x;
                        double obj1_rot_p1_y = (obj1_ori_p1_x - v_objs.lidar_obj[k].x)*sin(orientation_first)
                        +(obj1_ori_p1_y - v_objs.lidar_obj[k].y)*cos(orientation_first)+v_objs.lidar_obj[k].y;

                        double obj1_rot_p2_x = (obj1_ori_p2_x - v_objs.lidar_obj[k].x)*cos(orientation_first)
                        -(obj1_ori_p2_y - v_objs.lidar_obj[k].y)*sin(orientation_first)+v_objs.lidar_obj[k].x;
                        double obj1_rot_p2_y = (obj1_ori_p2_x - v_objs.lidar_obj[k].x)*sin(orientation_first)
                        +(obj1_ori_p2_y - v_objs.lidar_obj[k].y)*cos(orientation_first)+v_objs.lidar_obj[k].y;

                        double obj1_rot_p3_x = (obj1_ori_p3_x - v_objs.lidar_obj[k].x)*cos(orientation_first)
                        -(obj1_ori_p3_y - v_objs.lidar_obj[k].y)*sin(orientation_first)+v_objs.lidar_obj[k].x;
                        double obj1_rot_p3_y = (obj1_ori_p3_x - v_objs.lidar_obj[k].x)*sin(orientation_first)
                        +(obj1_ori_p3_y - v_objs.lidar_obj[k].y)*cos(orientation_first)+v_objs.lidar_obj[k].y;

                        double obj1_rot_p4_x = (obj1_ori_p4_x - v_objs.lidar_obj[k].x)*cos(orientation_first)
                        -(obj1_ori_p4_y - v_objs.lidar_obj[k].y)*sin(orientation_first)+v_objs.lidar_obj[k].x;
                        double obj1_rot_p4_y = (obj1_ori_p4_x - v_objs.lidar_obj[k].x)*sin(orientation_first)
                        +(obj1_ori_p4_y - v_objs.lidar_obj[k].y)*cos(orientation_first)+v_objs.lidar_obj[k].y;

                        double box_obj1[8]={obj1_rot_p1_x,obj1_rot_p1_y,obj1_rot_p2_x,obj1_rot_p2_y,
                        obj1_rot_p3_x,obj1_rot_p3_y,obj1_rot_p4_x,obj1_rot_p4_y};

                        // //从obj的四点坐标，旋转前
                        // double obj2_ori_p1_x = obj_2.x+dilate_parameter*obj2_l;
                        // double obj2_ori_p1_y = obj_2.y+dilate_parameter*obj2_w;
                        // double obj2_ori_p2_x = obj_2.x-dilate_parameter*obj2_l;
                        // double obj2_ori_p2_y = obj_2.y+dilate_parameter*obj2_w;
                        // double obj2_ori_p3_x = obj_2.x-dilate_parameter*obj2_l;
                        // double obj2_ori_p3_y = obj_2.y-dilate_parameter*obj2_w;
                        // double obj2_ori_p4_x = obj_2.x+dilate_parameter*obj2_l;
                        // double obj2_ori_p4_y = obj_2.y-dilate_parameter*obj2_w;
                        // //进行旋转
                        // double obj2_rot_p1_x = (obj2_ori_p1_x - obj_2.x)*cos(orientation_second)
                        // -(obj2_ori_p1_y - obj_2.y)*sin(orientation_second)+obj_2.x;
                        // double obj2_rot_p1_y = (obj2_ori_p1_x - obj_2.x)*sin(orientation_second)
                        // +(obj2_ori_p1_y - obj_2.y)*cos(orientation_second)+obj_2.y;

                        // double obj2_rot_p2_x = (obj2_ori_p2_x - obj_2.x)*cos(orientation_second)
                        // -(obj2_ori_p2_y - obj_2.y)*sin(orientation_second)+obj_2.x;
                        // double obj2_rot_p2_y = (obj2_ori_p2_x - obj_2.x)*sin(orientation_second)
                        // +(obj2_ori_p2_y - obj_2.y)*cos(orientation_second)+obj_2.y;

                        // double obj2_rot_p3_x = (obj2_ori_p3_x - obj_2.x)*cos(orientation_second)
                        // -(obj2_ori_p3_y - obj_2.y)*sin(orientation_second)+obj_2.x;
                        // double obj2_rot_p3_y = (obj2_ori_p3_x - obj_2.x)*sin(orientation_second)
                        // +(obj2_ori_p3_y - obj_2.y)*cos(orientation_second)+obj_2.y;

                        // double obj2_rot_p4_x = (obj2_ori_p4_x - obj_2.x)*cos(orientation_second)
                        // -(obj2_ori_p4_y - obj_2.y)*sin(orientation_second)+obj_2.x;
                        // double obj2_rot_p4_y = (obj2_ori_p4_x - obj_2.x)*sin(orientation_second)
                        // +(obj2_ori_p4_y - obj_2.y)*cos(orientation_second)+obj_2.y;
                        
                                                //从obj的四点坐标，旋转前
                        double obj2_ori_p1_x = v_objs.lidar_obj[j].x+dilate_parameter*obj2_l;
                        double obj2_ori_p1_y = v_objs.lidar_obj[j].y+dilate_parameter*obj2_w;
                        double obj2_ori_p2_x = v_objs.lidar_obj[j].x-dilate_parameter*obj2_l;
                        double obj2_ori_p2_y = v_objs.lidar_obj[j].y+dilate_parameter*obj2_w;
                        double obj2_ori_p3_x = v_objs.lidar_obj[j].x-dilate_parameter*obj2_l;
                        double obj2_ori_p3_y = v_objs.lidar_obj[j].y-dilate_parameter*obj2_w;
                        double obj2_ori_p4_x = v_objs.lidar_obj[j].x+dilate_parameter*obj2_l;
                        double obj2_ori_p4_y = v_objs.lidar_obj[j].y-dilate_parameter*obj2_w;

                        //进行旋转
                        double obj2_rot_p1_x = (obj2_ori_p1_x - v_objs.lidar_obj[j].x)*cos(orientation_second)
                        -(obj2_ori_p1_y - v_objs.lidar_obj[j].y)*sin(orientation_second)+v_objs.lidar_obj[j].x;
                        double obj2_rot_p1_y = (obj2_ori_p1_x - v_objs.lidar_obj[j].x)*sin(orientation_second)
                        +(obj2_ori_p1_y - v_objs.lidar_obj[j].y)*cos(orientation_second)+v_objs.lidar_obj[j].y;

                        double obj2_rot_p2_x = (obj2_ori_p2_x - v_objs.lidar_obj[j].x)*cos(orientation_second)
                        -(obj2_ori_p2_y - v_objs.lidar_obj[j].y)*sin(orientation_second)+v_objs.lidar_obj[j].x;
                        double obj2_rot_p2_y = (obj2_ori_p2_x - v_objs.lidar_obj[j].x)*sin(orientation_second)
                        +(obj2_ori_p2_y - v_objs.lidar_obj[j].y)*cos(orientation_second)+v_objs.lidar_obj[j].y;

                        double obj2_rot_p3_x = (obj2_ori_p3_x - v_objs.lidar_obj[j].x)*cos(orientation_second)
                        -(obj2_ori_p3_y - v_objs.lidar_obj[j].y)*sin(orientation_second)+v_objs.lidar_obj[j].x;
                        double obj2_rot_p3_y = (obj2_ori_p3_x - v_objs.lidar_obj[j].x)*sin(orientation_second)
                        +(obj2_ori_p3_y - v_objs.lidar_obj[j].y)*cos(orientation_second)+v_objs.lidar_obj[j].y;

                        double obj2_rot_p4_x = (obj2_ori_p4_x - v_objs.lidar_obj[j].x)*cos(orientation_second)
                        -(obj2_ori_p4_y - v_objs.lidar_obj[j].y)*sin(orientation_second)+v_objs.lidar_obj[j].x;
                        double obj2_rot_p4_y = (obj2_ori_p4_x - v_objs.lidar_obj[j].x)*sin(orientation_second)
                        +(obj2_ori_p4_y - v_objs.lidar_obj[j].y)*cos(orientation_second)+v_objs.lidar_obj[j].y;
                        bool inside_fuse_flag = false;
                        bool cross_area_flag = false;
                        // if(InsideJudge(obj_2.x , obj_2.y , box_obj1) && 
                        // (obj_2.z-0.5*obj_2.height)> (v_objs.lidar_obj[k].z - 0.5*v_objs.lidar_obj[k].height)&&
                        // (obj_2.z+0.5*obj_2.height)< (v_objs.lidar_obj[k].z + 0.5*v_objs.lidar_obj[k].height))
                         if(InsideJudge(v_objs.lidar_obj[j].x , v_objs.lidar_obj[j].y , box_obj1) && 
                        (v_objs.lidar_obj[j].z-0.5*v_objs.lidar_obj[j].height)> (v_objs.lidar_obj[k].z - 0.5*v_objs.lidar_obj[k].height)&&
                        (v_objs.lidar_obj[j].z+0.5*v_objs.lidar_obj[j].height)< (v_objs.lidar_obj[k].z + 0.5*v_objs.lidar_obj[k].height)) 
                        {
                            //std::cout<<"中心点被包含～～～～～～～～～～～～～～～～～～～～～～～～～～～～～`"<<std::endl;
                            inside_fuse_flag=(
                            InsideJudge(obj2_rot_p1_x , obj2_rot_p1_y , box_obj1)&&
                            InsideJudge(obj2_rot_p2_x , obj2_rot_p2_y , box_obj1)&&
                            InsideJudge(obj2_rot_p3_x , obj2_rot_p3_y , box_obj1)&&
                            InsideJudge(obj2_rot_p4_x , obj2_rot_p4_y , box_obj1));
                        }
                       else
                        {
                        //      std::cout<<"z轴未完全包围"<<std::endl;
                        }
                        if(!inside_fuse_flag)
                         {
                         const float* area1 =new float[7]{v_objs.lidar_obj[k].x,v_objs.lidar_obj[k].y,v_objs.lidar_obj[k].z,
                        v_objs.lidar_obj[k].length,v_objs.lidar_obj[k].width,v_objs.lidar_obj[k].height,v_objs.lidar_obj[k].orientation/180*M_PI};
                       // const float* area2 =new float[7]{obj_2.x,obj_2.y,obj_2.z,obj_2.length,obj_2.width,obj_2.height,obj_2.orientation/180*M_PI};
                        const float* area2 =new float[7]{v_objs.lidar_obj[j].x,v_objs.lidar_obj[j].y,v_objs.lidar_obj[j].z,v_objs.lidar_obj[j].length,v_objs.lidar_obj[j].width,v_objs.lidar_obj[j].height,v_objs.lidar_obj[j].orientation/180*M_PI};
                        float area1_2 = lidar::rotated::Rotated_Iou(area1, area2);
                        //int a=1,b=2;
                        //int area1_2 = lidar::rotated::Calculate(a,b);
                        delete area1;   //此处删除的时 系统内置类型数据 new delete不需要配对出现
                        delete area2;
                        //std::cout << "源码计算交叉面积"<< area1_2<< std::endl;
                     //   float area_min = obj_2.width*obj_2.length;
                        float area_min = v_objs.lidar_obj[j].width*v_objs.lidar_obj[j].length;
                        float  area_max  =  v_objs.lidar_obj[k].width*v_objs.lidar_obj[k].length;
                        if(area1_2/area_min>0.2 ||  area1_2/ area_max >0.2)
                        {
                            cross_area_flag =  true;
                            // std::cout << "ysz打印obj1的信息=============="<<area1_2/area_min<< std::endl;
                            // std::cout << "yszobj1的中心位置=============="<<v_objs.lidar_obj[k].x<<"," <<v_objs.lidar_obj[k].y<<","<<v_objs.lidar_obj[k].z<<std::endl;
                            // std::cout << "yszobj1的长宽角度=============="<<v_objs.lidar_obj[k].x<<"," <<v_objs.lidar_obj[k].length<<","<<v_objs.lidar_obj[k].width<<std::endl;
                            // std::cout << "ysz打印obj2的信息=============="<<area1_2/area_max<< std::endl;
                            // std::cout << "yszobj2的中心位置=============="<<v_objs.lidar_obj[j].x<<"," <<v_objs.lidar_obj[j].y<<","<<v_objs.lidar_obj[j].z<<std::endl;
                            // std::cout << "yszobj2的长宽角度=============="<<v_objs.lidar_obj[j].x<<"," <<v_objs.lidar_obj[j].length<<","<<v_objs.lidar_obj[j].width<<std::endl;
                        }
                        if(area1_2/area_min>1.01)
                        {
                            // std::cout << "ysz111小面积========="<<area_min<< std::endl;
                            // std::cout << "ysz111交面积========="<<area1_2<< std::endl;
                            // std::cout << "ysz111被包含占比========="<<area1_2/area_min<< std::endl;
                        }    
                         }
                         else
                         {
                         // std::cout<<"完全包围或者z轴不包含"<< std::endl;
                         }
                        //此处最终判断是否融合
                        if(inside_fuse_flag)
                        {
                            //std::cout << "被完全包含~~~~~~~~~~~~~~~~~~~~~~~~~~~ "<< std::endl;
                             v_objs.lidar_obj[j].id = 0;
                            for(int i = 0 ; i <  v_objs.lidar_obj[j].contour_point.size() ; i++)
                            {
                                msg_common::GridPoint temp_point;
                                temp_point.x =  v_objs.lidar_obj[j].contour_point[i].x;
                                temp_point.y =  v_objs.lidar_obj[j].contour_point[i].y;
                                temp_point.z =  v_objs.lidar_obj[j].contour_point[i].z;
                                v_objs.lidar_obj[k].contour_point.push_back(temp_point);
                            }
                        }
                        if (cross_area_flag == true)
                        {
                            double area_level;
                            if (v_objs.lidar_obj[j].width * v_objs.lidar_obj[j].length < v_objs.lidar_obj[k].length * v_objs.lidar_obj[k].width)
                            {
                                area_level = (v_objs.lidar_obj[k].length * v_objs.lidar_obj[k].width) / (v_objs.lidar_obj[j].width * v_objs.lidar_obj[j].length);
                                // if (area_level > 1.5)
                                // {
                                    v_objs.lidar_obj[j].id = 0;
                                    for (int i = 0; i < v_objs.lidar_obj[j].contour_point.size(); i++)
                                    {
                                        msg_common::GridPoint temp_point;
                                        temp_point.x = v_objs.lidar_obj[j].contour_point[i].x;
                                        temp_point.y = v_objs.lidar_obj[j].contour_point[i].y;
                                        temp_point.z = v_objs.lidar_obj[j].contour_point[i].z;
                                        v_objs.lidar_obj[k].contour_point.push_back(temp_point);
                                    }
                                    std::vector<msg_common::GridPoint> temp_vec;
                                    v_objs.lidar_obj[j].contour_point.swap(temp_vec);
                                // }
                            }
                            else
                            {
                                area_level = (v_objs.lidar_obj[j].width * v_objs.lidar_obj[j].length) / (v_objs.lidar_obj[k].length * v_objs.lidar_obj[k].width);
                                // if (area_level > 1.5)
                                // {
                                    v_objs.lidar_obj[k].id = 0;
                                    for (int i = 0; i < v_objs.lidar_obj[k].contour_point.size(); i++)
                                    {
                                        msg_common::GridPoint temp_point;
                                        temp_point.x = v_objs.lidar_obj[k].contour_point[i].x;
                                        temp_point.y = v_objs.lidar_obj[k].contour_point[i].y;
                                        temp_point.z = v_objs.lidar_obj[k].contour_point[i].z;
                                        v_objs.lidar_obj[j].contour_point.push_back(temp_point);
                                    }
                                    std::vector<msg_common::GridPoint> temp_vec;
                                    v_objs.lidar_obj[k].contour_point.swap(temp_vec);
                                // }
                            }
                        } 
                        
                    }
                }   
        }
        return true;
    }

bool ObjectState::InsideJudge(const double pointx , const double pointy , double box[])
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
double ObjectState::CalcClosenessCriterion(const std::vector<double>& C_1,
                                                const std::vector<double>& C_2)
{
    // Paper : Algo.4 Closeness Criterion
    const double min_c_1 =
        *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double max_c_1 =
        *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double min_c_2 =
        *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    const double max_c_2 =
        *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

    std::vector<double> D_1;  // col.4, Algo.4
    for (const auto& c_1_element : C_1)
    {
        const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
        D_1.push_back(std::fabs(v));
    }

    std::vector<double> D_2;  // col.5, Algo.4
    for (const auto& c_2_element : C_2)
    {
        const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
        D_2.push_back(std::fabs (v));
    }

    const double d_min = 0.05;
    const double d_max = 99990.50;
    double       beta = 0;  // col.6, Algo.4
    for (size_t i = 0; i < D_1.size(); ++i)
    {
        const double d = 
            std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
        beta += 1.0 / d;
    }
    return beta;
}

double ObjectState::CalcVarianceCriterion(const std::vector<double>& C_1,
                                                const std::vector<double>& C_2)
{
    const double min_c_1 =
        *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double max_c_1 =
        *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double min_c_2 =
        *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    const double max_c_2 =
        *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

        double d_cent_x = 0,d_cent_y = 0;
    #pragma omp parallel for  
    for(int i = 0; i < C_1.size(); i++)
    {
            d_cent_x += C_1.at(i);
            d_cent_y += C_2.at(i);
    }
    d_cent_x  = d_cent_x/C_1.size();
     d_cent_y  = d_cent_y/C_1.size();
    std::vector<double> vec_d1,vec_d2,vec_d3,vec_d4;
    double d_mean_d1 = 0, d_mean_d2 = 0, d_mean_d3 = 0, d_mean_d4 = 0;
    std::vector<double> vec_d21,vec_d22,vec_d23,vec_d24;
    double d_mean_d21 = 0, d_mean_d22 = 0, d_mean_d23 = 0, d_mean_d24 = 0;
   
    #pragma omp parallel for  
    for(int i = 0; i < C_1.size(); i++)
    {
        if((C_1.at(i) <= d_cent_x )&&(d_cent_x < (min_c_1*0.5+max_c_1*0.5)))
        {
            vec_d1.push_back(max_c_1 - C_1.at(i));
             d_mean_d1 += max_c_1 -C_1.at(i) ;

            vec_d2.push_back(C_1.at(i) -min_c_1);
            d_mean_d2 += C_1.at(i) -min_c_1;

            vec_d3.push_back(max_c_2 - C_2.at(i));
            d_mean_d3 += max_c_2 -C_2.at(i) ;

            vec_d4.push_back(C_2.at(i) -min_c_2);
            d_mean_d4 += C_2.at(i) -min_c_2;
        }
        else if((C_1.at(i) >= d_cent_x )&&(d_cent_x >= (min_c_1*0.5+max_c_1*0.5)))
        {
            vec_d1.push_back(max_c_1 - C_1.at(i));
             d_mean_d1 += max_c_1 -C_1.at(i) ;

            vec_d2.push_back(C_1.at(i) -min_c_1);
            d_mean_d2 += C_1.at(i) -min_c_1;

            vec_d3.push_back(max_c_2 - C_2.at(i));
            d_mean_d3 += max_c_2 -C_2.at(i) ;

            vec_d4.push_back(C_2.at(i) -min_c_2);
            d_mean_d4 += C_2.at(i) -min_c_2;
        }

         if((C_2.at(i) <= d_cent_y )&&(d_cent_y < (min_c_2*0.5+max_c_2*0.5)))
        {
            vec_d21.push_back(max_c_1 - C_1.at(i));
             d_mean_d21 += max_c_1 -C_1.at(i) ;

            vec_d22.push_back(C_1.at(i) -min_c_1);
            d_mean_d22 += C_1.at(i) -min_c_1;

            vec_d23.push_back(max_c_2 - C_2.at(i));
            d_mean_d23 += max_c_2 -C_2.at(i) ;

            vec_d24.push_back(C_2.at(i) -min_c_2);
            d_mean_d24 += C_2.at(i) -min_c_2;
        }
        else if((C_2.at(i) >= d_cent_y )&&(d_cent_y >= (min_c_2*0.5+max_c_2*0.5)))
        {
            vec_d21.push_back(max_c_1 - C_1.at(i));
             d_mean_d21 += max_c_1 -C_1.at(i) ;

            vec_d22.push_back(C_1.at(i) -min_c_1);
            d_mean_d22 += C_1.at(i) -min_c_1;

            vec_d23.push_back(max_c_2 - C_2.at(i));
            d_mean_d23 += max_c_2 -C_2.at(i) ;

            vec_d24.push_back(C_2.at(i) -min_c_2);
            d_mean_d24 += C_2.at(i) -min_c_2;
        }

    }
    
    d_mean_d1 = d_mean_d1/vec_d1.size();
    d_mean_d2 = d_mean_d2/vec_d1.size();
    d_mean_d3 = d_mean_d3/vec_d1.size();
    d_mean_d4 = d_mean_d4/vec_d1.size();

    d_mean_d21 = d_mean_d21/vec_d21.size();
    d_mean_d22 = d_mean_d22/vec_d21.size();
    d_mean_d23 = d_mean_d23/vec_d21.size();
    d_mean_d24 = d_mean_d24/vec_d21.size();

    double d_var1 = 0,d_var2 = 0,d_var3 = 0,d_var4 = 0;
    double d_var21 = 0,d_var22 = 0,d_var23 = 0,d_var24 = 0;
    for(int i = 0; i < vec_d1.size(); i++)
    {
        d_var1+= std::fabs(vec_d1.at(i) - d_mean_d1);
        d_var2+= std::fabs(vec_d2.at(i) - d_mean_d2);
        d_var3+= std::fabs(vec_d3.at(i) - d_mean_d3);
        d_var4+= std::fabs(vec_d4.at(i) - d_mean_d4);
    }
     for(int i = 0; i < vec_d21.size(); i++)
    {
        d_var21+= std::fabs(vec_d21.at(i) - d_mean_d21);
        d_var22+= std::fabs(vec_d22.at(i) - d_mean_d22);
        d_var23+= std::fabs(vec_d23.at(i) - d_mean_d23);
        d_var24+= std::fabs(vec_d24.at(i) - d_mean_d24);
    }   
     double dist1 = std::min(d_var1,d_var2);
     dist1 = std::min(dist1,d_var3);
     dist1 = std::min(dist1,d_var4);

    double d_num_var1 = vec_d1.size();

     double d_num_var2 = vec_d21.size();  
     double dist2 = std::min(d_var21,d_var22);
     dist2 = std::min(dist2,d_var23);
     dist2 = std::min(dist2,d_var24);
    
     
     
    return (1.0/dist1)*(d_num_var1/(d_num_var1+d_num_var2))+(1.0/dist2)*(d_num_var1/(d_num_var1+d_num_var2));

    std::vector<double> D_1;  // col.4, Algo.4

    std::vector<double> d_1_min,d_1_max;
    double d_1_sum_min = 0,d_1_sum_max = 0;
    for (const auto& c_1_element : C_1)
    {
        const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
   
        D_1.push_back(std::fabs(v));

        d_1_min.push_back(c_1_element - min_c_1);
        d_1_max.push_back(max_c_1 - c_1_element);
        d_1_sum_min += c_1_element - min_c_1;
        d_1_sum_max += max_c_1 - c_1_element;
    }

     std::vector<double> d_2_min,d_2_max;
    double d_2_sum_min = 0,d_2_sum_max = 0;

    std::vector<double> D_2;  // col.5, Algo.4
    for (const auto& c_2_element : C_2)
    {
        const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
       
        D_2.push_back(std::fabs(v));

        d_2_min.push_back(c_2_element - min_c_2);
        d_2_max.push_back(max_c_2 - c_2_element);
        d_2_sum_min += c_2_element - min_c_2;
        d_2_sum_max += max_c_2 - c_2_element;
        
    }    
    
    const double d_min = 0.05;
    const double d_max = 99990.50;
    double       beta = 0;  // col.6, Algo.4


    for (size_t i = 0; i < D_1.size(); ++i)
    {
       double d = 0;
        if(d_1_sum_min < d_1_sum_max)
        {
            d += d_1_min.at(i);
        }
        else
        {
           d += d_1_max.at(i);
        }

          if(d_2_sum_min < d_2_sum_max)
        {
            d += d_2_min.at(i);
        }
        else
        {
           d += d_2_max.at(i);
        }
        
        // const double d =  std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
       beta += 1.0 / d;
    }
    return beta;
}

double ObjectState::CalcVarianceCriterion1(const std::vector<double>& C_1,
                                           const std::vector<double>& C_2)
{
    //std::cout<<"000000000000000000000000000"<<std::endl;
    const double min_c_1 =
        *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double max_c_1 =
        *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double min_c_2 =
        *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    const double max_c_2 =
        *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

    double bary_cent_x = 0,bary_cent_y = 0;
    double area_cent_x = 0.5*(max_c_1+min_c_1) , area_cent_y = 0.5*(max_c_2+min_c_2);
    double short_lenght = std::min((max_c_1-min_c_1) , (max_c_2-min_c_2));
    double long_lenght = std::max((max_c_1-min_c_1) , (max_c_2-min_c_2));
    #pragma omp parallel for  
    for(int i = 0; i < C_1.size(); i++)
    {
            bary_cent_x += C_1.at(i);
            bary_cent_y += C_2.at(i);
    }
    bary_cent_x  = bary_cent_x/C_1.size();
    bary_cent_y  = bary_cent_y/C_1.size();
    //std::cout<<"area===       "<<area_cent_x<<"    "<<area_cent_y<<std::endl;
    //std::cout<<"bary===       "<<bary_cent_x<<"    "<<bary_cent_y<<std::endl;
    double diff_area_bary_x = area_cent_x - bary_cent_x;
    double diff_area_bary_y = area_cent_y - bary_cent_y;
    // double d_cent_x = 0.5*(min_c_1+max_c_1);
    // double d_cent_y = 0.5*(min_c_2+max_c_2);
    std::vector<double> vec_d1,vec_d2,vec_d3,vec_d4;
    double d_mean_d1 = 0, d_mean_d2 = 0, d_mean_d3 = 0, d_mean_d4 = 0;
    std::vector<double> vec_d21,vec_d22,vec_d23,vec_d24;
    double d_mean_d21 = 0, d_mean_d22 = 0, d_mean_d23 = 0, d_mean_d24 = 0;
    double sun_distance = 0;
    int quadrant_num = 0;
    //std::cout<<"real_cent_x========="<<real_cent_x<<"real_cent_y========="<<real_cent_y<<std::endl;
    if(diff_area_bary_x >= 0 )
        {
            if(diff_area_bary_y>=  0)
            {
                
                //std::cout<<"111111111111111111111111111"<<std::endl;
                quadrant_num = 1;
            }
            else
            {
               
                quadrant_num = 2;
            }
        }
        else
        {
            if(diff_area_bary_y>= 0)
            {
                
                quadrant_num = 3;
            }
            else
            {
                
                quadrant_num = 4;
            }
        }
    int sum_points_num = C_1.size();
    int eff_points_num = C_1.size();
    double k_area = (max_c_2-min_c_2)*(max_c_2-min_c_2)*(max_c_1-min_c_1)*(max_c_1-min_c_1)+1;
    int area_1 = 0;
    int area_2 = 0;
    int area_3 = 0;
    int area_4 = 0;
    double sum_nearest_distance = 0;
    double inf_point_x = 0 , inf_point_y = 0; 
    if(quadrant_num == 1)
    {
        #pragma omp parallel for  
        for(int i = 0; i < C_1.size(); i++)
        {  
            if((C_1.at(i) < area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                //点最多区域，取最近距离
                sum_nearest_distance +=  std::min(fabs( C_2.at(i) - min_c_2) ,fabs(C_1.at(i) - min_c_1));
                area_1++;
            }
            else if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                //惩罚区域
                sum_nearest_distance += std::max(fabs( C_2.at(i) - max_c_2) ,fabs(C_1.at(i) - max_c_1));
                area_2++;
            }

            else if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_1.at(i) - min_c_1);
                area_3++;

            }
            else if((C_1.at(i) < area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_2.at(i) - min_c_2);
                area_4++;
            }
        }
    }
    if(quadrant_num == 2)
    {
        #pragma omp parallel for  
        for(int i = 0; i < C_1.size(); i++)
        {  
            if((C_1.at(i) < area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                //点最多区域，取最近距离
                sum_nearest_distance +=  std::min(fabs( C_2.at(i) - max_c_2) ,fabs(C_1.at(i) - min_c_1));
                area_1++;
            }
            else if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                //惩罚区域
                sum_nearest_distance += 2*std::max(fabs( C_2.at(i) - min_c_2) ,fabs(C_1.at(i) - max_c_1));
                area_2++;
            }

            else if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_2.at(i) - max_c_2);
                area_3++;

            }
            else if((C_1.at(i) < area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_1.at(i) - min_c_1);
                area_4++;
            }
        }
    }
    if(quadrant_num == 3)
    {
        #pragma omp parallel for  
        for(int i = 0; i < C_1.size(); i++)
        {  
            if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                //点最多区域，取最近距离
                sum_nearest_distance +=  std::min(fabs( C_2.at(i) - max_c_2) ,fabs(C_1.at(i) - max_c_1));
                area_1++;
            }
            else if((C_1.at(i) < area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                //惩罚区域
                sum_nearest_distance += 2*std::max(fabs( C_2.at(i) - min_c_2) ,fabs(C_1.at(i) - min_c_1));
                area_2++;
            }

            else if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_1.at(i) - max_c_1);
                area_3++;

            }
            else if((C_1.at(i) < area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_2.at(i) - max_c_2);
                area_4++;
            }
        }
    }
    if(quadrant_num == 4)
    {
        #pragma omp parallel for  
        for(int i = 0; i < C_1.size(); i++)
        {  
            if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                //点最多区域，取最近距离
                sum_nearest_distance +=  std::min(fabs( C_2.at(i) - min_c_2) ,fabs(C_1.at(i) - max_c_1));
                area_1++;
            }
            else if((C_1.at(i) < area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                //惩罚区域
                sum_nearest_distance += 2*std::max(fabs( C_2.at(i) - max_c_2) ,fabs(C_1.at(i) - min_c_1));
                area_2++;
            }

            else if((C_1.at(i) >= area_cent_x ) && (C_2.at(i) >= area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_1.at(i) - max_c_1);
                area_3++;

            }
            else if((C_1.at(i) < area_cent_x ) && (C_2.at(i) < area_cent_y ) )
            {
                sum_nearest_distance +=fabs(C_2.at(i) - min_c_2);
                area_4++;
            }
        }
    }
        

    sum_nearest_distance = sum_nearest_distance/double(100) + 1;
    int k_max_min = fabs(area_1-area_2)+1;

    return fabs(k_max_min)/k_area/sum_nearest_distance/sum_nearest_distance;

}


double ObjectState::CalcVarianceCriterion3(const std::vector<double>& C_1,
                                           const std::vector<double>& C_2)
{
    //std::cout<<"000000000000000000000000000"<<std::endl;
    const double min_c_1 =
        *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double max_c_1 =
        *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double min_c_2 =
        *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    const double max_c_2 =
        *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    double k_area = (max_c_2-min_c_2)*(max_c_2-min_c_2)*(max_c_1-min_c_1)*(max_c_1-min_c_1)+0.01;

    return fabs(1)/k_area;

}





double ObjectState::CalcClosenessMCriterion(const std::vector<double>& C_1,
                                  const std::vector<double>& C_2)
{
   const double min_c_1 =
        *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double max_c_1 =
        *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double min_c_2 =
        *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    const double max_c_2 =
        *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

    
    float f_c_piece_c1 = (max_c_1 -min_c_1)/10.0;
    float f_c_piece_c2 = (max_c_2-min_c_2)/10.0;

    Eigen::MatrixXd mat_c1c2 = Eigen::MatrixXd::Zero(C_1.size(),2);
    
    float f_c1_min_temp ,f_c1_max_temp,f_c2_min_temp,f_c2_max_temp;
       std::vector<double> D_1,D_2;
       
    float f_max_scale = 0;
    for (int  j = 0; j < 5; j++)
    {
        f_c1_min_temp = min_c_1+f_c_piece_c1*j;
        f_c1_max_temp = max_c_1-f_c_piece_c1*j;

         f_c2_min_temp = min_c_2+f_c_piece_c2*j;
        f_c2_max_temp = max_c_2-f_c_piece_c2*j;

        D_1.clear(); // col.4, Algo.4
        for (const auto& c_1_element : C_1)
        {
            const double v = std::min(abs(f_c1_min_temp - c_1_element),abs(c_1_element - f_c1_max_temp) );
            D_1.push_back(std::fabs(v));
        }

        D_2.clear();
        // std::vector<double> D_2;  // col.5, Algo.4
        for (const auto& c_2_element : C_2)
        {
            const double v =  std::min(abs(f_c2_min_temp - c_2_element),abs(c_2_element - f_c2_max_temp) );
            D_2.push_back(std::fabs (v));
        }

        const double d_min = 0.05;
        const double d_max = 99990.50;
        double       beta = 0;  // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const double d = 
                std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            beta += 1.0 / d;
        }
        if (beta > f_max_scale)
        {
            f_max_scale = beta;
        }    
    }


    return f_max_scale;
}


}  // namespace detect
}  // namespace lidar
