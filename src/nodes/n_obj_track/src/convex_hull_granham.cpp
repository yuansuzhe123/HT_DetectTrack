#include "convex_hull_granham.h"


namespace algorithm
{
namespace convex
{
    POINT ConvexHull::p_0;

    ConvexHull::ConvexHull()
    {
    }

    ConvexHull::~ConvexHull()
    {
    }

    double ConvexHull::cross_product(POINT p0, POINT p1, POINT p2)
    {
        return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
    }

    double ConvexHull::Distance(POINT a,POINT b)
    {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }

    int ConvexHull::cmp(POINT a, POINT b)
    {
        double temp = cross_product(p_0, a, b);
        if (fabs(temp) < 1e-6)//极角相等按照距离从小到大排序
        {
            return Distance(p_0, a) < Distance(p_0, b);
        }
        else
        {
            return temp > 0;
        } 
    }

    void ConvexHull::GetConvetHullPts(msg_obj_perception::Obj& obj)
    {
        int contour_point_num = obj.grid_points.size();
        if(contour_point_num <= 3)
        {
            for(int i = 0 ; i < contour_point_num ; i++ )
            {
                msg_common::GridPoint hull_p;
                hull_p.x = obj.grid_points[i].x;
                hull_p.y = obj.grid_points[i].y;
                hull_p.z = obj.grid_points[i].z;
                obj.hull_points.push_back(hull_p);
            }
            return;
        }

        if(contour_point_num > 50000)
        {
            std::cout << "point size > 50000" << std::endl;
            return;
        }

        int k = 0;
        obj_min_z = obj.grid_points[0].z;
        for (auto tem_point : obj.grid_points)
        {
            p[k].x = tem_point.x;
            p[k].y = tem_point.y;
            p[k].z = tem_point.z;

            if(obj_min_z > tem_point.z)
            {
                obj_min_z = tem_point.z;
            }

            k++;
        }

        vector<POINT> ch;
        int top = 2;
        int index = 0;
        for (int i = 1; i < contour_point_num; ++i)//选出Y坐标最小的点，若Y坐标相等，选择X坐标小的点
        {
            if (p[i].y < p[index].y || (p[i].y == p[index].y && p[i].x < p[index].x))
            {
                index = i;
            }
        }

        swap(p[0], p[index]);
        p_0 = p[0];
        ch.push_back(p[0]);
        //按极角排序
        sort(p + 1, p + contour_point_num, cmp);
        ch.push_back(p[1]);
        ch.push_back(p[2]);
        for (int i = 3; i < contour_point_num; ++i)
        {
            while (top > 0 && cross_product(ch[top - 1], p[i], ch[top]) >= 0)
            {
                --top;
                ch.pop_back();
            }
            ch.push_back(p[i]);
            ++top;
        }

        //输出
        //obj.grid_points.clear(); 
        for(int i = 0; i < ch.size(); i++)
        {
            // msg_common::GridPoint points;

            // points.x = ch[i].x;
            // points.y = ch[i].y;
            // points.z = ch[i].z;

            // points.height = points.z - obj_min_z + 0.3;
            // points.width = 0.3;
            // points.length = 0.3;

            // obj.grid_points.push_back(points);


            double dis = Distance(ch[i], ch[(i + 1) % ch.size()]);
            int num = 1;
            if(dis > 1)
            {
                num = (int)(dis / 2) + 1;
            }

            double delt_x = (ch[(i + 1) % ch.size()].x - ch[i].x) / num;
            double delt_y = (ch[(i + 1) % ch.size()].y - ch[i].y) / num;
            double delt_z = (ch[(i + 1) % ch.size()].z - ch[i].z) / num;

            //插值处理
            for(int j = 0; j < num; j++)
            {
                msg_common::GridPoint points;

                points.x = ch[i].x + j * delt_x;
                points.y = ch[i].y + j * delt_y;
                points.z = ch[i].z + j * delt_z;

                points.height = points.z - obj_min_z + 0.2;
                points.width = 0.2;
                points.length = 0.2;

                obj.hull_points.push_back(points);
            }//for 插值
            
        }//for

    }


}
}
