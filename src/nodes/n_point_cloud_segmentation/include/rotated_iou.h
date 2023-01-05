
#ifndef ROTATED_IOU_H
#define ROTATED_IOU_H                       
#pragma once
namespace lidar
{
namespace rotated
{
        float Rotated_Iou(const float* box_a, const float* box_b);   
        //int Calculate(int a,int b);
}
}
#endif