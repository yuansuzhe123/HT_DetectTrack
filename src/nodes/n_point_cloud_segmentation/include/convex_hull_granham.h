#ifndef      _CONVEX_HULL_GRANHAM_H_
#define     _CONVEX_HULL_GRANHAM_H_

#include<cstdio>
#include<cmath>
#include<algorithm>
#include<cstring>
#include<iostream>
#include "msg_obj_perception/Obj.h"
#include "msg_obj/Obj.h"
#include "msg_common/GridPoint.h"

namespace algorithm
{
namespace convex
{

using namespace std;

typedef struct
{
    double x;
    double y;
    double z;
} POINT;


class ConvexHull
{
public:
    ConvexHull();
    ~ConvexHull();

    void GetConvetHullPts(msg_obj_perception::Obj& obj);

private:
    //叉乘
    static double cross_product(POINT p0, POINT p1, POINT p2);
    //求距离
    static double Distance(POINT a,POINT b);
    static int cmp(POINT a, POINT b);

    static POINT p_0;
    
    POINT p[50000];
    double obj_min_z;
};

}
}


#endif
