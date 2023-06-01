//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

// 判断是否相交
struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;      // 是否相交
    Vector3f coords;    // 相交点的坐标
    Vector3f normal;    // 相交点的法向量
    double distance;    // 相交点和相机的距离
    Object* obj;        // 相交的物体
    Material* m;        // 相交物体的材质
};
#endif //RAYTRACING_INTERSECTION_H
