#ifndef QUADTREEPCL_H
#define QUADTREEPCL_H

#include <Eigen/Dense>
#include "quadtree.h"


template <typename PointT>
class QuadTreePCL{
    typedef QuadTreePCL qtpcl;

    bool normalVectorSet;
    bool inserted_;
    QuadTree quad;
    Eigen::Vector3f normal_;
    Eigen::Quaternion<float> quaternion_;
    float z_;
public:

    QuadTreePCL(int level, float width, float x, float y){
                normalVectorSet = false;
                inserted_ = false;
    }
    QuadTreePCL()
        :  QuadTreePCL(1,0,0,0){}
    ~QuadTreePCL(){}

    void insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary);

    template <typename T>
    void createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices);


    void setNormal(Eigen::Vector3f normal);
    Eigen::Vector3f getNormal(void){ return normal_; }


    void setMaxLevel(int level){ quad.setMaxLevel(level); }
    void setMaxWidth(float width){ quad.setMaxWidth(width); }

private:
    void rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud);
    void rotateFromAxis(typename pcl::PointCloud<PointT>::Ptr cloud);

    int roundDown(float toRound){
        int tmp = std::abs(std::floor(toRound));
        if(toRound >= 0){
            return tmp - tmp % 10;
        } else {
            return (-10 + tmp % 10) - tmp;
        }
    }

    int roundUp(float toRound){
        int tmp = std::abs(std::ceil(toRound));
        if(toRound >= 0){
            return (10 - tmp % 10) + tmp;
        } else {
            return -tmp + tmp % 10;
        }
    }
};

#include "quadtreePCL.hpp"
#endif
