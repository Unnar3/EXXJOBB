#ifndef QUADTREEPCL_H
#define QUADTREEPCL_H

#include <Eigen/Dense>
#include "quadtree.h"


template <typename PointT>
class QuadTreePCL{
    typedef QuadTreePCL qtpcl;

    bool normalVectorSet;
    QuadTree quad;
    Eigen::Vector3f normal_;
    Eigen::Quaternion<float> quaternion;
public:

    QuadTreePCL(int level, float width, float x, float y)
        : quad(QuadTree(level, width, x, y)){
                normalVectorSet = false;
    }
    QuadTreePCL()
        : quad(QuadTree(1,10,0,0)){}
    ~QuadTreePCL(){}

    void insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary);

    template <typename T>
    void createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices);


    void setNormal(Eigen::Vector3f normal);
    Eigen::Vector3f getNormal(void){ return normal_; }


    void setMaxLevel(int level){ quad.setMaxLevel(level); }
    void setMaxWidth(float width){ quad.setMaxWidth(width); }

    void rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud);
private:

};

#include "quadtreePCL.hpp"
#endif
