#ifndef QUADTREEPCL_H
#define QUADTREEPCL_H

#include <Eigen/Dense>
#include "quadtree.h"


template <typename PointT>
class QuadTreePCL{
    typedef QuadTreePCL qtpcl;

    QuadTree quad;
    Eigen::Vector3f normal_;
    int k;
public:
//
    QuadTreePCL();
    QuadTreePCL(int level, float width, float x, float y);
    ~QuadTreePCL(){}
//
//     void insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary);
//
//     template <typename T>
//     void createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices);
//
//
//     void setNormal(Eigen::Vector3f normal){ normal_ = normal; }
//     Eigen::Vector3f getNormal(void){ return normal_; }
//
//
// private:
//     template <typename T>
//     void createMesh(typename qt::QuadTree &node, typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices);
//     // void colorBasedOnCellType(PointT &p);
// };

#include "quadtreePCL.hpp"
#endif
