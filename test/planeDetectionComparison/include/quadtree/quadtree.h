#ifndef QUADTREE_H
#define QUADTREE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/Vertices.h>

template <typename PointT>
class QuadTree{
    float x_, y_;
    float max_width_;
    float width_;
    int max_level_;
    int level_;
    bool use_color_;
    bool is_leaf_;
    bool is_boundary_;
    std::vector<QuadTree<PointT> > nodes;
    std::vector<PointT> points;
public:
    QuadTree(){};
    QuadTree(int level, float width, float x, float y);

    ~QuadTree(){}

    int insert(PointT point, bool boundary);

    // Determines if color information will be used in decemation.
    void useColor(bool use_color);
    void setMaxLevel(int level);
    void setMaxWidth(float width);
    void isLeaf(bool is_leaf);
    void isBoundary(bool is_boundary);

    // returns if color information will be used in plane decemation.
    bool    useColor()     { return use_color_; }
    float   getMaxWidth()  { return max_width_; }
    float   getWidth()     { return width_; }
    int     getMaxLevel()  { return max_level_; }
    int     getLevel()     { return level_;}
    bool    isLeaf()       { return is_leaf_;}
    bool    isBoundary()   { return is_boundary_;}
    typename std::vector<PointT>::iterator    pointsBegin()  { return points.begin();}
    typename std::vector<PointT>::iterator    pointsEnd()    { return points.end();}

    void clear();
    void printTree(std::vector<int> idx);
    int getTreeDepth();
    bool decemate();
    void createPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector< pcl::Vertices > &vertices);

private:
    bool shouldCreateSubNodes();
    void createSubNodesAndInherit();
    int returnQuadrant(PointT p);
};

#include "quadtree.hpp"
#endif
