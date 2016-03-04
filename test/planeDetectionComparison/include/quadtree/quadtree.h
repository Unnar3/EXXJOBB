#ifndef QUADTREE_H
#define QUADTREE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/Vertices.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K>                                  Polygon;

template <typename PointT>
class QuadTree{

    enum CellType{ Interior, Boundary, Exterior, Hole, Parent};

    float x_, y_;
    float max_width_;
    float width_;
    int max_level_;
    int level_;
    bool use_color_;
    bool is_leaf_;
    bool is_boundary_;
    CellType cellType;
    std::vector<QuadTree<PointT> > nodes;
    std::vector<PointT> points;
public:
    QuadTree(){};
    QuadTree(int level, float width, float x, float y);

    ~QuadTree(){}

    int insert(PointT point, bool boundary);
    bool insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary);
    bool insertBoundary(Polygon polygon);
    void insertHole(Polygon polygon);

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
    void createPointCloudBoundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector< pcl::Vertices > &vertices);

private:
    bool maxDepth();
    void createSubNodesAndInherit();
    int returnQuadrant(PointT p);
    bool polygonCompletelyWithinCell(Polygon &polygon);
    void setCellType(CellType type){ cellType = type; }
};

#include "quadtree.hpp"
#endif
