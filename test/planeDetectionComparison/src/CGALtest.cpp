#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/for_each_type.h>
#include <pcl/point_traits.h>
#include <pcl/visualization/cloud_viewer.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <planeDetectionComparison/cgalTriangulation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                    Delaunay;
typedef Kernel::Point_2                                                Point;

typedef CGAL::Constrained_triangulation_face_base_2<Kernel>              Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                 TDS;
typedef CGAL::Exact_predicates_tag                                  Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag>    CDT;
typedef CGAL::Spatial_sort_traits_adapter_2<Kernel,Point*> Search_traits;

using namespace std;

float createPlaneZ(float x, float y){
    float a,b,c,d;
    a = 1;
    b = 2;
    c = 3;
    d = 2;
    return -(a*x+b*y+d)/c;
}

template<typename PointT> void
toMeshCloud (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
{
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
    {
      msg.width  = static_cast<uint32_t>(cloud.points.size ());
      msg.height = 1;
    }
    else
    {
      assert (cloud.points.size () == cloud.width * cloud.height);
      msg.height = cloud.height;
      msg.width  = cloud.width;
    }

    // Fill point cloud binary data (padding and all)
    size_t data_size = sizeof (PointT) * cloud.points.size ();
    msg.data.resize (data_size);
    memcpy (&msg.data[0], &cloud.points[0], data_size);

    // Fill fields metadata
    msg.fields.clear ();
    pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT>(msg.fields));

    msg.header     = cloud.header;
    msg.point_step = sizeof (PointT);
    msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;
}

int main(int argc, char **argv) {
//
    ros::init(argc, argv,"test");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::Rate loop_rate(10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients());
    coeff->values.resize(3);
    coeff->values[0] = 1;
    coeff->values[1] = 2;
    coeff->values[2] = 3;

    std::vector<Point> plane_2d;
    std::vector<Point> boundary_2d;

    pcl::PointXYZ p;
    for (size_t i = 0; i < 10; i++) {
        for (size_t j = 0; j < 10; j++) {
            p.x = i;
            p.y = j;
            p.z = createPlaneZ(i,j);
            cloud->points.push_back(p);
        }
    }

    pclPlaneToCGAL<pcl::PointXYZ>(cloud, boundary, coeff, plane_2d, boundary_2d);
    std::vector<std::vector<unsigned int> > idx;
    std::cout << "hmmmmm" << std::endl;
    constrainedDelaunayTriangulation(plane_2d, boundary_2d, idx);

    pcl::PolygonMesh mesh;
    toMeshCloud(*cloud, mesh.cloud);


    pcl::Vertices vert;
    vert.vertices.resize(3);
    for(auto poly : idx){
        vert.vertices[0] = poly[0];
        vert.vertices[1] = poly[1];
        vert.vertices[2] = poly[2];
        mesh.polygons.push_back(vert);
        std::cout << poly[0] << ", " << poly[1] << ", " << poly[2] << std::endl;
        std::cout << "-----------------------" << std::endl;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh,"meshes",0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){s
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }



  // std::vector<Point> points;
  // points.push_back(Point(1,1));
  // points.push_back(Point(2,1));
  // points.push_back(Point(3,1));
  // points.push_back(Point(4,1));
  // points.push_back(Point(5,1));
  // points.push_back(Point(1,2));
  // points.push_back(Point(2,2));
  // points.push_back(Point(3,2));
  // points.push_back(Point(4,2));
  // points.push_back(Point(5,2));
  // points.push_back(Point(1,3));
  // points.push_back(Point(2,3));
  // points.push_back(Point(1,4));
  // points.push_back(Point(2,4));
  // points.push_back(Point(4,3));
  // points.push_back(Point(5,3));
  // points.push_back(Point(4,4));
  // points.push_back(Point(5,4));
  //
  // std::vector<Point> constrained;
  // constrained.push_back(Point(2.5,4));
  // constrained.push_back(Point(2.5,3));
  // constrained.push_back(Point(3,3));
  // constrained.push_back(Point(3.5,3));
  // constrained.push_back(Point(3.5,4));
  //
  // std::vector<std::vector<unsigned int> > idx;
  // constrainedDelaunayTriangulation(points, constrained, idx);


  // for(auto poly : idx){
  //     std::cout << poly[0] << ", " << poly[1] << ", " << poly[2] << std::endl;
  //     std::cout << "-----------------------" << std::endl;
  // }


}
