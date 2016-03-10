#include <iomanip>


struct myseps : std::numpunct<char> {
   /* use space as separator */
   // char do_thousands_sep() const { return '.'; }

   /* digits are grouped by 3 digits each */
   std::string do_grouping() const { return "\000"; }

   char do_decimal_point() const { return ','; }
};

template <typename T>
struct objStruct{
    std::vector<typename pcl::PointCloud<T>::Ptr>   clouds;
    std::vector<std::vector<pcl::Vertices>>         polygons;
    std::vector<std::vector<Eigen::Vector2f>>       texture_vertices;
    std::vector<pcl::ModelCoefficients::Ptr>        coefficients;
    std::vector<pcl::TexMaterial>                   materials;
    std::vector<cv::Mat>                            images;

    objStruct(int size){
        clouds.reserve(size);
        polygons.reserve(size);
        texture_vertices.reserve(size);
        coefficients.reserve(size);
        materials.reserve(size);
        images.reserve(size);
    }
};

template <typename T>
int
saveOBJFile (   const std::string                           &file_name,
                typename pcl::PointCloud<T>::Ptr            cloud,
                std::vector<std::vector<pcl::Vertices>>     &polygons,
                std::vector<std::vector<Eigen::Vector2f>>   &texture_vertices,
                std::vector<pcl::ModelCoefficients::Ptr>    &coefficients,
                std::vector<pcl::TexMaterial>               &materials,
                unsigned                                    precision = 5)
{
  if (cloud->size() == 0)
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.imbue(std::locale(std::locale(), new myseps));
  fs.precision (precision);
  fs.setf(ios::fixed);
  fs.open (file_name.c_str ());


  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = cloud->size();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    fs << "v ";
    fs << cloud->points[i].x << " ";
    fs << cloud->points[i].y << " ";
    fs << cloud->points[i].z << std::endl;
  }

  fs << "# Normals" << std::endl;
  for(int i = 0; i < coefficients.size(); ++i){
    //   if(coefficients[i]->values.size() != 3){
    //       PCL_ERROR ("[pcl::io::saveOBJFile] Normals of wrong size!\n");
    //       return (-2);
    //   }
      fs << "vn ";
      fs << coefficients[i]->values[0] << " ";
      fs << coefficients[i]->values[1] << " ";
      fs << coefficients[i]->values[2] << std::endl;
  }

  for(int i = 0; i < texture_vertices.size(); ++i){
      fs << "# Texture Vertices for submesh " << i << std::endl;
      for(int j = 0; j < texture_vertices[i].size(); ++j){
          fs << "vt ";
          fs << texture_vertices[i][j][0] << " ";
          fs << texture_vertices[i][j][1] << std::endl;
      }
  }

  for(int i = 0; i < polygons.size(); ++i){
      fs << "# Faces Vertices for submesh " << i << std::endl;
      fs << "usemtl " << materials[i].tex_name << std::endl;
      for(int j = 0; j < polygons[i].size(); ++j){
          fs << "f ";
          fs << polygons[i][j].vertices[0]+1 << "/" << polygons[i][j].vertices[0]+1 << "/" << i+1 << " ";
          fs << polygons[i][j].vertices[1]+1 << "/" << polygons[i][j].vertices[1]+1 << "/" << i+1 << " ";
          fs << polygons[i][j].vertices[2]+1 << "/" << polygons[i][j].vertices[2]+1 << "/" << i+1 << std::endl;
      }

  }
  fs << "# End of File";
  //
  // // Close obj file
  PCL_INFO ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  PCL_INFO ("Writing material files\n");
  //dont do it if no material to write
  // if(tex_mesh.tex_materials.size() ==0)
  //   return (0);

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < materials.size(); ++m)
  {
    m_fs << "newmtl " << materials[m].tex_name << std::endl;
    m_fs << "Ka "<< materials[m].tex_Ka.r << " " << materials[m].tex_Ka.g << " " << materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< materials[m].tex_Kd.r << " " << materials[m].tex_Kd.g << " " << materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< materials[m].tex_Ks.r << " " << materials[m].tex_Ks.g << " " << materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return (0);
}


template <typename T>
int
saveOBJFile ( const std::string &file_name,  objStruct<T> &object, unsigned precision = 5 )
{

    // general guard. Check if all vectors the same size.
    int size = object.clouds.size();
    std::cout << "cloud size(): " << size << std::endl;
    if(size == 0){
        PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
        return (-1);
    }
    if( size != object.polygons.size()          ||
        size != object.texture_vertices.size()  ||
        size != object.coefficients.size()      ||
        size != object.images.size()){
        //                 ||
        // size != object.materials.size()){

        PCL_ERROR ("[pcl::io::saveOBJFile] Vectors are not of the same size!\n");
        return (-1);
    }

    // Create a vector that containes integers with pointsizes so far.
    std::vector<int> sizes(object.clouds.size());
    std::cout << "sizes: ";
    for(int i = 0; i < sizes.size(); ++i){
        if(i == 0) sizes[i] = 0;
        else{
            sizes[i] = sizes[i-1] + object.clouds[i-1]->size();
            std::cout << sizes[i] << ", ";
        }
    }
    std::cout << " " << std::endl;
    // Open file
    std::ofstream fs;
    fs.imbue(std::locale(std::locale(), new myseps));
    fs.precision (precision);
    fs.setf(ios::fixed);
    fs.open (file_name.c_str ());


    // Define material file
    std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
    // Strip path for "mtllib" command
    std::string path = file_name.substr (0, file_name.find_last_of ("/"));
    std::string mtl_file_name_nopath = mtl_file_name;
    mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);
  //
  // /* Write 3D information */
  // // number of points
  // int nr_points  = cloud->size();
  //
      // Write the header information
    fs << "####" << std::endl;
    fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
    fs << "mtllib " << mtl_file_name_nopath << std::endl;
    fs << "####" << std::endl;

    // Write vertex coordinates
    fs << "# Vertices" << std::endl;
    for(int i = 0; i < size; ++i){
        for(int j = 0; j < object.clouds[i]->size(); ++j){
            fs << "v ";
            fs << object.clouds[i]->at(j).x << " ";
            fs << object.clouds[i]->at(j).y << " ";
            fs << object.clouds[i]->at(j).z << std::endl;
        }
    }
  //
    fs << "# Normals" << std::endl;
    for(int i = 0; i < object.coefficients.size(); ++i){
        // if(object.coefficients[i]->values.size() != 3){
        //       PCL_ERROR ("[pcl::io::saveOBJFile] Normals of wrong size!\n");
        //       return(-2);
        // }
        fs << "vn ";
        fs << object.coefficients[i]->values[0] << " ";
        fs << object.coefficients[i]->values[1] << " ";
        fs << object.coefficients[i]->values[2] << std::endl;
    }

    for(int i = 0; i < object.texture_vertices.size(); ++i){
        fs << "# Texture Vertices for submesh " << i << std::endl;
        for(int j = 0; j < object.texture_vertices[i].size(); ++j){
            fs << "vt ";
            fs << object.texture_vertices[i][j][0] + sizes[i] << " ";
            fs << object.texture_vertices[i][j][1] + sizes[i] << std::endl;
        }
    }

    for(int i = 0; i < object.polygons.size(); ++i){
        fs << "# Faces Vertices for submesh " << i << std::endl;
        fs << "usemtl " << "material_" << i << std::endl;
        for(int j = 0; j < object.polygons[i].size(); ++j){
            fs << "f ";
            fs << object.polygons[i][j].vertices[0]+1 + sizes[i] << "/" << object.polygons[i][j].vertices[0]+1 + sizes[i] << "/" << i+1 << " ";
            fs << object.polygons[i][j].vertices[1]+1 + sizes[i] << "/" << object.polygons[i][j].vertices[1]+1 + sizes[i] << "/" << i+1 << " ";
            fs << object.polygons[i][j].vertices[2]+1 + sizes[i] << "/" << object.polygons[i][j].vertices[2]+1 + sizes[i] << "/" << i+1 << std::endl;
        }
    }
  fs << "# End of File";

  // Close obj file
  PCL_INFO ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  PCL_INFO ("Writing material files\n");
  //dont do it if no material to write
  // if(tex_mesh.tex_materials.size() ==0)
  //   return (0);

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < object.images.size(); ++m){
      std::string image_path = path + "/texture_" + std::to_string(m) + ".png";
      cv::imwrite(image_path, object.images[m]);
        m_fs << "newmtl " << "material_" << m << std::endl;
        m_fs << "Ka "<< "0.2 0.2 0.2" << std::endl; // defines the ambient color of the material to be (r,g,b).
        m_fs << "Kd "<< "0.8 0.8 0.8" << std::endl; // defines the diffuse color of the material to be (r,g,b).
        m_fs << "Ks "<< "1.0 1.0 1.0" << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
        m_fs << "d " << "1" << std::endl; // defines the transparency of the material to be alpha.
        m_fs << "Ns "<< "1"  << std::endl; // defines the shininess of the material to be s.
        m_fs << "illum "<< "2" << std::endl; // denotes the illumination model used by the material.
        // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
        // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
        m_fs << "map_Kd " << image_path << std::endl;
        m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return (0);
}
