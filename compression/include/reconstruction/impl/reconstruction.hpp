namespace EXX{

    template <class T>
    void reconstruction::pclPlaneToCGAL(    typename pcl::PointCloud<T>::Ptr    plane,
                            typename pcl::PointCloud<T>::Ptr    boundary,
                            pcl::ModelCoefficients::Ptr         coeff,
                            std::vector<Point >               & plane_2d,
                            std::vector<Point >               & boundary_2d){


        // Make sure 2d vectors are empty and efficient.
        if(plane_2d.size() != 0){
            plane_2d.clear();
            plane_2d.reserve(plane->points.size());
        }
        if(boundary_2d.size() != 0){
            boundary_2d.clear();
            boundary_2d.reserve(boundary->points.size());
        }

        // Create the normal vector
        Eigen::Vector3f N;
        for (size_t i = 0; i < N.size(); i++) {
            N[i] = coeff->values[i];
        }
        N = N / N.norm();

        // Create arbitrary vector U not parallell to N to find a new X/Y axis
        Eigen::Vector3f U(1,0,0);
        if( std::abs(N.dot(U)) > 0.8 ){
            // U to similar to N change it to [0 1 0].
            U[0] = 0; U[1] = 1;
        }

        // create the new x y axis orthogonal to N.
        Eigen::Vector3f X = U - (U.dot(N) * N);
        Eigen::Vector3f Y = N.cross(X);

        // normalize the axis
        X = X / X.norm();
        Y = Y / Y.norm();

        // Loop through all points and project them to new 2d coordinate system
        Eigen::Vector3f point;
        for(auto p : plane->points){
            point[0] = p.x;
            point[1] = p.y;
            point[2] = p.z;
            plane_2d.push_back( Point( point.dot(X), point.dot(Y) ) );
        }

        for(auto p : boundary->points){
            point[0] = p.x;
            point[1] = p.y;
            point[2] = p.z;
            boundary_2d.push_back( Point( point.dot(X), point.dot(Y) ) );
        }
    }


}
