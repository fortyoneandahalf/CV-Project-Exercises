# include <pcl/io/pcd_io.h>
# include <pcl/point_types.h>
# include <pcl/features/normal_3d.h>
# include <pcl/visualization/cloud_viewer.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/filters/passthrough.h>
# include <pcl/filters/voxel_grid.h>
# include <cmath>
# include <iostream>

int createPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII ("/informatik2/students/home/5bradfie/Documents/CV Project/CV-Project-Exercises/03/CVProj_02_PCL/test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size (); ++i)
      std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return 0;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr createEllipsoidPC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    const float pi = 3.14159265358979f;
    const float two_pi = 2*pi;

    // The ellipsoid center at ( x0 , y0 , z0 )
    const float x0 = 0.0f, y0 = 0.0f, z0 = 0.0f;
    // ellipsoid semi - axes lengths ( for sphere , a = b = c )
    const float a = 1.0f, b = 1.0f, c = 1.0f;
    // theta : azimuthal coordinate , 0 to 2* pi
    // psi : polar coordinate , 0 to pi
    const float azimuthal_resolution = pi / 60.0f;
    const float polar_resolution = pi / 60.0f;

//    pcl :: PointCloud < pcl :: PointXYZ >:: Ptr cloud ( new pcl :: PointCloud < pcl :: PointXYZ >) ;

    for ( float theta = 0.0f ; theta <= two_pi ; theta += azimuthal_resolution )
    {
        for ( float psi =0.0f ; psi <= pi ; psi += polar_resolution )
        {
            const float x = x0 + a * std :: cos ( theta ) * std :: sin ( psi ) ;
            const float y = y0 + b * std :: sin ( theta ) * std :: sin ( psi ) ;
            const float z = z0 + c * std :: cos ( psi ) ;
            cloud -> push_back ( pcl :: PointXYZ (x ,y , z ) ) ;
        }
    }

    std :: cout << " Saving cloud ... " << std :: endl ;
    pcl :: io :: savePCDFileASCII ( "ellipsoid_pcd.pcd" , * cloud ) ;
    std :: cout << " Saved " << cloud -> points . size () << " data points to ellipsoid_pcd.pcd . " << std :: endl ;

    return cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    return cloud;
}


pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    ne.setRadiusSearch (0.05);
    ne.compute(*cloud_normals1);

    return cloud_normals1;
}


int visualisePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // See the output
    pcl :: visualization :: CloudViewer viewer ( " Simple Cloud Viewer " ) ;
    viewer . showCloud ( cloud ) ;
    while (! viewer . wasStopped () )
    { // no - op until viewer stopped
    }
}

int visualisePC2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualiser (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;

        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-10, 10);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_filtered);

//        pass.setInputCloud (cloud_filtered);
//        pass.setFilterFieldName("y");
//        pass.setFilterLimits (-0.5, 0.5);
//        //pass.setFilterLimitsNegative (true);
//        pass.filter (*cloud_filtered);

        return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr segmentPlanar (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

}


int main()
{
//    cloud = createEllipsoidPC(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPCDFile("/informatik2/students/home/5bradfie/Documents/CV Project/CV-Project-Exercises/03/CVProj_02_PCL/cloud.pcd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = passThroughFilter(cloud);

//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1;
//    cloud_normals1 = calculateSurfaceNormals(cloud_filtered, cloud_normals1);

    cloud_filtered = downsample(cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualiser(cloud_filtered);

//    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, cloud_normals1, 10, 0.05, "normals");

//    visualisePC(cloud);
    visualisePC2(cloud_filtered, viewer);

    return 0;
}
