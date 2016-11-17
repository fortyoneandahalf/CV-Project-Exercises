//#include "cvprojex05.h"


//pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(const std::string& path)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1) //* load the file
//    {
//      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//    }
//    std::cout << "Loaded "
//              << cloud->width * cloud->height
//              << " data points from test_pcd.pcd with the following fields: "
//              << std::endl;

//    return cloud;
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualiser ()
//{
//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
////  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//  return (viewer);
//}


//int visualisePC2(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
//{
//    while (!viewer->wasStopped ())
//    {
//      viewer->spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
//}
