#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>

#include <iostream>

//#include "cvprojex05.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(const std::string& path)
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


void removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
}


void downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualiser ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}


int visualisePC2(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main()
{
    // Helper object for this exercise; contains functions to be used
//    CVProjEx05 helper;

    // Read in PCD files
    std::string path1 ("/informatik2/students/home/5bradfie/Documents/CV Project/CV-Project-Exercises/05/cloud001.pcd");
    std::string path2 ("/informatik2/students/home/5bradfie/Documents/CV Project/CV-Project-Exercises/05/cloud002.pcd");

    std::cout << path1 << std::endl << path2 << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
    cloud1 = readPCDFile(path1);
    cloud2 = readPCDFile(path2);

//    std::cout << helper.test << std::endl;

    //TODO Check for error in reading files and exit.

    //TODO Filter clouds to remove NAN
    removeNANs(cloud1);
    removeNANs(cloud2);

    //TODO Downsample clouds
    downsample(cloud1);
    downsample(cloud2);

    //TODO Align using ICP (1) without initial guess and (2) with initial guess
    //TODO Output convergence, fitness score and final transformation

    // Visualize clouds
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualiser();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud1, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud1, red, "Cloud 1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud2, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, green, "Cloud 2");

    visualisePC2(viewer);

    return 0;
}

