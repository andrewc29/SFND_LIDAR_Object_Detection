/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{

    //Reduce cloud resolution for throughput
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, Eigen::Vector4f (-15, -6, -2, 1), Eigen::Vector4f (15, 6, 2, 1));

    //Segment cloud into ground and obstacles using RANSAC
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 200, 0.15);
    renderPointCloud(viewer,segmentedCloud.first,"planeCloud",Color(0,1,0));

    //Build KdTree
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> obstPoints;
    for (int i=0; i<segmentedCloud.second->points.size(); i++)
    {
        std::vector<float> point = {segmentedCloud.second->points[i].x,
                                    segmentedCloud.second->points[i].y,
                                    segmentedCloud.second->points[i].z};
        obstPoints.push_back(point);
        tree->insert(point,i); 
    }

    //Generate Clusters
    std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(obstPoints, tree, 0.4, 5);

    // Render clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(std::vector<int> cluster : clusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int indice: cluster)
            clusterCloud->points.push_back(pcl::PointXYZI(obstPoints[indice][0],obstPoints[indice][1],obstPoints[indice][2],1));
        renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = pointProcessorI->BoundingBox(clusterCloud);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    const std::string dataFilePath = "../src/sensors/data/pcd/data_1/";

    std::cout << dataFilePath << std::endl;


   ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
   std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
   auto streamIterator = stream.begin();
   pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
        
    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();

    viewer->spinOnce ();
    }

}