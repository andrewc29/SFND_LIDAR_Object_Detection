// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "kdtree.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (1.5,1.5,-0.5,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();
    pcl::PointCloud<PointT>::Ptr basePlane = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    
    std::unordered_set<int> inliersResult;
    int maxPoints = 0;
    int currPoints = 0;

    for (int index = 0; index < maxIterations; ++index){
        std::unordered_set<int> indices;
        std::unordered_set<int> tempResult;

        while(indices.size() < 3){
            indices.insert(rand()%(cloud->points.size()));
        }
        auto itr = indices.begin();
        const int first_it = *itr++;
        const int second_it = *itr++;  
        const int third_it = *itr++;

        float x1 = cloud->points[first_it].x;  float y1 = cloud->points[first_it].y;  float z1 = cloud->points[first_it].z;
        float x2 = cloud->points[second_it].x; float y2 = cloud->points[second_it].y; float z2 = cloud->points[second_it].z;
        float x3 = cloud->points[third_it].x;  float y3 = cloud->points[third_it].y;  float z3 = cloud->points[third_it].z;

        float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

        float A = i;
        float B = j;
        float C = k;
        float D = -(i*x1+j*y1+k*z1);

        for (int j = 0; j < cloud->points.size(); ++j){

            if (j == first_it || j == second_it || j == third_it){
                continue;
            }

            float currX = cloud->points[j].x;
            float currY = cloud->points[j].y;
            float currZ = cloud->points[j].z;
            float d = fabs(A*currX+B*currY+C*currZ+D)/(sqrt(pow(A,2)+pow(B,2)+pow(C,2)));

            if (d <= distanceTol){
                tempResult.insert(j);
                ++currPoints;
            }
        }
    
    
        if (tempResult.size() > inliersResult.size()){
            inliersResult = tempResult;
            maxPoints = currPoints;
        }
    }

    
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			basePlane->points.push_back(point);
		else
			filtered->points.push_back(point);
	}


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(basePlane, filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;

}

template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
    processed[index] = true;
	cluster.push_back(index);

	std::vector<int> nearest = tree->search(points[index],distanceTol);

	for (int id: nearest){
		if (!processed[id]){
			clusterHelper(id,points,cluster,processed,tree,distanceTol);
		}
	}
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int minimumClusterSize)
{
    auto startTime = std::chrono::steady_clock::now();

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false);

	int i = 0;
	while(i < points.size()){
		if(processed[i]){
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i,points,cluster,processed,tree,distanceTol);

        //Remove phantom clusters
        if (cluster.size() > minimumClusterSize){
            clusters.push_back(cluster);
        }
		i++;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
 
	return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}