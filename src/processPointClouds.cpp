// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
// #include "quiz/ransac/ransac3d.cpp"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roof_removed (new pcl::PointCloud<PointT>());
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);
    
    // create crop box object
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    // Removing the roof points
    pcl::CropBox<PointT> roof(true);
    std::vector<int> inlierIndices;
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    roof.setInputCloud(cloud_region);
    roof.setNegative(true);
    roof.filter(*roof_removed);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roof_removed;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>()); 
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    
	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliers.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
    
	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
	    std::unordered_set<int> inliersTmp;

        // Randomly sample subset and fit line
        while(inliersTmp.size()<3)
            inliersTmp.insert(rand() % cloud->points.size());

        auto itr = inliersTmp.begin();
			
        auto point1 = cloud->points[*itr++];
        auto point2 = cloud->points[*itr++];
        auto point3 = cloud->points[*itr];

        // Plane: ax + by + cz + d = 0
        Eigen::Vector3f v1, v2, v_normal;

        v1 << point2.x - point1.x, point2.y - point1.y, point2.z - point1.z; 
        v2 << point3.x - point1.x, point3.y - point1.y, point3.z - point1.z; 
        
        v_normal = v1.cross(v2); 

        float a = v_normal(0); 
        float b = v_normal(1); 
        float c = v_normal(2); 
        float d = -(v_normal(0)*point1.x + v_normal(1)*point1.y + v_normal(2)*point1.z); 

        // Measure distance between every point and fitted line
        // Distance between a point and a line = |ax + by + c|/sqrt(a^2 + b^2)
        float denom = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

        for(int index=0 ; index < cloud->points.size() ; index++)
        {
            auto point = cloud->points[index];
            float dist = fabs(a * point.x + b * point.y + c*point.z + d) / denom ;

            // If distance is smaller than threshold count it as inlier
            if(dist < distanceThreshold)
                inliersTmp.insert(index);
        }
        // Check and store the set of highest inliers 
        if(inliersTmp.size() > inliersResult.size())
            inliersResult = inliersTmp;

	}

    if (inliersResult.size () == 0)
    {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
  
    for (int i=0; i<cloud->points.size(); i++){
        std::vector<float> tmp{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    	tree->insert(tmp, i); 
        points.push_back(tmp);
    }
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
  	std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, 0.8);

    for (int i=0; i<clusterIndices.size(); i++){
        typename pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
        for (int index : clusterIndices[i]){
            auto point = cloud->points[index];
            tmp->points.push_back(point);
        }
        clusters.push_back(tmp);
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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
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
template<typename PointT>
void  ProcessPointClouds<PointT>::proximity(std::vector<std::vector<float>>points, int i, std::vector<bool> &processed, std::vector<int> &cluster, KdTree* &tree, float distanceTol)
{
	if (!processed[i])
	{
		processed[i] = true;
		cluster.push_back(i);
		
		std::vector<int> near_points = tree->search(points[i], distanceTol);

		for (int j : near_points)
		{
			if (!processed[j])
				proximity(points, j, processed, cluster, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// Return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	for (int i=0; i < points.size(); i++)
	{
		if (!processed[i])
		{
			std::vector<int> cluster;
			proximity(points, i, processed, cluster, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
	return clusters;

}