/* \author V Sai Subba Rao */
// Implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
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
            if(dist < distanceTol)
                inliersTmp.insert(index);
        }
        // Check and store the set of highest inliers 
        if(inliersTmp.size() > inliersResult.size())
            inliersResult = inliersTmp;

	}

	// Return indicies of inliers from fitted line with most inliers	

	std::cout<<"Number of inliers is "<<inliersResult.size()<<std::endl;
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// Max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 20, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
