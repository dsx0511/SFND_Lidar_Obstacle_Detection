/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	auto startTime = std::chrono::steady_clock::now();
	
	// TODO: Fill in this function
	int numPoints = cloud->size();

	// For max iterations 
	for (int i=0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line
		int firstIndex = rand() % numPoints;
		int secondIndex;
		do
		{ secondIndex = rand() % numPoints; }
		while (secondIndex == firstIndex);

		pcl::PointXYZ point1 = cloud->points[firstIndex];
		pcl::PointXYZ point2 = cloud->points[secondIndex];

		double A = point1.y - point2.y;
		double B = point2.x - point1.x;
		double C = point1.x * point2.y - point2.x * point1.y;

		// Measure distance between every point and fitted line
		std::unordered_set<int> inliersTmp;
		for (int j=0; j < numPoints; j++)
		{
			pcl::PointXYZ point = cloud->points[j];
			double d = fabs(A * point.x + B * point.y + C) / sqrt(A * A + B * B);

			// If distance is smaller than threshold count it as inlier
			if (d < distanceTol) 
			{ inliersTmp.insert(j); }
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliersTmp.size() > inliersResult.size())
		{   inliersResult = inliersTmp; 
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	auto startTime = std::chrono::steady_clock::now();
	
	// TODO: Fill in this function

	// For max iterations 
	for (int i=0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand() % cloud->size());

		pcl::PointXYZ p1, p2, p3;

		auto itr = inliers.begin();
		p1 = cloud->points[*itr];
		itr++;
		p2 = cloud->points[*itr];
		itr++;
		p3 = cloud->points[*itr];

		float A = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
		float B = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
		float C = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
		float D = -(A*p1.x+B*p1.y+C*p1.z);

		// Measure distance between every point and fitted line
		for (int j=0; j < cloud->points.size(); j++)
		{
			if (inliers.count(j) > 0)
				continue;

			pcl::PointXYZ point = cloud->points[j];
			float d = fabs(A*point.x+B*point.y+C*point.z+D)/sqrt(A*A+B*B+C*C);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol) 
				inliers.insert(j);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers; 
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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
