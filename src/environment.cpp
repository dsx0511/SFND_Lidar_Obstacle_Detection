/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0.0);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);

    // renderPointCloud(viewer, inputCloud, "inputCloud", Color(1,1,1));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
    }
}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display City Block     -----
//     // ----------------------------------------------------

//     // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//     // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-20, -5, -2, 1), Eigen::Vector4f(50, 7, 5, 1));

//     // renderPointCloud(viewer,inputCloud,"inputCloud");

//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);

//     // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//     renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 1000);

//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

//     for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         pointProcessorI->numPoints(cluster);
//         renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % 3]);
//         ++clusterId;

//         Box box = pointProcessorI->BoundingBox(cluster);
//         renderBox(viewer,box,clusterId);
//     }
// }



// To reviewer:
// Sorry for the unclear function name. I already implemented the function below that calls my own ransac and clsutering methods in ´cityBlock2´ in my first submission.
// For clarity, I commented the ´cityBlock´ function which calls the built-in methods, and renamed my own function to ´cityBlockwithOwnMethods´
void cityBlockwithOwnMethods(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-20, -5, -2, 1), Eigen::Vector4f(50, 7, 5, 1));

    std::unordered_set<int> inliersIndices = pointProcessorI->RansacPlane(filteredCloud, 100, 0.2);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int index : inliersIndices)
        inliers->indices.push_back(index);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SeparateClouds(inliers, filteredCloud);

    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<std::vector<float>> points;

    KdTree* tree = new KdTree;
    for(int i = 0; i < segmentCloud.first->points.size(); i++) {
        std::vector<float> point;
        point.push_back(segmentCloud.first->points[i].x);
        point.push_back(segmentCloud.first->points[i].y);
        point.push_back(segmentCloud.first->points[i].z);

        points.push_back(point);
        tree->insert(point, i);
    }

  	std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(points, tree, 0.5, 10);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    for(std::vector<int> cluster : clusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int indice: cluster)
        {
            pcl::PointXYZI clusterPoint;
            clusterPoint.x = points[indice][0];
            clusterPoint.y = points[indice][1];
            clusterPoint.z = points[indice][2];
            clusterPoint.intensity = 1.0f;
  			clusterCloud->points.push_back(clusterPoint);
        }
        cloudClusters.push_back(clusterCloud);
    }

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % 3]);
        ++clusterId;

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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
    // simpleHighway(viewer);

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
        cityBlockwithOwnMethods(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }

    // cityBlock(viewer);

    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // }
}