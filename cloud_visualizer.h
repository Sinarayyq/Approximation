//Lisa Chiang
//This functions are used to visualize a point cloud with 3D viewer in PCL.

#pragma once //If it is not added there is an error. It could disappear whenever I use this header (?)

// This is start of the header guard.  READ_PARAMETERS_H can be any unique name.  
// By convention, we use the name of the header file.
#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <string>
#include <iostream>
#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include <Eigen/Geometry> 


#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <QtGui>
#include <QLineF>
#include <QRectF>
#include <QApplication> 
#include <QGraphicsScene>
#include <QGraphicsView> 
#include <pcl/io/pcd_io.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <pcl/point_types.h>


enum camera_position { xy, yz, xz };

// -----Transform Coordinate System-----
pcl::PointCloud<pcl::PointXYZ>::Ptr relocate_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize);

// -----Visualize Chains-----
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string label_viewer_windowm,std::vector<int> hull_index,std::vector<int> cone_chain);
void visualizeShapes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_index, std::vector<int> chain);

// -----Visualize Convex Hull-----
boost::shared_ptr<pcl::visualization::PCLVisualizer> visConvex(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string label_viewer_window, std::vector<int> hull_index);
void visualizeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_indexes);


// -----Visualize Point Cloud-----
boost::shared_ptr<pcl::visualization::PCLVisualizer> visCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string window_label, camera_position camera_pos);
void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, camera_position camera_pos);

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	std::string label_viewer_window,
	camera_position camera_pos);



#endif