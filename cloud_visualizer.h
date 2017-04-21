//Lisa Chiang
//This functions are used to visualize a point cloud with 3D viewer in PCL.

#pragma once //If it is not added there is an error. It could disappear whenever I use this header (?)

// This is start of the header guard.  READ_PARAMETERS_H can be any unique name.  
// By convention, we use the name of the header file.
#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <boost/format.hpp>
#include <QtGui>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <QLineF>
#include <QRectF>
#include <QApplication> 
#include <QGraphicsScene>
#include <QGraphicsView> 

// It is a tool to visualize the given point cloud
// INPUT: the point cloud
// OUTPUT: a pointer to a visualization object
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

// It is a tool to visualize the given point cloud (RGB)
// INPUT: the point cloud
// OUTPUT: a pointer to a visualization object
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string label_viewer_windowm,std::vector<int> hull_index,std::vector<int> cone_chain);

void visualizeTwoPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
							 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, 
							 std::string label_viewer_window);

// It visualizes the given point cloud
// INPUT: the point cloud, the title of the viewer window
void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window);
void visualizeShapes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window,std::vector<int> hull_index,std::vector<int> chain);
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesconvex(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string label_viewer_window, std::vector<int> hull_index);
void visualizeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_indexes);


#endif