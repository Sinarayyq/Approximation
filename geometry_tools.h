//Lisa Chiang
//This file groups functions representing geometric operations.

#pragma once

#ifndef GEOMETRY_TOOLS_H
#define GEOMETRY_TOOLS_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry> 

#include "cloud_visualizer.h"
#include "io.h"
// It builds the transformation matrix, provided the reference coordinate system. 
// (First column: new axis x coordinates; second column: new axis y coordinates; third column: new axis z coordinates)
// INPUT: the 3 3-dimensional vectors (form library Eigen): axis x, axis y, axis z
// OUTPUT: the 3x3 matrix corresponding to the transformation
Eigen::Matrix3f buildTransformMatrixFromAxis(Eigen::Vector3f x_axis, Eigen::Vector3f y_axis, Eigen::Vector3f z_axis);

// It transforms a given point providing the coordinates referred to a new ref coordinate system.
// INPUT: the 3-dimensional vector (form library Eigen) of the point, the transformation matrix
// OUTPUT: the 3-dimensional vector (form library Eigen) of the transformed point
Eigen::Vector3f transformPointByMatrix(Eigen::Matrix3f transform_matrix, Eigen::Vector3f point_coord);

// It transforms the points in a given cloud by a given transformation matrix
// INPUT: the pointer to the point cloud to transform, the transformation matrix
// OUTPUT: the pointer to the point cloud containing the transformed points
pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud, Eigen::Matrix3f transform_matrix);

// It transforms the points in a given CYLINDRICAL cloud by a given transformation matrix, 
// moving all the points such that z-axis will correspond to the axis of the cylinder
// INPUT: the pointer to the point cloud to transform, the transformation matrix, the cylinder parameters
// OUTPUT: the pointer to the point cloud containing the transformed points
pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix_cylinder(
	pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud, 
	Eigen::Matrix3f transform_matrix, std::vector<float> cyl_param);

#endif