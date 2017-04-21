#pragma once

#ifndef PATCH_RECOGNITION_H
#define PATCH_RECOGNITION_H

#include "read_parameters.h"
#include "border_definition.h"
#include "segmentation.h"
#include <iostream>
#include <time.h>
#include <direct.h>
#include <string.h>
#include <deque>
#include <iterator>
#include <list>

#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



void PlaneRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, 
	                  int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

void CylinderRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	                            int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

void ConeRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	                        int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

std::string exePath();

#endif