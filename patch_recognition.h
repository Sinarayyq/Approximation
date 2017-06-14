#pragma once

#ifndef PATCH_RECOGNITION_H
#define PATCH_RECOGNITION_H

#include "read_parameters.h"
#include "border_definition.h"
#include "segmentation.h"
#include "io.h"
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
#include <pcl/filters/project_inliers.h>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include "boost/graph/topological_sort.hpp"
#include <boost/property_map/property_map.hpp>



int PlaneRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int threshold_inliers, 
	                 pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients);

int FindPlaneBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
                   	pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients,
	                int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

//void PlaneRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, 
//	                  int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

int CylinderRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	                    int threshold_inliers, pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients);

int FindCylinderBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	                   pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients,
	                   int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

//void CylinderRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
//	                            int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);
int ConeRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	                int threshold_inliers, pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients);

int FindConeBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	               pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients,
	               int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);

struct Models_recognition_results
{
	std::string model_with_maximum_points; //0=plane, 1=cylinder, 2=cone
	pcl::PointIndices::Ptr inliers_plane;
	pcl::ModelCoefficients::Ptr coefficients_plane;
	pcl::PointIndices::Ptr inliers_cylinder;
	pcl::ModelCoefficients::Ptr coefficients_cylinder;
	pcl::PointIndices::Ptr inliers_cone;
	pcl::ModelCoefficients::Ptr coefficients_cone;

};

//void ConeRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
//	                        int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);
//


int SinglePatchPartition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, int threshold_inliers,
	                     int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds, Models_recognition_results *results_after_single_patch_recognition);

int TwoPatchesPatition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, int threshold_inliers,
	                   int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds, Models_recognition_results results_single_patch_recognition);





#endif