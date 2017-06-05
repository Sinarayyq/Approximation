#pragma once

#ifndef IO_H
#define IO_H

#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/extract_indices.h>
#include <string>
#include <iostream>
#include <fstream>

#include "border_definition.h"
#include "alpha_shape_polygons.h"


extern std::string PATH_HEAD;

std::string exe_Path();

bool fillClouds(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals1);

void outputCloudOnExcel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud);

void outputCloudOnPTS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud);

void outputCloudOnExcel(pcl::ModelCoefficients::Ptr coefficients, std::string name_cloud, std::string type);

std::string exportCloudAsPTS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud);

std::string outputCloudOnTXT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud, int count);



//void outputCloudOnTXT_PtNumber(Polygon_2 polygon);

//void outputCloudOnTXT_PtNumber(std::vector<Alpha_shape_2::Point> alpha_shape_points);


#endif