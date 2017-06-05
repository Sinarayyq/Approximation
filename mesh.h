#pragma once

#ifndef MESH_H
#define MESH_H

#include <fstream>
#include <string>
#include <vector>
#include <windows.h>
#include <math.h>
#include <iostream> 

#include "read_parameters.h"
#include "cloud_visualizer.h"
#include "utils_sampling.hpp"
#include "vcg_mesh.hpp"
#include "io.h"

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>



bool ReadSTLFile(const char *cfilename, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals);
//extern pcl::PointCloud<pcl::Normal>::Ptr all_cloud_normals (new pcl::PointCloud<pcl::Normal>);

#endif