#pragma once

#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "read_parameters.h"

void setSegmentationParametersForPlane(pcl::SACSegmentation<pcl::PointXYZ>& seg);

void setSegmentationParametersForCylinder(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>& seg_cyl);

void setSegmentationParametersForCone(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>& seg_cone);

//void setSegmentationConstraintForCylinder()
#endif