#include "segmentation.h"

void setSegmentationParametersForPlane(pcl::SACSegmentation<pcl::PointXYZ>& seg_plane)
{
	// Create the segmentation object
	//pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg_plane.setOptimizeCoefficients (true);
	// Mandatory
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (PLANE_METHOD_TYPE);
	seg_plane.setMaxIterations (PLANE_MAX_NUM_ITER);
	seg_plane.setDistanceThreshold (PLANE_TOL);
}

void setSegmentationParametersForCylinder(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>& seg_cyl)
{
	// Set all the parameters for cylinder segmentation object
	seg_cyl.setOptimizeCoefficients (true);
	seg_cyl.setModelType (pcl::SACMODEL_CYLINDER);
	seg_cyl.setMethodType (CYL_METHOD_TYPE);
	seg_cyl.setNormalDistanceWeight (CYL_WEIGHT_NORMAL_DISTANCE);
	seg_cyl.setMaxIterations (CYL_MAX_NUM_ITER);
	seg_cyl.setDistanceThreshold (CYL_TOL);
	seg_cyl.setRadiusLimits (CYL_MIN_RADIUS_LIMIT, CYL_MAX_RADIUS_LIMIT);
}

void setSegmentationParametersForCone(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>& seg_cone)
{
	// Set all the parameters for cone segmentation object
	seg_cone.setOptimizeCoefficients (true);
	seg_cone.setModelType (pcl::SACMODEL_CONE);
	seg_cone.setMethodType (CONE_METHOD_TYPE);
	seg_cone.setMinMaxOpeningAngle(CONE_MIN_OPENING_ANGLE / 180.0  * M_PI, CONE_MAX_OPENING_ANGLE / 180.0 * M_PI); //it is in radiants; min=5degree, max=80degree
	seg_cone.setNormalDistanceWeight (CONE_WEIGHT_NORMAL_DISTANCE);
	seg_cone.setMaxIterations (CONE_MAX_NUM_ITER);
	seg_cone.setDistanceThreshold (CONE_TOL);
	seg_cone.setRadiusLimits (CONE_MIN_RADIUS_LIMIT, CONE_MAX_RADIUS_LIMIT);
}