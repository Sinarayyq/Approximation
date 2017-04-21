//Lisa Chiang
//This functions are used to build borders of the found surface patches.

#pragma once

#ifndef BORDER_DEFINITION_H
#define BORDER_DEFINITION_H

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>  // std::abs
#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#include <boost/foreach.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>

#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\imgproc\types_c.h>
#include <opencv\cv.h>
#include <opencv2/core/mat.hpp>

#include <Eigen/Geometry> 
#include <boost/tuple/tuple.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "io.h"
#include "alpha_shape_polygons.h"
#include "geometry_tools.h"
#include "cloud_visualizer.h"
#include "read_parameters.h"



//#include "C:\Extract_indices\build\geometry_tools.h"
//#include "C:\Extract_indices\build\cloud_visualizer.h"
//#include "C:\Extract_indices\build\read_parameters.h"


#include <pcl/surface/convex_hull.h>
//#include <C:\PCl\pcl\surface\include\pcl\surface\impl\convex_hull.hpp>

typedef boost::tuple<double, double> point;
typedef boost::geometry::model::polygon<point> polygon;

struct result {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two;
	Eigen::MatrixXf indexes_one;
	Eigen::MatrixXf indexes_two;
	std::vector<float> line_param_one;//its a row vector with 6 coloumns, [0]= coord x head,[1]=coord y head,[2]=coord z head, [3]= coord x head,[4]=coord y head,[5]=coord z head
	std::vector<float> line_param_second;
	
};
 extern struct result data[10];

////////////////////////////////////////////////////////////////////////////////////////////////////
//PLANE 

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)

// Functions for the computation of the Minimum Area Rectangle (NOT CURRENTLY USED)
void getMinAreaRectOfPlanarPatch(
	pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud, std::vector<float> plane_param);

cv::RotatedRect getMinAreaRectForPlanarPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud);
//

Eigen::MatrixXf MainPlanarPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud, std::vector<float> plane_param, bool *good_patch_marker_plane, int *count_flattened_cloud);

// It transforms the set of points corresponding to a PLANAR patch, moving them
// such that z axis is aligned with the normal to the plane and xy-plane 
// is concident with the PLANE.
// INPUT: the point cloud corresponding to the patch, the PLANE parameters
// OUTPUT: a pointer to the new point cloud, containing the coordinates of the points in the new coord system
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPlanarPatchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud, std::vector<float> plane_param);

// It creates a matrix corresponding to the transformation that has to be applied
// to the points in the cloud, such that z axis is aligned with the normal to the PLANE and xy-plane 
// is concident with the plane.
// INPUT: the plane parameters
// OUTPUT: the 3x3 transformation matrix
Eigen::Matrix3f getTransformMatrixForAlignmentWithNormalToPlane(std::vector<float> plane_param);

// It compute a random point lying on a given plane
// INPUT: the plane parameters
// OUTPUT: a 3-dim vector containing the coordinates of the computed points
Eigen::Vector3f computeAPointOnPlane(std::vector<float> plane_param);

// It computes two points that lie on a given PLANE, provided its parameters.
// INPUT: the plane parameters
// OUTPUT: two vectors of length 3 representing the two points on the plane
std::pair<Eigen::Vector3f, Eigen::Vector3f> computeTwoPointsOnPlane(std::vector<float> plane_param);


std::vector<int> IdentifyPlaneChains(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,std::vector<int> &indexes,std::vector<int> &marker_chain);

////////////////////////////////////////////////////////////////////////////////////////////////////
//CYLINDER

// this the main function which coordinates all the function related to the cylinder
// it is revoked upon finding a cylindrical patch by the main function
// It returns a X*3 matrix, such that:
// column one represents the index of orignal 3D cloud that can form first vertix of a candidate line
// column two represents the index of orignal 3D cloud that can form second vertix of a candidate line
// Column three represnt the chain number, untill the chain number is not changed it implies that the vertixes can be merged and one candidate line can be formed
Eigen::MatrixXf  MainCylindricalPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr cylindrical_patch_cloud, std::vector<float> cone_param, bool *good_patch_marker_cylinder, int *count_flattened_cloud);

// This function takes input convex hull points as its input and return of vector of points which can form candidate lines when joined together 
std::vector<point> CylinderCandiateLines (std::vector<point> convex_hull_points,std::vector<float> cone_param);

// It flattens a cylindrical patch on the xz-plane
// INPUT: the point cloud corresponding to the cylindrical patch, the geometrical parameters of the cylinder
// OUTPUT: the point cloud corresponding to the flattened cylinder (y coordinates = 0)
pcl::PointCloud<pcl::PointXYZ>::Ptr flattenCylindricalPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cy_patch_cloud, std::vector<float> cyl_param);

// It transforms the cloud corresponding to a cylindrical patch in 'canonical' position (z-axis corresponding to the cylinder axis)
// INPUT: the point cloud corresponding to the cylindrical patch, the geometrical parameters of the cylinder
// OUTPUT: the point cloud corresponding to the cylinder in 'canonical' position
pcl::PointCloud<pcl::PointXYZ>::Ptr transformCylindricalPatchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cylindrical_patch_cloud, std::vector<float> cyl_param);

// It builds the transformation matrix 3x3 that will be used to bring the cylinder in canonical position
// INPUT: the geometrical parameters of the cylinder
// OUTPUT: the 3x3 transformation matrix
Eigen::Matrix3f getTransformMatrixForAlignmentWithCylinderAxis(std::vector<float> cyl_param);

// It returns the coefficients a,b,c,d corresponding to a plane defined by a point and a given outer normal
// INPUT: the point, the normal vector
// OUTPUT: a 4 dimensional vector containing the coefficients
std::vector<float> computePlanePassingThroughPointWithGivenNormal(Eigen::Vector3f point_through, Eigen::Vector3f normal);

//////////////////////////////////////////////////////
//CONE
// this the main function which conical all the function related to the cylinder
// it is revoked upon finding a conical patch by the main function
// It returns a X*3 matrix, such that:
// column one represents the index of orignal 3D cloud that can form first vertix of a candidate line
// column two represents the index of orignal 3D cloud that can form second vertix of a candidate line
// Column three represnt the chain number, untill the chain number is not changed it implies that the vertixes can be merged and one candidate line can be formed
Eigen::MatrixXf  MainConicalPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud, std::vector<float> plane_param, bool *good_patch_marker_cone, int *count_flattened_cloud);

// This function will try to merge the candidate lines to form a continious chains 
std::vector<int> IdentifyChains(pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud ,std::vector<int> &indexes,std::vector<int> &marker_chain);

// This function takes input covex hull points as its input and return of vector of points which can form candidate lines when joined together 
std::vector<point> ConeCandiateLines (std::vector<point> convex_hull_points);

std::vector<point> PreSelectionForBorder (pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud);

// This function returns a flatten conical point cloud. It revokes "TranformConicalPoint CLoud", "sortcloud" and "Cutting direction" functions
pcl::PointCloud<pcl::PointXYZ>::Ptr FlattenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud, std::vector<float> cone_param);

// This function takes sorted point cloud as a parameter and reurn then cutting angle in (-pi, pi)
//It looks for the largest gap among consecutive sorted thetas
float CuttingDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud);

// This funciton take canonical cloud(rad, theta) and cone parameters as input
// And returns a sorted point cloud sorted in incresing theta value
pcl::PointCloud<pcl::PointXYZ>::Ptr SortCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud, std::vector<float> cone_param);

// This is functions sorts an array in decresing order.
// It sort the entire row based on the value of a particular column using a prebuilt std::sort function
void sortrows(std::vector<std::vector<double>>& matrix, int col) ;

// This function takes input in form of canonical point cloud
// It return the collpased cloud, X cordinate represent the radius, Y cordinate represent the theta, and Z=0
// To visualize this collapsed cloud, it also computes Collapsed_cloud_2D, where X = rad*cos(theta) Y= rad*sin(theta) and Z=0
pcl::PointCloud<pcl::PointXYZ>::Ptr CollapsedConeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_clouds);

// This function takes input the point cloud, cone parameters and gets transformation matrix by calling "getTransformMatrixForAlignmentWithConeAxis"
// And transforms the point cloud to the canonical position and return the transformed cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr transformConicalPatchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud, std::vector<float> cone_param);

// Input to this function is the cone paprmaeters and it returnt he transformation matrix
//This function defines the Transformation matrix to bring cone into its canonical position
Eigen::Matrix3f getTransformMatrixForAlignmentWithConeAxis(std::vector<float> cone_param);

//////////////////////////////////////Functions for the computation of convex Hull////////////////////////////////////////////

polygon buildPolygonProjectingOnXZPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud);

std::vector<point> getConvexHullOfBoostPolygon(polygon poly);

double getConvexHullArea(std::vector<point> convex_hull_points);

std::vector<point> convexHullForPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud);

std::vector<point> GenerateConvexHull(std::vector<point> convex_hull_points1);

std::vector<point> SequenceConvexHull(std::vector<point> convex_hull_points);

///////////////////////////////////////////////////////////////Miscellaneous Functions/////////////////////////////////////////
//MERGING PATCHES

// This function takes as input a flattened cloud and a vector of the coordinates of the points of the convex hull.
// It matches the x and z cordinates of  candidate points to that of points in the point cloud, to find the index of the point
// in the cloud that corresponds to those coordinates.
// it then return std::vector indexes, in which two consecutive elements represent the indices of the estreme points of a candidate line
std::vector<int> IdentifyIndexOfCandiatePointInFlattenCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud, std::vector<point> Candidate_points);

// This function is revoked from main, and gets all the point cloud pathces and patch data as its parameters
void merge_patches(	Eigen::MatrixXf *patch_data ,pcl::PointCloud<pcl::PointXYZ>::Ptr* sourceClouds, int num_patch, double tolerance);

// this function will compute the distance between the candidate line of the first patch and different candidate points in the second patch
// finally if it finds that two patches are in close tolerance it will store the informatation in the struct 'result'
void ComputePatchDistance(int start,int end,	Eigen::MatrixXf patch_one,Eigen::MatrixXf patch_second,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,double tolerance,int &r_count );

#endif