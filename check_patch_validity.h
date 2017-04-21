//Lisa Chiang
//This functions are used to check if the patch that has been found by the method is valid.
//A patch is not valid if:
//- it is a narrow strip 
//- it is composed by 2 or more groups of points far from each other

#pragma once //If it is not added there is an error. It could disappear whenever I use this header (?)

#ifndef CHECK_PATCH_VALIDITY_H
#define CHECK_PATCH_VALIDITY_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <pcl/common/geometry.h>
//Ransac
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/model_outlier_removal.h>

#include <deque>
#include <iterator>

#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>

#include <list>
#include <limits>
#include <vector>

#include "../build/read_parameters.h"

using namespace boost;

typedef adjacency_list < vecS, vecS, undirectedS, no_property, property < edge_weight_t, double >> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;

typedef property < edge_weight_t, double >Weight;
typedef std::pair < int, int > E;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef adjacency_list < vecS, vecS, undirectedS, no_property, no_property> GraphNoWeight;

template <typename EdgeWeightMap>
		struct positive_edge_weight {
		  positive_edge_weight() { }
		  positive_edge_weight(EdgeWeightMap weight) : m_weight(weight) { }
		  template <typename Edge>
		  bool operator()(const Edge& e) const {
			return 0 < get(m_weight, e);
		  }
		  EdgeWeightMap m_weight;
		};

// It returns the Minimum Spanning Tree of a given graph using Prim's algorithm 
// INPUT: the graph
// OUTPUT: a vector p such that p(i)=index of the successor of i-(th) node in the tree
// (if p(i)=i then i is the index corresponding to an "extreme" node)
std::vector<uint32_t, std::allocator<char32_t>> mstPrim (Graph g);

// It reads and sets the method parameters from a txt file 
// INPUT: the MST graph, an int corresponding to the index of the starting point node, 
//        the vector of parents (parents(i)=predecessor of i-(th) node), 
//        the vector of distances from the source (distances(i)=distance of i-(th) node from the source) 
void myBFS (Graph mstGraph, int source, std::vector<int>& parents, std::vector<int>& distances);

int furthestNode (int source, Graph mstGraph, std::list<int>& path);

void findLongestPathInTree (Graph mstGraph, std::list<int>& longest_path);

bool checkMSTToFindStrips (Graph mstGraph);


std::pair<bool,std::list<std::pair<vertex_descriptor,vertex_descriptor>>> longLinks(Graph mstGraph, 
	std::vector<uint32_t, std::allocator<char32_t>> p);

Graph buildGraphFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const int num_nodes);


std::pair<Graph, std::vector<uint32_t, std::allocator<char32_t>>> getMSTGraph(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const int num_nodes);


std::tuple<bool, Graph, std::vector<uint32_t, std::allocator<char32_t>>> isStrip(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);


bool isToBeSeparated(std::string surface_type,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Graph mstGraph, std::vector<uint32_t, std::allocator<char32_t>> p,
	std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>& group_cloud);

void separateAndOptimizePlanarPatches(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points);

void separateAndOptimizeCylindricalPatches(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points);

void separateAndOptimizeConicalPatches(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points);

void separateAndOptimizePatches(std::string surface_type, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_the_patch, 
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_on_the_patch,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points, 
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_of_the_other_points);

// It checks if the found surface patch (grouping a certain subcloud of points)
// is a strip or it groups points that are not close enough.
// If there are groups of points to be separated, this function will separate and optimize them.
// INPUT: the cloud of points grouped by the found patch, the cloud of all the points still to fit (also the points on the patch)
// OUTPUT: TRUE if the patch is valid (it is not a strip); FALSE if it is not valid
bool checkPatchValidity(std::string surface_type, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_the_patch, 
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_on_the_patch,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points, 
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_of_the_other_points);

#endif