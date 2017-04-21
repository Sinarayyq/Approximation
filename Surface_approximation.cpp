#include <iostream>
#include <time.h>
#include <deque>
#include <iterator>
#include <list>
#include <direct.h>
using namespace std;
#define NOMINMAX
//CGAL headers
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

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

//#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include "boost/graph/topological_sort.hpp"
#include <boost/property_map/property_map.hpp>
//#undef BOOST_NO_CXX11_SCOPED_ENUMS


#include "read_parameters.h"
#include "cloud_visualizer.h"
#include "patch_recognition.h"
//#include "check_patch_validity.h"
#include "geometry_tools.h"
#include "border_definition.h"
#include "segmentation.h"
#include "io.h"
#include "mesh.h"

#include "ui_Alpha_shapes_2.h"
#include <CGAL/Qt/resources.h>

using namespace std;


bool IsSupportPos(std::string* load_file)
{
	/*std::cerr << *((*load_file).end() - 1) << std::endl;
	std::cerr << *((*load_file).end() - 2) << std::endl;
	std::cerr << *((*load_file).end() - 3) << std::endl;
	std::cerr << *((*load_file).end() - 4) << std::endl;*/

	if (*((*load_file).end() - 1) == '"')
	{
		if ((*((*load_file).end() - 2) == 'l') || (*((*load_file).end() - 2) == 'L') &&
			(*((*load_file).end() - 3) == 't') || (*((*load_file).end() - 3) == 'T') &&
			(*((*load_file).end() - 4) == 'S') || (*((*load_file).end() - 4) == 's') &&
			(*((*load_file).end() - 5) == ('.')))
		{
			(*load_file).erase((*load_file).begin(), (*load_file).begin() + 1);
			(*load_file).erase((*load_file).begin() + (*load_file).size() - 1, (*load_file).begin() + (*load_file).size());
			return true;
		}

	}
	else
	{
		if ((*((*load_file).end() - 1) == 'l') || (*((*load_file).end() - 1) == 'L') &&
			(*((*load_file).end() - 2) == 't') || (*((*load_file).end() - 2) == 'T') &&
			(*((*load_file).end() - 3) == 'S') || (*((*load_file).end() - 2) == 's') &&
			(*((*load_file).end() - 4) == ('.')))
			return true;
	}

	return false;
}

int main(int argc, char** argv)
{
	using namespace std;
	using namespace pcl;
	using namespace boost;
	using namespace Eigen;

	//Create temp folder if not already existed.
	string project_path = exePath() + "\\..\\..";
	if ((CreateDirectory((project_path + "\\temp").c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()))
	{
		
	}
	else
	{
		mkdir((project_path + "\\temp").c_str());
	}
		

	//Read the parameters from the external text file
	if (!readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt"))
	{
		return(0);
	}

	string load_file;
	while (getline(cin, load_file))
	{
		if (!IsSupportPos(&load_file))
		{
			std::cout << "ERROR: File is not stl." << std::endl;
			continue;
		}


		/*int length_of_load_file = load_file.size();
		bool flag_space = false;
		for (int i = 0; i < length_of_load_file - 1; i++)
		{
		if (load_file[i] == ' ')
		{
		flag_space = true;
		}
		}
		if (flag_space == true)
		{
		std::cout << "ERROR: File name contains space." << std::endl;
		continue;
		}*/




		/*string::size_type pos = 0;
		while ((pos = load_file.find_first_of('\\', pos)) != string::npos)
		{
		load_file.insert(pos, "\\");
		pos = pos + 2;
		}*/

		//cout << "1load_file:" << load_file << endl;

		//load_file = load_file + ".stl";
		//string load_path = "C:\\Extract_indices\\STL\\" + load_file;


		// Define the point clouds for points and for normals that will be used in the process
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		ReadSTLFile(load_file.c_str(), &cloud_filtered, &cloud_normals);
		//pcl::io::loadPCDFile(PATH_PCD_DOC, *cloud_filtered);
		//// Fill the initial point clouds reading from the PCD file
		//if (!fillClouds(PATH_PCD_DOC, cloud_filtered, cloud_normals))
		//{
		//	return (-1);
		//}

		// Define the object for writing in output PCD files the segmented sub clouds
		//pcl::PCDWriter writer;

		// Visualize the initial point cloud
		//visualizePointCloud(cloud_filtered,"INITIAL POINT CLOUD");	

		// Set the minimum number of inliers required for each surface fitting 
		const int nr_points = (int)cloud_filtered->points.size();
		const double threshold_inliers = MIN_INLIERS * nr_points;
		cerr << endl << "INITIAL NUMBER OF POINTS IN THE CLOUD: " << nr_points << endl;
		//std::cerr << std::endl << "MIN_INLIERS = " << MIN_INLIERS << std::endl;
		//cerr << endl << "threshold_inliers = " << threshold_inliers << endl;

		

		//int i = 0;
		//int num_inliers = 0;
		int patch_count = 0;
		const int max_patches = nr_points / threshold_inliers;  //max number of patches that we could find
		Eigen::MatrixXf *patch_data = new Eigen::MatrixXf[max_patches]; //vector of matrices
		pcl::PointCloud<pcl::PointXYZ>::Ptr *sourceClouds = new pcl::PointCloud<pcl::PointXYZ>::Ptr[max_patches]; //vector of pointers to cloud
																												  //vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > sourceClouds;
		//int count_flattened_cloud = 0;

		while (cloud_filtered->points.size() > threshold_inliers)
		{
			
			std::cerr << std::endl << std::endl << "===========================================================================" << std::endl;
			std::cerr << "If you want to try  plane   recognition, input 0;" << std::endl;
			std::cerr << "If you want to try cylinder recognition, input 1;" << std::endl;
			std::cerr << "If you want to try   cone   recognition, input 2;" << std::endl;
			std::cerr << "If you want to quit, input anything else;" << std::endl;
			visualizePointCloud(cloud_filtered, "POINT CLOUD");
			char recognition_type;
			std::cin >> recognition_type;
			cin.clear();
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			if (recognition_type == '0')
			{
				PlaneRecognition(&cloud_filtered, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds);
			}
			else
			{
				if (recognition_type == '1')
				{
					CylinderRecognition(&cloud_filtered, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds);
				}
				else
				{
					if (recognition_type == '2')
					{
						ConeRecognition(&cloud_filtered, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds);
					}
					else
					{
						return 0;
					}
				}
			}
		}
		


		//// ----> Match of the found patches, according to the previously identified border lines.
		//// Print of the info found so far
		//std::cerr << std::endl << "========================================================================" << std::endl;
		//std::cerr << "============================ PATCH MATCHING ============================" << std::endl << std::endl;
		//std::cerr << "NUMBER OF CYLINDRICAL and CONICAL PATCHES:" << patch_count<< endl <<endl;
		//std::cerr << "Print of the data related to the found chains of candidate lines" << endl;
		//std::cerr << "(1st column: index of the head of the line" << endl;
		//std::cerr << " 2nd column: index of the tail of the line" << endl;
		//std::cerr << " 3rd column: index of the chain of lines it belongs to)" << endl << endl;
		//for (int i=0; i < patch_count;i++)
		//{
		//	cerr << endl << "- " << i+1 << "th patch." <<endl<<endl;
		//	//cerr<<endl<< "NUMBER OF CANDIDATE LINES = " << patch_data[i].rows() <<endl;
		//	for (int j =0;j<patch_data[i].rows();j++)
		//	{
		//		cerr<< patch_data[i](j )<<" "<<patch_data[i](j+patch_data[i].rows())<<" "<< patch_data[i](j+2*(patch_data[i].rows()))<<endl;
		//	}
		//}

		//// Put in corrispondence patches that has close enough border lines
		//merge_patches(patch_data, sourceClouds,patch_count,TOLERANCE_FOR_ADJACENT_CHAINS);

		//std::cout << "matrix 1: "  << std::endl;
		//std::cout << data[0].indexes_one << std::endl;

		//std::cout << "matrix 2: "  << std::endl;
		//std::cout << data[0].indexes_two << std::endl;
		//

		//// ---> Program terminates.
		//std::cerr << std::endl << "============================================" << std::endl;
		//std::cerr << "============================================" << std::endl << std::endl;
		//std::cerr << cloud_filtered->size() << " points have not been fitted in any surface." << std::endl;
		std::cerr << "====================END=====================" << std::endl << std::endl;
		std::system("pause");
	}
	return 0;
}