#include "border_definition.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/tuple/tuple.hpp>
using namespace std;
#define FEQ(x,y) (abs(x-y)<.00000001 ? 1:0)
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/pcl_visualizer.h>

float max_gap, beta_fan, beta_max;


////////////////////////////////////////////////////////////////////////////////////////////////////
//PLANE 

#pragma region Minimum bounding rectangle function (NOT USED)
////////////Functions for the computation of the Minimum Area Rectangle:

/*void getMinAreaRectOfPlanarPatch(
pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud, std::vector<float> plane_param)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_patch_cloud (new pcl::PointCloud<pcl::PointXYZ>);
transformed_patch_cloud = transformPlanarPatchPoints(planar_patch_cloud, plane_param);
//std::vector<point> ddd = convexHullForPlanarPoints(transformed_patch_cloud);
cv::RotatedRect output_rect = getMinAreaRectForPlanarPoints(transformed_patch_cloud);
cv::Point2f pts[4];
output_rect.points(pts); //this fills the argument array with the coordinates of the 4 points defining the rectagle.

//Print computed Min Area Rectangle information:
for (std::size_t i = 0; i < 4 ; ++i)
std::cerr << i << " vertex of the rectangle: (" << pts[i].x << ", " << pts[i].y << ")" << std::endl;
//see http://docs.opencv.org/master/db/dd6/classcv_1_1RotatedRect.html#ae1be388780b8d5faf450be18cbbf30f1
//for details about cv::RotatedRect

//std::cerr << "angle rectangular: " << calculatedRect.angle << std::endl;
}*/

/*cv::RotatedRect getMinAreaRectForPlanarPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud)
{
std::vector<point> const& convex_hull_points = convexHullForPoints(planar_patch_cloud);

//We compute the minimum area rectangle by using the minAreaRect function from the library openCV.
//We first convert the points in the convex hull in the format accepted by the openCV function.
std::vector<cv::Point2f> points_opencv_format;
for( int i = 0; i < convex_hull_points.size(); i++ )
points_opencv_format.push_back(cv::Point2f((float)(convex_hull_points[i].get_head()),
(float)(convex_hull_points[i].get_tail().get_head())));

cv::RotatedRect calculatedRect = cv::minAreaRect(points_opencv_format);

return calculatedRect;
}*/
#pragma endregion


//////////////////////////////////////////////////////////////////////////////////////////////////
///Plane
Eigen::MatrixXf MainPlanarPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud, std::vector<float> plane_param, bool *good_patch_marker_plane, int *count_flattened_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pla_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_pla_patch_cloud = transformPlanarPatchPoints(planar_patch_cloud, plane_param);
	//visualizePointCloud(transformed_pla_patch_cloud, "flattened_plane");
	double alpha_shape_area = getAlphaShape(outputCloudOnTXT(transformed_pla_patch_cloud, "flattened_plane", (*count_flattened_cloud)++));
	std::cout << std::endl << "alpha_shape_area = " << alpha_shape_area << std::endl;



	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\build\\transformed_pla_patch_cloud.pcd", *transformed_pla_patch_cloud);
	//visualizePointCloud(transformed_pla_patch_cloud, "transformed_pla_patch_cloud");
	//outputCloudOnExcel(transformed_pla_patch_cloud,"transformed_pla_patch_cloud");

	//We compute the convex hull of the flatten cloud. it returns the a vector of 2dim coordinates corresponding to the vertices of the CH
	std::vector<point> convex_hull_points = convexHullForPoints(transformed_pla_patch_cloud);
	double convex_hull_area = getConvexHullArea(convex_hull_points);
	std::cout << "convex_hull_area = " << convex_hull_area << std::endl;
	std::cout << "alpha_shape_area/convex_hull_area = " << alpha_shape_area / convex_hull_area << std::endl;
	if (alpha_shape_area / convex_hull_area < PLANE_AREA_PERCENTAGE)
	{
		*good_patch_marker_plane = 0;
		Eigen::MatrixXf plane_data(0, 0);
		return plane_data;
	}
	//std::cerr << "number of CH points = " << convex_hull_points.size() << endl;
	//We recover the indices in the flattened cloud corresponding to the vertices of the convex hull
	std::vector<int> hull_indexes = IdentifyIndexOfCandiatePointInFlattenCloud(transformed_pla_patch_cloud, convex_hull_points);

	//We identify the candidate lines corresponding to sides of the CH such that they are aligned with some
	//cylinder generatrix lines
	std::vector<int> marker_chain;  //this is the vector the assigns to every candidate line an index corresponding to which chain they belong to
	//We recover the indices in the flattened cloud corresponding to the vertices of the candidate lines


	//We join the candidate lines in chains of candidate lines, whenever they are adjacent and with similar orientations
	std::vector<int> plane_chain = IdentifyPlaneChains(transformed_pla_patch_cloud, hull_indexes, marker_chain);
	const int num_plane_chains = plane_chain.size();
	const int num_marker_chains = marker_chain.size();

	cerr << endl;
	cerr << "Indices of vertices extremes of cylinder chains (read 2 by 2):" << endl;
	for (int i = 0; i < num_plane_chains; i = i + 2)
	{
		cerr << plane_chain[i] << "  " << plane_chain[i + 1] << endl;
	}

	visualizeShapes(transformed_pla_patch_cloud, "Plane chains", hull_indexes, plane_chain);

	//We build a matrix of dimensions (num of candidate lines assigned to a chain) x 3
	//for every line we store: index head of the candidate line; index tail of the candidate line; index corresponding to the assigned chain
	Eigen::MatrixXf plane_data(num_marker_chains, 3);
	for (int iter = 0; iter< num_marker_chains; iter++)
	{
		plane_data(iter, 0) = plane_chain[2 * iter];
		plane_data(iter, 1) = plane_chain[2 * iter + 1];
		plane_data(iter, 2) = marker_chain[iter];
	}

	return plane_data;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPlanarPatchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_patch_cloud, std::vector<float> plane_param)
{
	Eigen::Matrix3f transform_matrix = getTransformMatrixForAlignmentWithNormalToPlane(plane_param);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ptr_output_cloud = transformCloudByMatrix(planar_patch_cloud, transform_matrix);
	//std::cerr << "List of the transformed points: " << std::endl;
	const int num_nodes = ptr_output_cloud->points.size();

	/*for (std::size_t i = 0; i < num_nodes; ++i)
	{
	std::cerr << "(" << ptr_output_cloud->at(i).x << "," << ptr_output_cloud->at(i).y << "," << ptr_output_cloud->at(i).z << ")" << std::endl;
	}*/
	return ptr_output_cloud;
}

Eigen::Matrix3f getTransformMatrixForAlignmentWithNormalToPlane(std::vector<float> plane_param)
{
	//std::cerr << "number of parameters: " << plane_param.size() << std::endl;

	//NORMALIZATION OF THE NORMAL VECTOR TO THE PLANE??

	Eigen::Vector3f x_axis;
	Eigen::Vector3f y_axis;
	Eigen::Vector3f z_axis;

	//z axis definition	
	for (std::size_t i = 0; i < 3; ++i)
	{
		y_axis[i] = plane_param[i];
	}
	double norm_y = y_axis.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		y_axis[i] = y_axis[i] / norm_y;
	}
	//std::cerr << "z_axis = (" << z_axis[0] << "," << z_axis[1] << "," << z_axis[2] << ")" << std::endl;

	//x axis definition: 
	//we find 2 points A,B on the plane, then we consider as x axis the unit vector A-B 
	//std::vector<float> A(3), B(3);
	std::pair<Eigen::Vector3f, Eigen::Vector3f> twoPointsOnPlane = computeTwoPointsOnPlane(plane_param);
	Eigen::Vector3f dif = twoPointsOnPlane.first - twoPointsOnPlane.second;
	//std::cerr << "Difference:  (" << dif[0] << "," << dif[1] << "," << dif[2] << ")" << std::endl;
	double norm = dif.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		x_axis[i] = dif[i] / norm;
	}
	//std::cerr << "x_axis = (" << x_axis[0] << "," << x_axis[1] << "," << x_axis[2] << ")" << std::endl;

	//y axis definition
	z_axis = x_axis.cross(y_axis);
	//std::cerr << "y_axis = (" << y_axis[0] << "," << y_axis[1] << "," << y_axis[2] << ")" << std::endl;

	//Get the transformation matrix referred to the new coordinate system
	Eigen::Matrix3f trasf_matrix = buildTransformMatrixFromAxis(x_axis, y_axis, z_axis);
	Eigen::Matrix3f trasf_matrix_inv = trasf_matrix.inverse();

	return trasf_matrix_inv;
	//return trasf_matrix;
}

Eigen::Vector3f computeAPointOnPlane(std::vector<float> plane_param)
{
	double threshold = pow(10, -5.0);

	float a = plane_param[0];
	float b = plane_param[1];
	float c = plane_param[2];
	float d = plane_param[3];

	float xA, yA, zA;

	int upperbound = 100;

	//double r = ((double) rand() / (RAND_MAX)); //random double in [0,1)
	//double random = (upperbound - lowerbound) * r + lowerbound; //random double in [lowerbound, upperbound)
	//int random_coord = rand() % (2*upperbound) - upperbound; //it produces a random integer between -100 and 100

	if (std::abs(c)<threshold)  //if c==0
	{
		if (std::abs(b)<threshold)  //if c==0 and b==0 
		{
			//a!=0
			xA = -d / a;
			yA = rand() % (2 * upperbound) - upperbound;
			zA = rand() % (2 * upperbound) - upperbound;
		}
		else   //if c==0 and b!=0
		{
			xA = rand() % (2 * upperbound) - upperbound;
			yA = -a / b*xA - d / b;
			zA = rand() % (2 * upperbound) - upperbound;
		}
	}
	else   //if c!=0
	{
		xA = rand() % (2 * upperbound) - upperbound;
		yA = rand() % (2 * upperbound) - upperbound;
		zA = -a / c*xA - b / c*yA - d / c;
	}

	Eigen::Vector3f A(xA, yA, zA);

	return A;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> computeTwoPointsOnPlane(std::vector<float> plane_param)
{
	double threshold = pow(10, -5.0);

	float a = plane_param[0];
	float b = plane_param[1];
	float c = plane_param[2];
	float d = plane_param[3];

	float xA, yA, zA, xB, yB, zB;

	if (std::abs(c)<threshold)  //if c==0
	{
		if (std::abs(b)<threshold)  //if c==0 and b==0 
		{
			//a!=0
			xA = -d / a;
			xB = -d / a;
			yA = 0; zA = 0;    //chosen randomly
			yB = 100; zB = 0;  //chosen randomly
		}
		else   //if c==0 and b!=0
		{
			xA = 0; xB = 0;    //chosen randomly
			yA = -a / b*xA - d / b;
			yB = -a / b*xB - d / b;
			zA = 100; zB = 0;  //chosen randomly
		}
	}
	else   //if c!=0
	{
		xA = 0; yA = 0;
		xB = 100; yB = 0;
		zA = -a / c*xA - b / c*yA - d / c;
		zB = -a / c*xB - b / c*yB - d / c;
	}

	/*std::vector<float> A, B;
	A[0]=xA; A[1]=yA; A[2]=zA;
	B[0]=xB; B[1]=yB; B[2]=zB;*/

	Eigen::Vector3f A(xA, yA, zA);
	Eigen::Vector3f B(xB, yB, zB);

	return std::make_pair(A, B);
}

std::vector<int> IdentifyPlaneChains(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, std::vector<int> &hull_indexes, std::vector<int> &marker_chain)
{
	const int num_hull_vertices = hull_indexes.size();
	/*Eigen::Vector2f point;
	for(std::size_t i = 0; i < num_hull_vertices-1; i++)
	{
	point[0]=plane_cloud->at(hull_indexes[i]).x;
	point[1]=plane_cloud->at(hull_indexes[i]).z;
	cout << "point = " << point[0] << " , " << point[1] << endl;
	}*/

	//form chains by connecting adjacent lines with the same slope
	double norm_one, norm_second;
	bool flag = true;
	int i = 0, j = 0, counter, marker = 1;
	std::vector<int> plane_chain, plane_chain_temp;
	Eigen::Vector2f A, B, C, D, diff_one, diff_second;
	while (i < hull_indexes.size() - 1)
	{
		A[0] = plane_cloud->at(hull_indexes[i]).x;
		A[1] = plane_cloud->at(hull_indexes[i]).z;
		B[0] = plane_cloud->at(hull_indexes[i + 1]).x;
		B[1] = plane_cloud->at(hull_indexes[i + 1]).z;
		norm_one = (B - A).norm();
		if (norm_one == 0)
		{
			i++;
			continue;
		}
		for (std::size_t i = 0; i < 2; i++)
			diff_one[i] = (B[i] - A[i]) / norm_one;

		j = i;
		counter = 0;
		// It checks further with next lines, if the dot product is close it assumes it as a one chain
		while (j<hull_indexes.size() - 1)
		{
			C[0] = plane_cloud->at(hull_indexes[j]).x;
			C[1] = plane_cloud->at(hull_indexes[j]).z;
			D[0] = plane_cloud->at(hull_indexes[j + 1]).x;
			D[1] = plane_cloud->at(hull_indexes[j + 1]).z;
			norm_second = (D - C).norm();
			if (norm_second == 0)
			{
				j++;
				counter++;
				continue;
			}
			for (std::size_t i = 0; i < 2; i++)
			{
				diff_second[i] = (D[i] - C[i]) / norm_second;
			}
			if (abs(diff_one.dot(diff_second))>0.99)
			{
				counter += 1;
				if (flag == true)
				{
					marker_chain.push_back(marker);
					plane_chain_temp.push_back(hull_indexes[i]);
					plane_chain_temp.push_back(hull_indexes[j + 1]);
					flag = false;
				}
				else
				{
					plane_chain_temp.pop_back();
					plane_chain_temp.push_back(hull_indexes[j + 1]);
				}
				j = j + 1;
			}
			else
			{
				marker++;
				flag = true;
				break;
			}
		}
		i = i + counter;
	}

	//avoid the situation that the last line( point(n-1)->point(0) ) have the same slope with the first line, the length of which is not 0
	A[0] = plane_cloud->at(hull_indexes[i]).x;
	A[1] = plane_cloud->at(hull_indexes[i]).z;
	B[0] = plane_cloud->at(hull_indexes[0]).x;
	B[1] = plane_cloud->at(hull_indexes[0]).z;
	norm_one = (B - A).norm();
	if (norm_one == 0)
	{
		return plane_chain_temp;
	}

	for (std::size_t j = 0; j < num_hull_vertices - 1; j++)
	{
		C[0] = plane_cloud->at(hull_indexes[j]).x;
		C[1] = plane_cloud->at(hull_indexes[j]).z;
		D[0] = plane_cloud->at(hull_indexes[j + 1]).x;
		D[1] = plane_cloud->at(hull_indexes[j + 1]).z;
		norm_second = (D - C).norm();
		if (norm_second != 0)
		{
			break;
		}
	}

	for (std::size_t j = 0; j < 2; j++)
	{
		diff_one[j] = (B[j] - A[j]) / norm_one;
		diff_second[j] = (D[j] - C[j]) / norm_second;
	}

	if (abs(diff_one.dot(diff_second))>0.99)
	{
		plane_chain.push_back(hull_indexes[i]);
		for (std::size_t j = 1; j < plane_chain_temp.size(); j++)
		{
			plane_chain.push_back(plane_chain_temp[j]);
		}
	}
	else
	{
		marker++;
		marker_chain.push_back(marker);
		for (std::size_t j = 0; j < plane_chain_temp.size(); j++)
		{
			plane_chain.push_back(plane_chain_temp[j]);
		}
		plane_chain_temp.push_back(hull_indexes[i]);
		plane_chain_temp.push_back(hull_indexes[0]);
	}


	if (plane_chain.size() == 0)
	{
		plane_chain = hull_indexes;
	}
	return plane_chain;
}







////////////////////////////////////////////////////////////////////////////////////////////////////
///CYLINDER
Eigen::MatrixXf MainCylindricalPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr cylindrical_patch_cloud, std::vector<float> cyl_param, bool *good_patch_marker_cylinder, int *count_flattened_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_cyl_patch_cloud = transformCylindricalPatchPoints(cylindrical_patch_cloud, cyl_param);

	//visualizePointCloud(transformed_cyl_patch_cloud, "transformed_cyl_patch_cloud");
	//We flatten the cloud corresponding to the cylindrical patch 
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	flattened_cloud = flattenCylindricalPatch(transformed_cyl_patch_cloud, cyl_param);
	double alpha_shape_area = getAlphaShape(outputCloudOnTXT(flattened_cloud, "flattened_cylinder", (*count_flattened_cloud)++));
	std::cout << std::endl << "alpha_shape_area = " << alpha_shape_area << std::endl;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud= transformConicalPatchPoints(conical_patch_cloud, cone_param);	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud= FlattenCloud(canonical_output_cloud, cone_param);

	//We compute the convex hull of the flatten cloud. it returns the a vector of 2dim coordinates corresponding to the vertices of the CH
	std::vector<point> convex_hull_points = convexHullForPoints(flattened_cloud);
	//std::cerr << "number of CH points = " << convex_hull_points.size() << endl;
	double convex_hull_area = getConvexHullArea(convex_hull_points);
	std::cout << "convex_hull_area = " << convex_hull_area << std::endl;
	std::cout << "alpha_shape_area/convex_hull_area = " << alpha_shape_area / convex_hull_area << std::endl;
	if (alpha_shape_area / convex_hull_area < CYL_AREA_PERCENTAGE)
	{
		*good_patch_marker_cylinder = 0;
		Eigen::MatrixXf cyl_data(0, 0);
		return cyl_data;
	}

	//We recover the indices in the flattened cloud corresponding to the vertices of the convex hull
	std::vector<int> hull_indexes = IdentifyIndexOfCandiatePointInFlattenCloud(flattened_cloud, convex_hull_points);
	//cout << "border index is " << hull_indexes.size() <<endl;
	//cout << "border convex is " << convex_hull_points.size() <<endl;

	//We identify the candidate lines corresponding to sides of the CH such that they are aligned with some
	//cylinder generatrix lines
	std::vector<point> Candidate_points = CylinderCandiateLines(convex_hull_points, cyl_param);
	std::vector<int> marker_chain;  //this is the vector the assigns to every candidate line an index corresponding to which chain they belong to
	const int num_candidates_vertices = Candidate_points.size();
	//cerr<<"Number of candidate lines: "<< num_candidates_vertices/2<< endl;	
	//We recover the indices in the flattened cloud corresponding to the vertices of the candidate lines
	std::vector<int> indexes = IdentifyIndexOfCandiatePointInFlattenCloud(flattened_cloud, Candidate_points);

	////We print the just recovered indices corresponding to the points extremes of the candidate lines (they need to be read 2 by 2)
	//cerr<<"Coordinates of vertices to form candidate line: "<<endl;
	//for (int i = 0; i < num_candidates_vertices; i++)
	//	std::cerr<< indexes[i]<<endl;

	//We join the candidate lines in chains of candidate lines, whenever they are adjacent and with similar orientations
	std::vector<int> cyl_chain = IdentifyChains(transformed_cyl_patch_cloud, indexes, marker_chain);
	const int num_cyl_chains = cyl_chain.size();
	const int num_marker_chains = marker_chain.size();

	cerr << endl;
	cerr << "Indices of vertices extremes of cylinder chains (read 2 by 2):" << endl;
	for (int i = 0; i < num_cyl_chains; i = i + 2)
		cerr << cyl_chain[i] << "  " << cyl_chain[i + 1] << endl;

	//visualizeShapes(flattened_cloud, "Candidate Lines",hull_indexes, indexes);
	visualizeShapes(flattened_cloud, "Cylinder chains", hull_indexes, cyl_chain);

	//We build a matrix of dimensions (num of candidate lines assigned to a chain) x 3
	//for every line we store: index head of the candidate line; index tail of the candidate line; index corresponding to the assigned chain
	Eigen::MatrixXf cyl_data(num_marker_chains, 3);
	for (int iter = 0; iter< num_marker_chains; iter++)
	{
		cyl_data(iter, 0) = indexes[2 * iter];
		cyl_data(iter, 1) = indexes[2 * iter + 1];
		cyl_data(iter, 2) = marker_chain[iter];
	}

	return cyl_data;
}

std::vector<point> CylinderCandiateLines(std::vector<point> convex_hull_points, std::vector<float> cyl_param)
{
	const int num_hull_vertices = convex_hull_points.size();
	//std::vector<int> Candidate_indexes;
	std::vector<point> Candidate_points;
	Eigen::Vector2f cyl_axis(0, 1), vector_point;
	double norm_point;

	//std::cerr<< num_hull_vertices<< "------"<<endl ;

	Eigen::Vector2f point_i, point_iplus;
	for (std::size_t i = 0; i < num_hull_vertices - 1; i++)
	{
		point_i[0] = boost::get<0>(convex_hull_points[i]);
		point_i[1] = boost::get<1>(convex_hull_points[i]);
		point_iplus[0] = boost::get<0>(convex_hull_points[i + 1]);
		point_iplus[1] = boost::get<1>(convex_hull_points[i + 1]);
		Eigen::Vector2f dif_point = point_iplus - point_i;
		norm_point = dif_point.norm();
		if (norm_point == 0)
		{
			continue;
		}
		for (std::size_t i = 0; i < 2; ++i)
		{
			vector_point[i] = dif_point[i] / norm_point;
		}


		//cerr<<vector_point.dot(cyl_axis)<<endl;

		if (abs(vector_point.dot(cyl_axis))>0.99)
		{
			//Candidate_indexes.push_back(i);
			//Candidate_indexes.push_back(i+1);
			Candidate_points.push_back(boost::make_tuple(point_i[0], point_i[1]));
			Candidate_points.push_back(boost::make_tuple(point_iplus[0], point_iplus[1]));
		}

	}
	return Candidate_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr flattenCylindricalPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud, std::vector<float> cyl_param)
{
	//>>>> Flatten the cylinder on the xz-plane
	const int num_points = transformed_cyl_patch_cloud->points.size();
	double cyl_radius = cyl_param[6];
	//std::cerr << "cyl_radius = " << cyl_radius << std::endl;
	Eigen::Vector3f point_to_flatten, flattened_point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<double> x_vector(num_points);

	for (std::size_t i = 0; i < num_points; ++i)
	{
		flattened_point[0] = cyl_radius*atan2(transformed_cyl_patch_cloud->at(i).y, transformed_cyl_patch_cloud->at(i).x);
		flattened_point[1] = 0;
		flattened_point[2] = transformed_cyl_patch_cloud->at(i).z;
		flattened_cloud->push_back(pcl::PointXYZ(flattened_point[0], flattened_point[1], flattened_point[2]));
		x_vector[i] = flattened_point[0]; //save the x coordinate of the current point
	}
	//>>>>We scan x coordinates of the flattened cylinder to see if we cut the patch right in the middle of it.
	//>>>>In that case we need to move the portions of the flattened patch such that it is again a single patch
	std::sort(x_vector.begin(), x_vector.end());
	bool found_gap = false;
	double cutting_x;
	double max_gap_cyl = 2 * M_PI * cyl_radius - abs(x_vector[0] - x_vector[num_points - 1]);

	for (std::size_t i = 0; i < (num_points - 1); ++i)
	{
		if (abs(x_vector[i] - x_vector[i + 1]) > max_gap_cyl)
		{
			found_gap = true;
			max_gap_cyl = abs(x_vector[i] - x_vector[i + 1]);
			cutting_x = abs(x_vector[i] - x_vector[i + 1]) / 2 + x_vector[i]; //We want to cut in the exact middle of the gap
		}
	}

	//In case there is a gap (i.e. we separated the flattened cyl patch in two portions)
	//we move the portion on the left such that it is sticked right on the right of the other one.
	if (found_gap)
	{
		for (std::size_t i = 0; i < num_points; ++i)
		{
			if (flattened_cloud->at(i).x <= cutting_x)
			{
				flattened_cloud->at(i).x = cyl_radius*(atan2(transformed_cyl_patch_cloud->at(i).y, transformed_cyl_patch_cloud->at(i).x) + 2 * M_PI);
			}
		}
	}
	//outputCloudOnTXT(flattened_cloud, "flat_cyl");
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\flattened_cloud.pcd", *flattened_cloud);
	//visualizePointCloud(flattened_cloud, "FLATTENED CYLINDER");
	//visualizePointCloud(flattened_cloud, "flattened_cylinder");



	return flattened_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformCylindricalPatchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cylindrical_patch_cloud, std::vector<float> cyl_param)
{
	Eigen::Matrix3f transform_matrix = getTransformMatrixForAlignmentWithCylinderAxis(cyl_param);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ptr_output_cloud = transformCloudByMatrix_cylinder(cylindrical_patch_cloud, transform_matrix, cyl_param);

	return ptr_output_cloud;
}

Eigen::Matrix3f getTransformMatrixForAlignmentWithCylinderAxis(std::vector<float> cyl_param)
{
	/*std::cerr << std::endl<< "Cylinder surface parameters: " << cyl_param.size() << " parameters" << std::endl;
	for (std::size_t i = 0; i < cyl_param.size(); ++i)
	std::cerr << cyl_param[i] << std::endl;*/

	//NORMALIZATION OF THE CYLINDER AXIS VECTOR??

	Eigen::Vector3f x_axis;
	Eigen::Vector3f y_axis;
	Eigen::Vector3f z_axis;

	//z axis definition	
	Eigen::Vector3f axis_direction(cyl_param[3], cyl_param[4], cyl_param[5]);
	for (std::size_t i = 0; i < 3; ++i)
	{
		z_axis[i] = axis_direction[i];
	}
	double norm_z = z_axis.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		z_axis[i] = z_axis[i] / norm_z;
	}
	//std::cerr << "z_axis = (" << z_axis[0] << "," << z_axis[1] << "," << z_axis[2] << ")" << std::endl;
	//std::cerr << "z_axis norm = " << z_axis.norm() << std::endl;

	//x axis definition: 
	//we find 2 points A,B on the plane: one is provided as a parameter (point on axis), the other one is computed
	//Then we consider as x axis the unit vector A-B 
	Eigen::Vector3f point_on_axis(cyl_param[0], cyl_param[1], cyl_param[2]);
	std::vector<float> plane_passing_through_point_on_axis_param = computePlanePassingThroughPointWithGivenNormal(point_on_axis, axis_direction);
	Eigen::Vector3f point_on_plane = computeAPointOnPlane(plane_passing_through_point_on_axis_param);
	Eigen::Vector3f dif = point_on_axis - point_on_plane;
	//std::cerr << "Difference:  (" << dif[0] << "," << dif[1] << "," << dif[2] << ")" << std::endl;
	double norm = dif.norm();
	for (std::size_t i = 0; i < 3; ++i)
		x_axis[i] = dif[i] / norm;
	//std::cerr << "x_axis = (" << x_axis[0] << "," << x_axis[1] << "," << x_axis[2] << ")" << std::endl;
	//std::cerr << "x_axis norm = " << x_axis.norm() << std::endl;

	//y axis definition
	y_axis = z_axis.cross(x_axis);
	//std::cerr << "y_axis = (" << y_axis[0] << "," << y_axis[1] << "," << y_axis[2] << ")" << std::endl;
	//std::cerr << "y_axis norm = " << y_axis.norm() << std::endl;

	//Get the transformation matrix referred to the new coordinate system
	Eigen::Matrix3f trasf_matrix = buildTransformMatrixFromAxis(x_axis, y_axis, z_axis);

	return trasf_matrix;
}

std::vector<float> computePlanePassingThroughPointWithGivenNormal(Eigen::Vector3f point_through, Eigen::Vector3f normal)
{
	std::vector<float> output_plane_param(4);
	float d = -(normal[0] * point_through[0] + normal[1] * point_through[1] + normal[2] * point_through[2]);
	for (std::size_t i = 0; i < 3; ++i)
		output_plane_param[i] = normal[i];
	output_plane_param[3] = d;
	return output_plane_param;
}





////////////////////////////////////////////////////////////////////////////////////////////////////
//CONE

Eigen::MatrixXf MainConicalPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud, std::vector<float> cone_param, bool *good_patch_marker_cone, int *count_flattened_cloud)
{
	//rotate the cone so that the axis aligns with the +Z axis, the origin point is the conic point
	pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud = transformConicalPatchPoints(conical_patch_cloud, cone_param);
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\build\\canonical_output_cloud.pcd", *canonical_output_cloud);
	//visualizePointCloud(canonical_output_cloud,"canonical_output_cloud");

	//cout << endl << " points.size_before = " << canonical_output_cloud->points.size() <<endl <<endl;
	//canonical_output_cloud->push_back(pcl::PointXYZ (500, 600,2000));
	//cout << endl << " points.size_after = " << canonical_output_cloud->points.size() <<endl <<endl;
	//visualizePointCloud(canonical_output_cloud,"canonical_output_cloud");

	//flatten 3d point cloud into XOZ-Plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud = FlattenCloud(canonical_output_cloud, cone_param);
	double alpha_shape_area = getAlphaShape(outputCloudOnTXT(flatten_cloud, "flattened_cone", (*count_flattened_cloud)++));
	std::cout << std::endl << "alpha_shape_area = " << alpha_shape_area << std::endl;

	Eigen::Vector2f flatten_point_i;
	float alpha;
	pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud1(new pcl::PointCloud<pcl::PointXYZ>), flatten_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<point> convex_hull_points, convex_hull_points1, convex_hull_points2;
	for (int i = 0; i < flatten_cloud->size(); i++)
	{
		flatten_point_i[0] = flatten_cloud->points[i].x;
		flatten_point_i[1] = flatten_cloud->points[i].z;
		alpha = atan2(flatten_point_i[1], flatten_point_i[0]);
		//cutting the fan into two halves, the first half of which lies in range (max_gap/2, (max_gap + beta_fan)/2) 
		if (alpha >= ((max_gap*0.5) / (2 * M_PI)) * beta_max - 0.1 &&  alpha <= ((max_gap / (2 * M_PI)) * beta_max + beta_fan)* 0.5)
		{
			flatten_cloud1->push_back(pcl::PointXYZ(flatten_point_i[0], 0, flatten_point_i[1]));
		}
		else
		{
			flatten_cloud2->push_back(pcl::PointXYZ(flatten_point_i[0], 0, flatten_point_i[1]));
		}
	}
	//visualizePointCloud(flatten_cloud, "flatten_cloud");
	//visualizePointCloud(flatten_cloud1, "flatten_cloud1");
	//visualizePointCloud(flatten_cloud2, "flatten_cloud2");

	convex_hull_points1 = convexHullForPoints(flatten_cloud1);
	convex_hull_points2 = convexHullForPoints(flatten_cloud2);
	convex_hull_points = convexHullForPoints(flatten_cloud);
	double convex_hull_area = getConvexHullArea(convex_hull_points);
	std::cout << "convex_hull_area = " << convex_hull_area << std::endl;
	std::cout << "alpha_shape_area/convex_hull_area = " << alpha_shape_area / convex_hull_area << std::endl;
	if (alpha_shape_area / convex_hull_area < CONE_AREA_PERCENTAGE)
	{
		*good_patch_marker_cone = 0;
		Eigen::MatrixXf cone_data(0, 0);
		return cone_data;
	}
	//std::cerr << "number of CH points = " << convex_hull_points.size() << endl;
	//visualizeConvexHull(flatten_cloud, "1hull_indexes2", hull_indexes2);
	//visualizeConvexHull(flatten_cloud, "1hull_indexes1", hull_indexes1);

	convex_hull_points1 = GenerateConvexHull(convex_hull_points1);
	convex_hull_points2 = GenerateConvexHull(convex_hull_points2);
	//hull_indexes2 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, convex_hull_points2);
	//visualizeConvexHull(flatten_cloud, "2hull_indexes2", hull_indexes2);
	//The function named sequenceConvexHull leads to the mess (when testing Cone_curved_generatrix_2.stl, there is nothing in candidate_line1)

	//hull_indexes1 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, convex_hull_points1);
	//visualizeConvexHull(flatten_cloud, "2hull_indexes1", hull_indexes1);
	convex_hull_points1 = SequenceConvexHull(convex_hull_points1);
	convex_hull_points2 = SequenceConvexHull(convex_hull_points2);

	/*hull_indexes1 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, convex_hull_points1);
	visualizeConvexHull(flatten_cloud, "3hull_indexes1", hull_indexes1);
	*/

	//get the data of points which are on the convex hull 
	std::vector<int> hull_indexes1 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, convex_hull_points1);
	std::vector<int> hull_indexes2 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, convex_hull_points2);
	std::vector<int> hull_indexes = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, convex_hull_points);

	/*visualizeConvexHull(flatten_cloud, "hull_indexes", hull_indexes);
	visualizeConvexHull(flatten_cloud, "4hull_indexes1", hull_indexes1);
	visualizeConvexHull(flatten_cloud, "hull_indexes2", hull_indexes2);*/



	std::vector<point> Candidate_points1 = ConeCandiateLines(convex_hull_points1);
	std::vector<point> Candidate_points2 = ConeCandiateLines(convex_hull_points2);

	std::vector<int>indexes1 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, Candidate_points1);
	std::vector<int>indexes2 = IdentifyIndexOfCandiatePointInFlattenCloud(flatten_cloud, Candidate_points2);

	std::vector<int> marker_chain1, marker_chain2;
	std::vector<int> cone_chain1 = IdentifyChains(canonical_output_cloud, indexes1, marker_chain1);
	std::vector<int> cone_chain2 = IdentifyChains(canonical_output_cloud, indexes2, marker_chain2);

	//visualizeShapes(flatten_cloud, "Candidate Lines1",hull_indexes1,indexes1);
	//visualizeShapes(flatten_cloud, "Candidate Lines2",hull_indexes2,indexes2);
	//visualizeShapes(flatten_cloud, "Chains1",hull_indexes1,cone_chain1);
	//visualizeShapes(flatten_cloud, "Chains2",hull_indexes2,cone_chain2);

	std::vector<int> indexes;
	for (int i = 0; i < indexes1.size(); i++)
	{
		indexes.push_back(indexes1[i]);
	}
	for (int i = 0; i < indexes2.size(); i++)
	{
		indexes.push_back(indexes2[i]);
	}

	std::vector<int> marker_chain;
	for (int i = 0; i < marker_chain1.size(); i++)
	{
		marker_chain.push_back(marker_chain1[i]);
	}
	for (int i = 0; i < marker_chain2.size(); i++)
	{
		marker_chain.push_back(marker_chain2[i] + marker_chain1[marker_chain1.size() - 1]);
	}

	std::vector<int> cone_chain;
	for (int i = 0; i < cone_chain1.size(); i++)
	{
		cone_chain.push_back(cone_chain1[i]);
	}
	for (int i = 0; i < cone_chain2.size(); i++)
	{
		cone_chain.push_back(cone_chain2[i]);
	}

	//Eigen::Vector2f point; 
	//const int num = hull_indexes.size();
	//for(std::size_t i = 0; i < num-1; i++)
	//{
	//	point[0]=flatten_cloud->at(hull_indexes[i]).x;
	//	point[1]=flatten_cloud->at(hull_indexes[i]).z;
	//	cout << "point = " << point[0] << " , " << point[1] << endl;
	//}



	//visualizeShapes(flatten_cloud, "Candidate Lines",hull_indexes,indexes);
	visualizeShapes(flatten_cloud, "cone chains", hull_indexes, cone_chain);

	const int num_marker_chains = marker_chain.size();
	const int num_cone_chain = cone_chain.size();
	Eigen::MatrixXf cone_data(num_marker_chains, 3);
	cerr << endl;
	cerr << "Indices of vertices extremes of cone chains (read 2 by 2):" << endl;
	for (int i = 0; i < num_cone_chain; i = i + 2)
	{
		cerr << cone_chain[i] << "  " << cone_chain[i + 1] << endl;
	}

	for (int iter = 0; iter< num_marker_chains; iter++)
	{
		cone_data(iter, 0) = indexes[2 * iter];
		cone_data(iter, 1) = indexes[2 * iter + 1];
		cone_data(iter, 2) = marker_chain[iter];
	}
	return cone_data;
}

std::vector<int> IdentifyChains(pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud, std::vector<int> &indexes, std::vector<int> &marker_chain)
{
	double norm_one, norm_second;
	bool flag = true, found = false, rearrange = false;
	int i = 0, j = 0, counter, marker = 1;
	std::vector<int> cone_chain;
	Eigen::Vector2f A, B, C, D, diff_one, diff_second;
	// To ensure that starting point to scan for a chain is such that we dont leave any part of chain behind
	// array of indexes is rearranged such that index[0] and index[size-1] cant form a chain
	/*while(indexes[0]==indexes[indexes.size()-1])
	{
	rearrange =true;
	int last=indexes[indexes.size()-1];
	int second_last=indexes[indexes.size()-2];
	indexes.pop_back();
	indexes.pop_back();
	indexes.insert(indexes.begin(),last);
	indexes.insert(indexes.begin(),second_last);

	}
	if (rearrange==true)
	{
	cerr<<"Indices of vertices to form candidate line (ordering rearranged, read 2 by 2): "<<endl;
	}
	for (int i = 0; rearrange==true && i < indexes.size(); i=i+2)
	{
	std::cerr << indexes[i] << "  " << indexes[i+1] << endl;
	}*/

	// Loop checks each two consequtive indexes to form a reference candidate line
	while (i < indexes.size())
	{
		A[0] = canonical_output_cloud->at(indexes[i]).x;
		A[1] = canonical_output_cloud->at(indexes[i]).y;
		B[0] = canonical_output_cloud->at(indexes[i + 1]).x;
		B[1] = canonical_output_cloud->at(indexes[i + 1]).y;
		norm_one = (B - A).norm();
		if (norm_one == 0.0)
		{
			continue;
		}
		//cout << "A: " << A[0] << " , " << A[1] << endl;
		//cout << "B: " << B[0] << " , " << B[1] << endl;
		//cout << "B-A: " << B[0]-A[0] << " , " << B[1]-A[1] << endl;
		//cout << "norm_one: " << norm_one << endl;

		for (std::size_t i = 0; i < 2; i++)
		{
			diff_one[i] = (B[i] - A[i]) / norm_one;
		}
		//cout << "diff_one: " << diff_one[0] << " , " << diff_one[1] << endl << endl;	
		j = i;
		counter = 0;

		// It checks further with next candidate lines, if the dot product is close it assumes it as a one chain
		while (j<indexes.size())
		{
			C[0] = canonical_output_cloud->at(indexes[j]).x;
			C[1] = canonical_output_cloud->at(indexes[j]).y;
			D[0] = canonical_output_cloud->at(indexes[j + 1]).x;
			D[1] = canonical_output_cloud->at(indexes[j + 1]).y;
			norm_second = (D - C).norm();
			if (norm_second == 0.0)
			{
				continue;
			}
			//cout << "c: " << C[0] << " , " << C[1] << endl;
			// cout << "d: " << D[0] << " , " << D[1] << endl;
			//cout << "D-C: " << D[0]-C[0] << " , " << D[1]-C[1] << endl;
			//cout << "norm_second: " << norm_second << endl;

			for (std::size_t i = 0; i < 2; i++)
			{
				diff_second[i] = (D[i] - C[i]) / norm_second;
			}
			//cout << "diff_second: " << diff_second[0] << " , " << diff_second[1] << endl;	
			//cout << "abs(diff_one.dot(diff_second)): " << abs(diff_one.dot(diff_second)) << endl << endl;	
			if (abs(diff_one.dot(diff_second))>0.9999)
			{
				counter += 2;
				if (flag == true)
				{
					marker_chain.push_back(marker);
					cone_chain.push_back(indexes[i]);
					cone_chain.push_back(indexes[j + 1]);
					flag = false;
				}
				else
				{
					marker_chain.push_back(marker);
					cone_chain.pop_back();
					cone_chain.push_back(indexes[j + 1]);
				}
				j = j + 2;
			}
			else
			{
				marker++;
				flag = true;
				found = false;
				break;
			}
		}
		i = i + counter;
	}

	if (cone_chain.size() == 0)
	{
		cone_chain = indexes;
	}

	return cone_chain;
}

std::vector<point> ConeCandiateLines(std::vector<point> convex_hull_points)
{
	const int num_hull_vertices = convex_hull_points.size();
	//cout << endl << "num_hull_vertices = " << num_hull_vertices << endl << endl;
	std::vector<point> Candidate_points;
	Eigen::Vector2f appex(0, 0), vector_point, vector_appex;
	double norm_appex, norm_point;
	float a, b, dist;
	Eigen::Vector2f point_iplus, point_i;
	// Since convex hull points are in sequence , we try of connect each points to see if its within some tolerance

	//bug
	for (std::size_t i = 0; i < num_hull_vertices - 1; i++)
	{
		// For each point we create a vector by joining the i th point to i+1th point
		point_i[0] = boost::get<0>(convex_hull_points[i]);
		point_i[1] = boost::get<1>(convex_hull_points[i]);
		point_iplus[0] = boost::get<0>(convex_hull_points[i + 1]);
		point_iplus[1] = boost::get<1>(convex_hull_points[i + 1]);
		//cout << "A: " << point_i[0] << " , " << point_i[1] << endl;
		//cout << "B: " << point_iplus[0] << " , " << point_iplus[1] << endl;
		Eigen::Vector2f dif_point = point_iplus - point_i;
		norm_point = dif_point.norm();
		if (norm_point == 0.0)
		{
			continue;
		}
		//cout << "norm_point: " << norm_point << endl;
		for (std::size_t i = 0; i < 2; ++i)
		{
			vector_point[i] = dif_point[i] / norm_point;
		}
		//cout << "vector_point: " << vector_point[0] << " , " << vector_point[1] << endl;



		// For each point we create a vector by joining the i th point to appex
		Eigen::Vector2f dif_appex = point_i - appex;
		norm_appex = dif_appex.norm();
		if (norm_appex == 0.0)
		{
			continue;
		}
		//cout << "A: " << point_i[0] << " , " << point_i[1] << endl;
		//cout << "O: " << appex[0] << " , " << appex[1] << endl;
		for (std::size_t i = 0; i < 2; ++i)
		{
			vector_appex[i] = dif_appex[i] / norm_appex;
		}
		//cout << "norm_appex: " << norm_appex << endl;
		//cout << "vector_appex: " << vector_appex[0] << " , " << vector_appex[1] << endl;

		// There might be a point in the point cloud which is extremly close to the appex. For such points it is essential to compute the distance 
		// rather than just the vector joining the appex and the point
		a = (point_iplus[1] - point_i[1]) / (point_iplus[0] - point_i[0]);
		dist = abs((a*point_i[0] - point_i[1]) / sqrt((a*a) + 1));
		//cout << "abs(vector_point.dot(vector_appex)): " << abs(vector_point.dot(vector_appex)) << endl << endl;

		// Candidate points will be an array such that consequtive two points forms a pair which can be joined to form a candidate points  
		if (abs(vector_point.dot(vector_appex))>0.99 || dist<TOLERANCE_FOR_APPEX)
		{
			Candidate_points.push_back(boost::make_tuple(point_i[0], point_i[1]));
			Candidate_points.push_back(boost::make_tuple(point_iplus[0], point_iplus[1]));
		}
	}
	return Candidate_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FlattenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud, std::vector<float> cone_param)
{
	float x_cordi, z_cordi, rad_1, rad_2, sine_angle = sin(cone_param[6]);
	Eigen::Vector3f point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Flattened_cloud_2D(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud = SortCloud(canonical_output_cloud, cone_param);
	const int num_nodes = sorted_cloud->points.size();
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\sorted_cloud.pcd", *sorted_cloud);
	//outputCloudOnExcel(sorted_cloud, "sorted_cloud" );

	float cutting_dirc = CuttingDirection(sorted_cloud);

	//Define a rotation matrix to allign cone such that cutting_dirc is along X axis and get a perfectly alligned cone	 
	Eigen::Matrix3f transform_1 = Eigen::Matrix3f::Identity();

	if (cutting_dirc<0)
	{
		transform_1(0, 0) = cos(abs(cutting_dirc));
		transform_1(0, 1) = -sin(abs(cutting_dirc));
		transform_1(1, 0) = sin(abs(cutting_dirc));
		transform_1(1, 1) = cos(abs(cutting_dirc));
	}
	else
	{
		transform_1(0, 0) = cos(abs(cutting_dirc));
		transform_1(0, 1) = sin(abs(cutting_dirc));
		transform_1(1, 0) = -sin(abs(cutting_dirc));
		transform_1(1, 1) = cos(abs(cutting_dirc));
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr exact_alligned_matrix = transformCloudByMatrix(canonical_output_cloud, transform_1);
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\build\\exact_alligned_matrix.pcd", *exact_alligned_matrix);
	//cout<<"cutting_dirc: "<<cutting_dirc<<endl<<endl;

	//visualizePointCloud(exact_alligned_matrix, "exact_alligned_matrix");
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr wierd_point_aligned (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("C:\\Extract_indices\\build\\wierd_point_aligned.pcd", *wierd_point_aligned);
	visualizeTwoPointClouds (exact_alligned_matrix, wierd_point_aligned, "Exact aligned cloud with wierd point");*/

	//cout << " sine_angle = " <<sine_angle<< endl<< endl;
	//cout << " cone_param[6] = " << cone_param[6] << endl<< endl;

	beta_max = 2 * M_PI*sine_angle; // The central angle of expanded fan-shape.	

	float beta;
	double theta;
	float theta_max = 2 * M_PI - max_gap; // The total revolution angle of the cone shape.
	beta_fan = (theta_max / (2 * M_PI))*beta_max;
	//cout << " beta_fan = " << beta_fan << endl;
	for (std::size_t i = 0; i < num_nodes - 1; ++i)
	{
		point = Eigen::Vector3f(exact_alligned_matrix->at(i).getArray3fMap());
		//rad_1 = sqrt(point[0]*point[0]+ point[1]*point[1]);
		rad_2 = sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
		theta = atan2(point[1], point[0]);

		// Since the atan2 return in (-pi,pi) to convert the theta values in (0,2*pi)
		if (theta < -2.4E-04)
		{
			theta = theta + 2 * M_PI;
		}

		beta = (theta / (2 * M_PI))*beta_max;

		x_cordi = rad_2*cos(beta);
		z_cordi = rad_2*sin(beta);
		/*if (abs(x_cordi + 2617.134) < 0.01 && abs(z_cordi - 44.214989) < 0.01)
		{
		cout << "x = " << point[0] << endl;
		cout << "y = " << point[1] << endl;
		cout << "z = " << point[2] << endl;
		}*/

		Flattened_cloud_2D->push_back(pcl::PointXYZ(x_cordi, 0, z_cordi));
		//Flattened_cloud_2D->push_back(pcl::PointXYZ(x_cordi * cos(-0.5*max_gap) - z_cordi * sin(-0.5*max_gap), 0, x_cordi * sin(-0.5*max_gap)+z_cordi * cos(-0.5*max_gap)));
	}
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\build\\Flattened_cloud_2D.pcd", *Flattened_cloud_2D);
	//visualizePointCloud(Flattened_cloud_2D, "flattened_cloud_cone");

	//outputCloudOnExcel(Flattened_cloud_2D, "Flattened_cloud_2D" );
	return Flattened_cloud_2D;
}

//find how many angles need to rotate till the cutting edge could rotate to x-axis
float CuttingDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud)
{
	const int num_nodes = sorted_cloud->points.size();
	float point_i, point_iplus, distance_i;
	float edge1, edge2;
	int edge1_index, edge2_index;
	int k, l;
	edge1 = sorted_cloud->at(num_nodes - 1).y;
	edge2 = sorted_cloud->at(0).y;
	edge1_index = num_nodes - 1;
	edge2_index = 0;
	max_gap = abs(2 * M_PI - (edge2 - edge1));
	//cout << "`max_gap = " << max_gap << endl << endl;

	for (std::size_t i = 0; i < num_nodes - 1; ++i)
	{
		point_i = sorted_cloud->at(i).y;
		point_iplus = sorted_cloud->at(i + 1).y;
		if (abs(point_iplus - point_i)> max_gap)
		{
			max_gap = abs(point_iplus - point_i);
			//cout << "1max_gap = " << max_gap << endl << endl;
			edge1 = point_i;//(point_iplus+point_i)/2 ;	// we will cut across the least value of theta considering the range of 0 to 2*pi
			edge2 = point_iplus;
			edge1_index = i;
			edge2_index = i + 1;
			//k = i;
		}
	}

	/*cout << "max_gap = " << max_gap << endl << endl;
	cout << " edge 1"<< endl;
	cout << " x[" << edge1_index << "] = " << sorted_cloud->at(edge1_index).x << endl << endl;
	cout << " y[" << edge1_index << "] = " << sorted_cloud->at(edge1_index).y << endl << endl;
	cout << " z[" << edge1_index << "] = " << sorted_cloud->at(edge1_index).z << endl << endl;

	cout << " edge 2"<< endl;
	cout << " x[" << edge2_index << "] = " << sorted_cloud->at(edge2_index).x << endl << endl;
	cout << " y[" << edge2_index << "] = " << sorted_cloud->at(edge2_index).y << endl << endl;
	cout << " z[" << edge2_index << "] = " << sorted_cloud->at(edge2_index).z << endl << endl;*/

	float max_distance1 = 0.0, max_distance2 = 0.0;
	float range = max_gap;

	//find max_distance near edge1
	if (edge1 + range <= M_PI)
	{
		for (std::size_t i = 0; i < num_nodes; ++i)
		{
			point_i = sorted_cloud->at(i).y;
			distance_i = sorted_cloud->at(i).x;
			if (point_i < edge1 + range && point_i > edge1)
			{
				if (distance_i > max_distance1)
				{
					max_distance1 = distance_i;
				}
			}
		}
	}
	else
	{
		for (std::size_t i = 0; i < num_nodes; ++i)
		{
			point_i = sorted_cloud->at(i).y;
			distance_i = sorted_cloud->at(i).x;

			if ((point_i <= M_PI && point_i > edge1) || (point_i > (-M_PI) && point_i < (edge1 + range - 2 * M_PI)))
			{
				if (distance_i > max_distance1)
				{
					max_distance1 = distance_i;
				}
			}
		}
	}
	//cout << "max_distance1 = " << max_distance1 << endl;

	//find cutting direction1
	float cutting_direction1;

	for (int i = edge1_index; i >= 0; --i)
	{
		if (sorted_cloud->at(i).x >= 0.8 * max_distance1)
		{
			cutting_direction1 = sorted_cloud->at(i).y;
			k = i;
			break;
		}
	}
	//update max_gap
	max_gap = max_gap + abs(cutting_direction1 - edge1);
	//cout << "cutting direction 1: "<< sorted_cloud->at(k).x<<" , "<<sorted_cloud->at(k).y<<" , "<<sorted_cloud->at(k).z<<endl<<endl;

	//find max_distance near edge2
	if (edge2 - range > -M_PI)
	{
		for (std::size_t i = 0; i < num_nodes; ++i)
		{
			point_i = sorted_cloud->at(i).y;
			distance_i = sorted_cloud->at(i).x;
			if (point_i > edge2 - range && point_i < edge2)
			{
				if (distance_i > max_distance2)
				{
					max_distance2 = distance_i;
				}
			}
		}
	}
	else
	{
		for (std::size_t i = 0; i < num_nodes; ++i)
		{
			point_i = sorted_cloud->at(i).y;
			distance_i = sorted_cloud->at(i).x;
			if ((point_i > edge2 - range && point_i < edge2) || (point_i <= M_PI && point_i >(edge2 - range + 2 * M_PI)))
			{
				if (distance_i > max_distance2)
				{
					max_distance2 = distance_i;
				}
			}
		}
	}
	//cout << "max_distance2 = " << max_distance2 << endl;

	//find cutting direction2
	float cutting_direction2;
	for (int i = edge2_index; i < num_nodes; ++i)
	{
		if (sorted_cloud->at(i).x >= 0.8 * max_distance2)
		{
			cutting_direction2 = sorted_cloud->at(i).y;
			k = i;
			break;
		}
	}
	//cout << "cutting direction 2: "<< sorted_cloud->at(k).x<<" , "<<sorted_cloud->at(k).y<<" , "<<sorted_cloud->at(k).z<<endl<<endl;

	//update max_gap
	max_gap = max_gap + abs(cutting_direction2 - edge2);
	//cout << "max_gap' = " << max_gap << endl << endl;

	//change the cutting_dirc so that the middle of gap is aligned with the x-axis
	float cutting_dirc = cutting_direction1;
	cutting_dirc -= 0.5 * max_gap;

	// If the it does not span across Pi. 
	// If differnce between the first and last point of the sorted array is the largest
	// We will cut across the least value of theta
	//if ( abs(abs(sorted_cloud->at(0).y)+abs(sorted_cloud->at(num_nodes-1).y)-2*M_PI)>.1)
	//{

	// (sorted_cloud->at(0).y+sorted_cloud->at(num_nodes-1).y)/2 +m_pi;
	//}

	//cerr<<endl<<cutting_dirc;
	//cout << endl << " max_gap = " <<max_gap <<endl <<endl;
	return cutting_dirc;
}

//get the point cloud in which the points are represented by polar coordinates and the second parameters(theta) is sorted in ascending order 
pcl::PointCloud<pcl::PointXYZ>::Ptr SortCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud, std::vector<float> cone_param)
{
	//get the polar coordinates of each point
	pcl::PointCloud<pcl::PointXYZ>::Ptr Collapsed_cloud = CollapsedConeCloud(canonical_output_cloud);
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\Collapsed_cloud.pcd", *Collapsed_cloud);
	//outputCloudOnExcel(Collapsed_cloud, "Collapsed_cloud");

	const int num_nodes = Collapsed_cloud->points.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Define a Matrix of size num_node*3 to store the point cloud
	std::vector<std::vector<double>> flat_points_matrix;
	flat_points_matrix.resize(num_nodes);
	for (int i = 0; i < num_nodes; ++i)
	{
		flat_points_matrix[i].resize(3);
		flat_points_matrix[i][0] = Collapsed_cloud->at(i).x;
		flat_points_matrix[i][1] = Collapsed_cloud->at(i).y;
		flat_points_matrix[i][2] = Collapsed_cloud->at(i).z;
	}

	//sort the points with polar coordinates according to the sequence of the length of the projective points on XOY plane(the first parameter of polar coordinates)
	sortrows(flat_points_matrix, 1);

	for (std::size_t i = 1; i < num_nodes; ++i)
	{
		sorted_cloud->push_back(pcl::PointXYZ(flat_points_matrix[i][0], flat_points_matrix[i][1], flat_points_matrix[i][2]));
	}
	//sorted_cloud->push_back (pcl::PointXYZ (100, -0.6, 2133.21));
	//sorted_cloud->push_back (pcl::PointXYZ (100, -2.55, 2309.63));
	return sorted_cloud;
}

void sortrows(std::vector<std::vector<double>>& flat_points_matrix, int col)
{
	std::sort(flat_points_matrix.begin(),
		flat_points_matrix.end(),
		[col](const std::vector<double>& lhs, const std::vector<double>& rhs) {
		return lhs[col] > rhs[col];
	});
}

//get the cylindrical coordinate system of the input cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr CollapsedConeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud)
{
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\canonical_output_cloud.pcd", *canonical_output_cloud);
	//visualizePointCloud(canonical_output_cloud, "canonical_output_cloud");
	const int num_nodes = canonical_output_cloud->points.size();
	float rad, theta, x_cordi, y_cordi;
	Eigen::Vector3f point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Collapsed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Collapsed_cloud_2D(new pcl::PointCloud<pcl::PointXYZ>);

	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(canonical_output_cloud->at(i).getArray3fMap());
		rad = sqrt(point[0] * point[0] + point[1] * point[1]);
		theta = atan2(point[1], point[0]);
		Collapsed_cloud->push_back(pcl::PointXYZ(rad, theta, point[2]));
		//std::cerr << "(" << Collapsed_cloud->at(i).x << "," << Collapsed_cloud->at(i).y << "," << Collapsed_cloud->at(i).z << ")" << std::endl;

		//project the point to XOY plane and then get the data of the projective point
		x_cordi = rad*cos(theta);
		y_cordi = rad*sin(theta);
		Collapsed_cloud_2D->push_back(pcl::PointXYZ(x_cordi, y_cordi, 0));
	}
	//visualizePointCloud(Collapsed_cloud_2D, "Collapsed cone");
	return Collapsed_cloud;
}

//rotate the cone so that the axis aligns with the +Z axis
pcl::PointCloud<pcl::PointXYZ>::Ptr transformConicalPatchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr conical_patch_cloud, std::vector<float> cone_param)
{
	Eigen::Matrix3f transform_matrix = getTransformMatrixForAlignmentWithConeAxis(cone_param);

	const int num_nodes = conical_patch_cloud->points.size();
	Eigen::Vector3f point, transformed_point;
	Eigen::Vector3f appex(cone_param[0], cone_param[1], cone_param[2]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr canonical_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(conical_patch_cloud->at(i).getArray3fMap());
		// Since Appex has to be put on the origin of cordinate frame
		transformed_point = transform_matrix* (point - appex);
		canonical_output_cloud->push_back(pcl::PointXYZ(transformed_point[0], transformed_point[1], transformed_point[2]));
	}
	//visualizePointCloud(canonical_output_cloud, "Canonical cone");
	//pcl::io::savePCDFileASCII("C:\\Extract_indices\\build\\canonical_output_cloud.pcd", *canonical_output_cloud);
	return canonical_output_cloud;
}

Eigen::Matrix3f getTransformMatrixForAlignmentWithConeAxis(std::vector<float> cone_param)
{
	//NORMALIZATION OF THE CONE AXIS VECTOR
	Eigen::Vector3f x_axis;
	Eigen::Vector3f y_axis;
	Eigen::Vector3f z_axis;

	//z axis definition	
	Eigen::Vector3f axis_direction(cone_param[3], cone_param[4], cone_param[5]);
	for (std::size_t i = 0; i < 3; ++i)
	{
		z_axis[i] = axis_direction[i];
	}
	double norm_z = z_axis.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		z_axis[i] = z_axis[i] / norm_z;
	}

	//x axis definition: 
	//we find 2 points A,B on the plane(defined by the normal and the appex point): one is provided as a parameter (Appex point), the other one is computed
	//Then we consider as x axis the unit vector A-B 
	Eigen::Vector3f point_on_axis(cone_param[0], cone_param[1], cone_param[2]);
	std::vector<float> plane_passing_through_point_on_axis_param = computePlanePassingThroughPointWithGivenNormal(point_on_axis, axis_direction);
	Eigen::Vector3f point_on_plane = computeAPointOnPlane(plane_passing_through_point_on_axis_param);
	Eigen::Vector3f dif = point_on_axis - point_on_plane;
	double norm = dif.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		x_axis[i] = dif[i] / norm;
	}

	//y axis definition
	y_axis = z_axis.cross(x_axis);

	//Get the transformation matrix referred to the new coordinate system
	Eigen::Matrix3f trasf_matrix = buildTransformMatrixFromAxis(x_axis, y_axis, z_axis);
	Eigen::Matrix3f trasf_matrix_inv = trasf_matrix.inverse();

	return trasf_matrix_inv;
}





//////////////////////////////////////Functions for the computation of convex Hull////////////////////////////////////////////

std::vector<point> convexHullForPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud)
{
	polygon poly = buildPolygonProjectingOnXZPlane(patch_cloud);

	return getConvexHullOfBoostPolygon(poly);
}

polygon buildPolygonProjectingOnXZPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud)
{
	const int num_of_points = patch_cloud->points.size();
	polygon poly;
	std::string poly_vertices_def("polygon((");
	for (std::size_t i = 0; i < num_of_points - 1; ++i)
	{
		poly_vertices_def = poly_vertices_def +
			boost::lexical_cast<std::string>(patch_cloud->at(i).x) +
			" " +
			boost::lexical_cast<std::string>(patch_cloud->at(i).z) +
			", ";
	}
	poly_vertices_def = poly_vertices_def +
		boost::lexical_cast<std::string>(patch_cloud->at(num_of_points - 1).x) +
		" " +
		boost::lexical_cast<std::string>(patch_cloud->at(num_of_points - 1).z) +
		"))";
	//std::cerr << poly_vertices_def << std::endl;

	boost::geometry::read_wkt(poly_vertices_def, poly);

	return poly;
}

double getConvexHullArea(std::vector<point> convex_hull_points)
{
	const int num_of_points = convex_hull_points.size();
	polygon poly;
	std::string poly_vertices_def("polygon((");
	for (std::size_t i = 0; i < num_of_points - 1; ++i)
	{
		poly_vertices_def = poly_vertices_def +
			boost::lexical_cast<std::string>(boost::get<0>(convex_hull_points[i])) +
			" " +
			boost::lexical_cast<std::string>(boost::get<1>(convex_hull_points[i])) +
			", ";
	}
	poly_vertices_def = poly_vertices_def +
		boost::lexical_cast<std::string>(boost::get<0>(convex_hull_points[num_of_points - 1])) +
		" " +
		boost::lexical_cast<std::string>(boost::get<1>(convex_hull_points[num_of_points - 1])) +
		"))";
	boost::geometry::read_wkt(poly_vertices_def, poly);
	return boost::geometry::area(poly);
}

std::vector<point> getConvexHullOfBoostPolygon(polygon poly)
{
	polygon hull;
	boost::geometry::convex_hull(poly, hull);
	using boost::geometry::dsv;
	// std::cout << "polygon: " << dsv(poly) << endl;
	// std::cout<< "hull: " << dsv(hull) << std::endl ;
	std::vector<point> const& convex_hull_points = hull.outer();
	return convex_hull_points;
}

std::vector<point> GenerateConvexHull(std::vector<point> convex_hull_points)
{
	std::vector<point> convex_hull_points_final;
	Eigen::Vector2f point_temp;
	float alpha_temp;
	for (int i = 0; i < convex_hull_points.size(); i++)
	{
		point_temp[0] = boost::get<0>(convex_hull_points[i]);
		point_temp[1] = boost::get<1>(convex_hull_points[i]);
		alpha_temp = atan2(point_temp[1], point_temp[0]);

		// to convert the range of alpha_temp from (-pi, pi) to (0,2pi)
		if (alpha_temp<0)
		{
			alpha_temp += 2 * M_PI;
		}

		//question 1
		if (abs(alpha_temp - ((max_gap*0.5) / (2 * M_PI)) * beta_max)<0.3 || abs(alpha_temp - (beta_fan + ((max_gap*0.5) / (2 * M_PI)) * beta_max))<0.3)
		{
			convex_hull_points_final.push_back(boost::make_tuple(point_temp[0], point_temp[1]));
		}
	}
	return convex_hull_points_final;
}

std::vector<point> SequenceConvexHull(std::vector<point> convex_hull_points)
{
	std::vector<point> convex_hull_points_final;
	int num_convex_hull = convex_hull_points.size();
	Eigen::Vector2f point_temp;
	//float length[100];

	std::vector<std::vector<double> > points_matrix;
	points_matrix.resize(num_convex_hull);
	for (int i = 0; i < num_convex_hull; ++i)
	{
		points_matrix[i].resize(3);
		points_matrix[i][0] = boost::get<0>(convex_hull_points[i]);
		points_matrix[i][1] = boost::get<1>(convex_hull_points[i]);
		points_matrix[i][2] = points_matrix[i][0] * points_matrix[i][0] + points_matrix[i][1] * points_matrix[i][1];
		//cout << "before: " << points_matrix[i][0] << "  " << points_matrix[i][1] << "  " << points_matrix[i][2] <<endl<<endl;
	}

	//sort the points with polar coordinates according to the sequence of the length of the projective points on XOY plane(the first parameter of polar coordinates)
	sortrows(points_matrix, 2);
	Eigen::Vector2f point_i;
	for (std::size_t i = 0; i < num_convex_hull; ++i)
	{
		point_i[0] = points_matrix[i][0];
		point_i[1] = points_matrix[i][1];
		//cout << "after : " << point_i[0] << "  " << point_i[1] << "  " << point_i[0]*point_i[0]+point_i[1]*point_i[1]<<endl<<endl;
		convex_hull_points_final.push_back(boost::make_tuple(point_i[0], point_i[1]));
	}
	return convex_hull_points_final;
}


//int IdentifyIndexOfAPointInCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<point> point )
//{
//	int index;
//	const int num_cloud = cloud->points.size();
//	const int num_point= point.size();
//	float point_x, point_y, point_z, cloud_x, cloud_y, cloud_z;
//	for(std::size_t i = 0; i < num_hull_vertices; i++)
//	{
//		point_x=boost::get<0>(point[i]);
//		point_y=boost::get<1>(point[i]);
//		point_z=boost::get<2>(point[i]);
//		for(std::size_t j = 0; j < num_nodes; j++)
//		{	
//			cloud_x = cloud->at(j).x;
//			cloud_y = cloud->at(j).y;
//			cloud_z = cloud->at(j).z;
//
//			if (FEQ(point_x, cloud_x)==1 && FEQ(point_y, cloud_y)==1 && FEQ(point_z, cloud_z)==1)
//			{
//				index = j;
//				break;
//			}
//		}
//	}
//	return index;
//}

std::vector<int> IdentifyIndexOfCandiatePointInFlattenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud, std::vector<point> Candidate_points)
{
	std::vector<int> indexes;
	const int num_nodes = flatten_cloud->points.size();
	const int num_hull_vertices = Candidate_points.size();
	float convex_x, convex_z, flat_x, flat_z;
	for (std::size_t i = 0; i < num_hull_vertices; i++)
	{
		convex_x = boost::get<0>(Candidate_points[i]);
		convex_z = boost::get<1>(Candidate_points[i]);
		for (std::size_t j = 0; j < num_nodes; j++)
		{
			flat_x = flatten_cloud->at(j).x;
			flat_z = flatten_cloud->at(j).z;

			if (FEQ(convex_x, flat_x) == 1 && FEQ(convex_z, flat_z) == 1)
			{
				indexes.push_back(j);
				break;
			}
		}
	}
	return indexes;
}







///////////////////////////////////////////////////////////////Miscellaneous Functions/////////////////////////////////////////
//MERGING PATCHES

void merge_patches(Eigen::MatrixXf *patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr* sourceClouds, int num_patch, double tolerance)
{
	int start, end, r_count = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector3f point_start, point_end, point;
	for (std::size_t i = 0; i<num_patch - 1; ++i)
	{
		cerr << "----->>> New patch: position " << i << endl;
		Eigen::MatrixXf patch_one = patch_data[i];
		start = 0;

		for (std::size_t j = 0; j<patch_one.rows(); ++j)
		{
			// We will create a candidate line such that it joins first and last vertices of a candidate chain
			//(start is the index of the head of the chain, end is the tail of the chain)
			//Thus we are skipping until there is a change in the marker (the index that identifies the chain)
			if (j<patch_one.rows() - 1 && patch_one(j, 2) == patch_one(j + 1, 2))
				continue;
			cloud = sourceClouds[i];
			end = j;

			cerr << "----->>> start index: " << start << endl;
			cerr << "----->>> end index: " << end << endl;
			for (std::size_t z = i + 1; z<num_patch; ++z)
			{
				cerr << "----------->>> second patch: position " << z << endl;
				ComputePatchDistance(start, end, patch_one, patch_data[z], cloud, sourceClouds[z], tolerance, r_count);
			}
			start = j + 1;
		}
	}
	if (r_count == 0)
		cerr << "----->>> Not Found any mutual boundary for cylindrical and Conical Patches" << endl;
}
#pragma region PREVIOUS VERSION OF merge_patches BY PARAS
//void merge_patches(	Eigen::MatrixXf *patch_data ,pcl::PointCloud<pcl::PointXYZ>::Ptr* sourceClouds, int num_patch, double tolerance)
//{
//	int start, end,r_count=0;  
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	Eigen::Vector3f point_start, point_end, point;
//	for (std::size_t i=0; i<num_patch; ++i)
//	{	
//		Eigen::MatrixXf patch_one = patch_data[i];
//		start=0;
//		
//		for (std::size_t j=0; j<patch_one.rows(); ++j)
//		{
//			end=j;
//			// We will create a candidate line such that it joins first and last vertices of a candidate chain
//			//(start is the index of the head of the chain, end is the tail of the chain)
//			//Thus we are skipping until there is a change in the marker (the index that identifies the chain)
//			if (j<patch_one.rows()-1 && patch_one(j,2)==patch_one(j+1,2) )
//				continue ;
//			cloud=sourceClouds[i];
//			
//			for (std::size_t z=0; z<num_patch; ++z)
//			{	
//				//we will check each patch to all the other patch except itself
//				if (i==z) 
//					//z++;
//					continue;
//				ComputePatchDistance(start, end, patch_one,patch_data[z],cloud,sourceClouds[z], tolerance,r_count);
//			}
//			start=j+1;
//		}
//	}
//	if (r_count==0)
//		cerr<<"----->>> Not Found any mutual boundary for cylindrical and Conical Patches";
//} 
#pragma endregion

void ComputePatchDistance(int start, int end, Eigen::MatrixXf patch_one, Eigen::MatrixXf patch_second, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second, double tolerance, int &r_count)
{
	int counter = 1, flag = 0;
	float max_dist = 0, dist, norm_num, norm_denom;
	Eigen::Vector3f point_start, point_end, point_first, point_second, point_line;
	point_start = Eigen::Vector3f(cloud_one->at(patch_one(start, 0)).getArray3fMap());
	std::vector<float> line_param_one(6);
	std::vector<float> line_param_second(6);
	struct result data[10];

	point_end = Eigen::Vector3f(cloud_one->at(patch_one(end, 1)).getArray3fMap());
	for (std::size_t i = 0; i<patch_second.rows(); ++i)
	{
		point_first = Eigen::Vector3f(cloud_second->at(patch_second(i, 0)).getArray3fMap());
		norm_num = ((point_first - point_start).cross(point_end - point_start)).norm();
		norm_denom = (point_end - point_start).norm();
		dist = norm_num / norm_denom;

		if (dist> max_dist)
			max_dist = abs(dist);

		point_second = Eigen::Vector3f(cloud_second->at(patch_second(i, 1)).getArray3fMap());
		norm_num = ((point_second - point_start).cross(point_end - point_start)).norm();
		norm_denom = (point_end - point_start).norm();
		dist = norm_num / norm_denom;
		if (dist> max_dist)
			max_dist = abs(dist);

		// we will measure the max of the perpendicular distance of all the points that lie on the same chain of the second patch
		// Only after the chain marker changes we will make the max-sit =0 and search again
		if (i<patch_second.rows() - 1 && patch_second(i, 2) == patch_second(i + 1, 2))
		{
			counter++;
			continue;
		}
		if (max_dist< tolerance)
		{
			cerr << endl << ">>>MATCH FOUND BETWEEN 2 PATCHES! Max distance= " << max_dist << endl;
			data[r_count].cloud_one = cloud_one;
			data[r_count].cloud_two = cloud_second;
			data[r_count].indexes_one = patch_one.block(start, 0, end - start + 1, 2);
			//data[r_count].indexes_two=patch_second.block(i-1,0,counter,2);
			cerr << endl << "flag =  " << flag << endl;
			cerr << endl << "counter =  " << counter << endl;
			data[r_count].indexes_two = patch_second.block(flag, 0, counter, 2);
			line_param_one.push_back(point_start[0]);
			line_param_one.push_back(point_start[1]);
			line_param_one.push_back(point_start[2]);
			line_param_one.push_back(point_end[0]);
			line_param_one.push_back(point_end[1]);
			line_param_one.push_back(point_end[2]);
			data[r_count].line_param_one = line_param_one;
			point_line = Eigen::Vector3f(cloud_second->at(patch_second(flag, 0)).getArray3fMap());
			line_param_second.push_back(point_line[0]);
			line_param_second.push_back(point_line[1]);
			line_param_second.push_back(point_line[2]);
			line_param_second.push_back(point_second[0]);
			line_param_second.push_back(point_second[1]);
			line_param_second.push_back(point_second[2]);
			data[r_count].line_param_second = line_param_second;
			r_count++;
			break;
		}
		else
		{
			cerr << endl << ">>>Match NOT found with these two border lines. Max distance= " << max_dist << endl;
			flag = i + 1;
			max_dist = 0;
			counter = 1;
		}

	}
}

//void ComputePatchDistance(int start,int end,Eigen::MatrixXf patch_one,Eigen::MatrixXf patch_second,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one,
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second, double tolerance, int &r_count)
//{
//	int counter=1, flag=0;
//	float max_dist=0, dist, norm_num,norm_denom;
//	Eigen::Vector3f point_start, point_end,point_first, point_second, point_line;
//	point_start=Eigen::Vector3f(cloud_one->at(patch_one(start,0)).getArray3fMap());
//	std::vector<float> line_param_one(6);
//	std::vector<float> line_param_second(6);
//	std::size_t i=0;
//	point_end=Eigen::Vector3f(cloud_one->at(patch_one(end,1)).getArray3fMap());for (std::size_t i=0; i<patch_second.rows(); i++)
//	for (; i<patch_second.rows(); i++)
//	{
//		point_first = Eigen::Vector3f(cloud_second->at(patch_second(i,0)).getArray3fMap());
//		norm_num= ((point_first-point_start).cross(point_end-point_start)).norm();
//		norm_denom= (point_end-point_start).norm();
//		dist=norm_num/norm_denom;
//		
//		if (dist> max_dist)
//			max_dist=abs(dist);
//				
//		point_second = Eigen::Vector3f(cloud_second->at(patch_second(i,1)).getArray3fMap());
//		norm_num= ((point_second-point_start).cross(point_end-point_start)).norm();
//		norm_denom= (point_end-point_start).norm();
//		dist=norm_num/norm_denom;
//		if (dist> max_dist)
//			max_dist=abs(dist);
//		
//		// we will measure the max of the perpendicular distance of all the points that lie on the same chain of the second patch
//		// Only after the chain marker changes we will make the max-sit =0 and search again
//		if (i<patch_second.rows()-1 && patch_second(i,2)==patch_second(i+1,2))
//		{
//			counter++;
//			continue;
//		}
//		if (max_dist< tolerance)
//		{
//			cerr<<endl<<endl;
//			cerr<<"///////////////////////////"<<"match found/////////////////////// ";
//			cerr<<endl<<"Max distance= "<< max_dist<<endl<<endl;		
//			data[r_count].cloud_one=cloud_one;
//			data[r_count].cloud_two=cloud_second;
//			data[r_count].indexes_one=patch_one.block(start,0,end-start+1,2);
//			data[r_count].indexes_two=patch_second.block(i-1,0,counter,2);
//				line_param_one.push_back(point_start[0]);
//				line_param_one.push_back(point_start[1]);
//				line_param_one.push_back(point_start[2]);
//				line_param_one.push_back(point_end[0]);
//				line_param_one.push_back(point_end[1]);
//				line_param_one.push_back(point_end[2]);
//			data[r_count].line_param_one=line_param_one;
//			point_line = Eigen::Vector3f(cloud_second->at(patch_second(flag,0)).getArray3fMap());
//				line_param_second.push_back(point_line[0]);
//				line_param_second.push_back(point_line[1]);
//				line_param_second.push_back(point_line[2]);
//				line_param_second.push_back(point_second[0]);
//				line_param_second.push_back(point_second[1]);
//				line_param_second.push_back(point_second[2]);
//			data[r_count].line_param_second=line_param_second;	
//			r_count++;
//		}
//		flag=i;
//		cerr<< "Max distance: "<<max_dist<<endl;
//		max_dist=0 ;
//		counter=1 ;
//	}
//}
