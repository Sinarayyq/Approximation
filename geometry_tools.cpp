
#include "geometry_tools.h"


Eigen::Matrix3f buildTransformMatrixFromAxis(Eigen::Vector3f x_axis, Eigen::Vector3f y_axis, Eigen::Vector3f z_axis)
{
	using namespace Eigen;

	Matrix3f transform_matrix;
	transform_matrix << x_axis, y_axis, z_axis;
	//std::cout << "Transformation matrix: "  << std::endl;
	//std::cout << transform_matrix << std::endl;

	return transform_matrix;
}

Eigen::Vector3f transformPointByMatrix(Eigen::Matrix3f transform_matrix, Eigen::Vector3f point_coord)
{
	return transform_matrix*point_coord;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud, Eigen::Matrix3f transform_matrix)
{
	const int num_nodes = patch_cloud->points.size();
	Eigen::Vector3f point, transformed_point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_patch_cloud (new pcl::PointCloud<pcl::PointXYZ>);


	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(patch_cloud->at(i).getArray3fMap());
		transformed_point = transform_matrix*point;
		// OR transformed_point = transformPointByMatrix(transform_matrix, point);
		transformed_patch_cloud->push_back (pcl::PointXYZ (transformed_point[0], transformed_point[1], transformed_point[2]));
	}

	return transformed_patch_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix_cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud,Eigen::Matrix3f transform_matrix, std::vector<float> cyl_param)
{
	const int num_nodes = patch_cloud->points.size();
	Eigen::Vector3f point, transformed_point, point_on_axis(cyl_param[0], cyl_param[1], cyl_param[2]), translation_vector;
	translation_vector = transform_matrix.inverse()*point_on_axis;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_patch_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(patch_cloud->at(i).getArray3fMap());
		transformed_point = transform_matrix.inverse()*point-translation_vector;
		transformed_patch_cloud->push_back (pcl::PointXYZ (transformed_point[0], transformed_point[1], transformed_point[2]));
	}
	return transformed_patch_cloud;
}

template <typename Point> void list_coordinates(Point const& p) 
{ 
    using boost::geometry::get; 
    
    std::cout << "x = " << get<0>(p) << " y = " << get<1>(p) << std::endl; 

} 



