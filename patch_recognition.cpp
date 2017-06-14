#include "patch_recognition.h"


int PlaneRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int threshold_inliers,
	                 pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients)
{
	std::cerr << std::endl << "Plane recognition:" << std::endl;
	//readParameterFile(PATH_HEAD + "\\source\\input extract indices.txt");

	// create the segmentation object for plane
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// set all the parameters for the segmentation object for plane
	setSegmentationParametersForPlane(seg);
	// segment the largest planar component from the remaining cloud
	seg.setInputCloud(cloud);
	seg.segment(**inliers, **coefficients);

	int num_inliers = (*inliers)->indices.size();
	if (num_inliers == 0)
	{
		std::cerr << "no plane found." << num_inliers << std::endl << std::endl;
		return 0;
	}

	std::cerr << "found a plane! number of inliers = " << num_inliers  << std::endl;
	if (num_inliers < threshold_inliers)
	{
		std::cerr << num_inliers << " < " << threshold_inliers << ". min num of inliers not reached ----->>> patch discarded" << std::endl << std::endl << std::endl;
		return 0;
	}

	// extract the inliers
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(*inliers);
	extract.setNegative(false);
	extract.filter(*cloud_plane);*/
	//std::cerr << "pointcloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	
	return num_inliers;
}

int FindPlaneBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, 
	                pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, 
	                int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{

	// print the coefficients of the found plane model:
	std::cerr << "coef plane: " << std::endl;
	std::cerr << "Ax + By + Cz + D = 0" << std::endl;
	std::cerr << "A: " << coefficients->values[0] << std::endl;
	std::cerr << "B: " << coefficients->values[1] << std::endl;
	std::cerr << "C: " << coefficients->values[2] << std::endl;
	std::cerr << "D: " << coefficients->values[3] << std::endl;


	// extract the inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

	extract.setInputCloud(*cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_plane);

	// visualize the fitted points
	visualizePointCloud(*cloud, cloud_plane, "planar patch", xy);


	//Project the inliers on the RANSAC parametric model.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_plane);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	(*cloud_plane).swap(*cloud_projected);


	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pla_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_pla_patch_cloud = transformPlanarPatchPoints(cloud_plane, coefficients->values);

	//alpha shape
	double alpha_shape_area = getAlphaShapeArea(transformed_pla_patch_cloud);
	displayAlphaShape(transformed_pla_patch_cloud);
	//std::cout << "alpha_shape_area = " << alpha_shape_area << std::endl;

	//border
	double convex_hull_area;
	Eigen::MatrixXf temp_patch_data = MainPlanarPatch(transformed_pla_patch_cloud, coefficients->values, &convex_hull_area);
	//std::cout << "convex_hull_area = " << convex_hull_area << std::endl;


	if (alpha_shape_area / convex_hull_area < PLANE_AREA_PERCENTAGE)
	{
		std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
		return -1;
	}
	(*patch_data)[*patch_count] = temp_patch_data;
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	*sourceCloud = *cloud_plane;
	(*sourceClouds)[*patch_count] = sourceCloud;
	(*patch_count)++;


	// update the cloud of points still to fit in a patch
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr remaining_cloud_normals(new pcl::PointCloud<pcl::Normal>);

	extract.setNegative(true);
	extract.filter(*remaining_cloud);
	(*cloud).swap(remaining_cloud);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(*cloud_normals);
	extract_normals.setIndices(inliers);
	extract_normals.filter(*remaining_cloud_normals);
	(*cloud_normals).swap(remaining_cloud_normals);


	

	std::cerr << std::endl << "------ remaining points: " << (*cloud)->size() << " data points." << std::endl << std::endl;

	return cloud_plane->points.size();

}




int CylinderRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, 
	                    int threshold_inliers, pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients)
{
	std::cerr << std::endl << "Cylinder recognition:" << std::endl;
	//readParameterFile(PATH_HEAD + "\\source\\input extract indices.txt");

	// create the segmentation object
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cyl;
	setSegmentationParametersForCylinder(seg_cyl);
	// segment the largest cylindrical component from the remaining cloud
	seg_cyl.setInputCloud(cloud);
	seg_cyl.setInputNormals(cloud_normals);
	seg_cyl.segment(**inliers, **coefficients);
	int num_inliers = (*inliers)->indices.size();

	if (num_inliers == 0)
	{
		std::cerr << "no cylinder found." << num_inliers << std::endl;
		return 0;
	}

	std::cerr << "found a cylinder! number of inliers = " << num_inliers << std::endl;
	if (num_inliers < threshold_inliers)
	{
		std::cerr << num_inliers << " < " << threshold_inliers << ". min num of inliers not reached ----->>> patch discarded" << std::endl << std::endl << std::endl;
		return 0;
	}

	//// extract the inliers
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud);
	//extract.setIndices(*inliers);
	//extract.setNegative(false);
	//extract.filter(*cloud_cylinder);

	
	return num_inliers;

}

int FindCylinderBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	                   pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients,
	                   int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{
	// print the coefficients of the found cylinder model:
	std::cerr << std::endl;
	std::cerr << "coef cylinder: " << std::endl;
	std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
	std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
	std::cerr << "radius:       " << coefficients->values[6] << std::endl << std::endl;
	std::cerr << std::endl;



	// extract the inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

	extract.setInputCloud(*cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_cylinder);

	visualizePointCloud(*cloud, cloud_cylinder, "cylindrical patch", xy);

	//Project the inliers on the RANSAC parametric model.
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	proj.setModelType(pcl::SACMODEL_CYLINDER);
	proj.setInputCloud(cloud_cylinder);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	(*cloud_cylinder).swap(*cloud_projected);


	//We flatten the cloud corresponding to the cylindrical patch 
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_cyl_patch_cloud = transformCylindricalPatchPoints(cloud_cylinder, coefficients->values);
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	flattened_cloud = flattenCylindricalPatch(transformed_cyl_patch_cloud, coefficients->values);
	
	//alpha shape
	double alpha_shape_area = getAlphaShapeArea(flattened_cloud);
	displayAlphaShape(flattened_cloud);
	//std::cout << "alpha_shape_area = " << alpha_shape_area << std::endl;

	//convex hull
	double convex_hull_area;
	Eigen::MatrixXf temp_patch_data = MainCylindricalPatch(transformed_cyl_patch_cloud, flattened_cloud, coefficients->values, &convex_hull_area);
	//std::cout << "convex_hull_area = " << convex_hull_area << std::endl;

	if (alpha_shape_area / convex_hull_area < CYL_AREA_PERCENTAGE)
	{
		std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
		return -1;
	}

	(*patch_data)[*patch_count] = temp_patch_data;
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	*sourceCloud = *cloud_cylinder;
	(*sourceClouds)[*patch_count] = sourceCloud;
	(*patch_count)++;



	// update the cloud of points still to fit in a patch
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr remaining_cloud_normals(new pcl::PointCloud<pcl::Normal>);
	extract.setNegative(true);
	extract.filter(*remaining_cloud);
	(*cloud).swap(remaining_cloud);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(*cloud_normals);
	extract_normals.setIndices(inliers);
	extract_normals.filter(*remaining_cloud_normals);
	(*cloud_normals).swap(remaining_cloud_normals);

	
	std::cerr << "------ remaining points: " << (*cloud)->size() << " data points." << std::endl << std::endl;
	return cloud_cylinder->points.size();
}

//void CylinderRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
//	int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
//{
//	while ((*cloud)->points.size() > threshold_inliers)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f(new pcl::PointCloud<pcl::Normal>);
//		
//		// Define 
//		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//		
//
//		// Create the filtering object
//		pcl::ExtractIndices<pcl::PointXYZ> extract;
//		pcl::ExtractIndices<pcl::Normal> extract_normals;
//		readParameterFile(PATH_HEAD + "\\source\\input extract indices.txt");
//		
//		
//
//		
//
//		std::cerr << "Input 0 if you think the panel is good enough." << std::endl;
//		std::cerr << "Input 1 if you want to change the tolerence and try cylinder recognition again." << std::endl;
//		std::cerr << "Input 2 if you want to try other recognition methods." << std::endl;
//
//		int check_again;
//		std::cin >> check_again;
//		cin.clear();
//		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//		if (check_again == 2)
//		{
//			break;
//		}
//		else
//		{
//			if (check_again == 1)
//			{
//				continue;
//			}
//		}
//
//
//
//		
//
//		// update the cloud of points still to fit in a patch
//		extract.setNegative(true);
//		extract.filter(*cloud_f);
//		(*cloud).swap(cloud_f);
//		extract_normals.setNegative(true);
//		extract_normals.setInputCloud(*cloud_normals);
//		extract_normals.setIndices(inliers);
//		extract_normals.filter(*cloud_normals_f);
//		(*cloud_normals).swap(cloud_normals_f);
//
//		
//	
//		// check validity of the patch (discard strips and check if there are groups of fitted points to be separated)
//		/*if(checkpatchvalidity("cylinder", cloud_p, cloud_filtered))
//		{
//
//		}*/
//
//
//		
//		//std::cerr << "------ remaining normals: " << all_cloud_normals->size() << " data points." << std::endl;
//
//	}
//}



int ConeRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
	                int threshold_inliers, pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients)
{
	std::cerr << std::endl << "Cone recognition:" << std::endl;

	// Create the segmentation object for cone
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
	// Set all the parameters for cone segmentation object
	setSegmentationParametersForCone(seg_cone);
	
	// Segment the largest conical component from the remaining cloud
	seg_cone.setInputCloud(cloud);
	seg_cone.setInputNormals(cloud_normals);
	seg_cone.segment(**inliers, **coefficients);
	int num_inliers = (*inliers)->indices.size();

	if (num_inliers == 0)
	{
		std::cerr << "NO CONE FOUND." << num_inliers << std::endl;
		return 0;
	}

	std::cerr << "FOUND A CONE! Number of inliers = " << num_inliers << std::endl;
	if (num_inliers < threshold_inliers)
	{
		std::cerr << num_inliers << " < " << threshold_inliers << ". Min num of inliers not reached ----->>> PATCH DISCARDED" << std::endl << std::endl << std::endl;
		return 0;
	}

	//// Extract the inliers
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cone(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud);
	//extract.setIndices(*inliers);
	//extract.setNegative(false);
	//extract.filter(*cloud_cone);

	
	return num_inliers;
}

int FindConeBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	               pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients,
	               int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{
	// Print the coefficients of the found cylinder model:	
	std::cerr << std::endl;
	std::cerr << "coef cone: " << std::endl;
	std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
	std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
	std::cerr << "semi-angle:   " << coefficients->values[6] << std::endl;
	std::cerr << std::endl;


	// extract the inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cone(new pcl::PointCloud<pcl::PointXYZ>);
	extract.setInputCloud(*cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_cone);


	// Visualize the fitted points
	visualizePointCloud(*cloud, cloud_cone, "found conical patch", xy);


	//Project the inliers on the RANSAC parametric model.
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	proj.setModelType(pcl::SACMODEL_CONE);
	proj.setInputCloud(cloud_cone);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	(*cloud_cone).swap(*cloud_projected);



	//rotate the cone so that the axis aligns with the +Z axis, the origin point is the conic point
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cone_patch_cloud = transformConicalPatchPoints(cloud_cone, coefficients->values);
	//flatten 3d point cloud into XOZ-Plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud = FlattenCloud(transformed_cone_patch_cloud, coefficients->values);


	//alpha shape
	double alpha_shape_area = getAlphaShapeArea(flattened_cloud);
	displayAlphaShape(flattened_cloud);
	//std::cout << "alpha_shape_area = " << alpha_shape_area << std::endl;

	//convex hull
	double convex_hull_area;
	Eigen::MatrixXf temp_patch_data = MainConicalPatch(transformed_cone_patch_cloud, flattened_cloud, coefficients->values, &convex_hull_area);
	//std::cout << "convex_hull_area = " << convex_hull_area << std::endl;

	if (alpha_shape_area / convex_hull_area < CONE_AREA_PERCENTAGE)
	{
		std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
		return -1;
	}

	(*patch_data)[*patch_count] = temp_patch_data;
	//patch_data[patch_count] = MainConicalPatch(cloud_p, coefficients->values);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	*sourceCloud = *cloud_cone;
	(*sourceClouds)[*patch_count] = sourceCloud;
	(*patch_count)++;



	// update the cloud of points still to fit in a patch
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr remaining_cloud_normals(new pcl::PointCloud<pcl::Normal>);
	extract.setNegative(true);
	extract.filter(*remaining_cloud);
	(*cloud).swap(remaining_cloud);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(*cloud_normals);
	extract_normals.setIndices(inliers);
	extract_normals.filter(*remaining_cloud_normals);
	(*cloud_normals).swap(remaining_cloud_normals);


	std::cerr << "------ Remaining points: " << (*cloud)->size() << " data points." << std::endl << std::endl;

	return cloud_cone->points.size();
}

//void ConeRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
//	int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
//{
//	while ((*cloud)->points.size() > threshold_inliers)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f(new pcl::PointCloud<pcl::Normal>);
//		
//		// Define 
//		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
//		
//		// Create the filtering object
//		pcl::ExtractIndices<pcl::PointXYZ> extract;
//		pcl::ExtractIndices<pcl::Normal> extract_normals;
//		readParameterFile(PATH_HEAD + "\\source\\input extract indices.txt");
//		// Create the segmentation object for cone
//		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
//		// Set all the parameters for cone segmentation object
//		setSegmentationParametersForCone(seg_cone);
//		std::cerr << std::endl << "Cone recognition:" << std::endl;
//
//		// Segment the largest conical component from the remaining cloud
//		seg_cone.setInputCloud(*cloud);
//		seg_cone.setInputNormals(*cloud_normals);
//		seg_cone.segment(*inliers, *coefficients);
//		int num_inliers = inliers->indices.size();
//
//		if (num_inliers == 0)
//		{
//			std::cerr << std::endl;
//			std::cerr << "NO CONE FOUND." << num_inliers << std::endl;
//			break;
//		}
//
//		std::cerr << std::endl;
//		std::cerr << "FOUND A CONE! Number of inliers = " << num_inliers << std::endl;
//		if (num_inliers < threshold_inliers)
//		{
//			std::cerr << num_inliers << " < " << threshold_inliers << ". Min num of inliers not reached ----->>> PATCH DISCARDED" << std::endl << std::endl << std::endl;
//			break;
//		}
//
//		// Extract the inliers
//		extract.setInputCloud(*cloud);
//		extract.setIndices(inliers);
//		extract.setNegative(false);
//		extract.filter(*cloud_p);
//
//		// Visualize the fitted points
//		//visualizePointCloud(cloud_p,"found conical patch");
//		visualizePointCloud(*cloud, cloud_p, "found conical patch", xy);
//		
//		std::cerr << "Input 0 if you think the panel is good enough." << std::endl;
//		std::cerr << "Input 1 if you want to change the tolerence and try cone recognition again." << std::endl;
//		std::cerr << "Input 2 if you want to try other recognition methods." << std::endl;
//
//		int check_again;
//		std::cin >> check_again;
//		cin.clear();
//		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//		if (check_again == 2)
//		{
//			break;
//		}
//		else
//		{
//			if (check_again == 1)
//			{
//				continue;
//			}
//		}
//
//
//		// Determine the possible shared border lines of the patch
//		bool good_patch_marker_cone = 1;
//		Eigen::MatrixXf temp_patch_data = MainConicalPatch(cloud_p, coefficients->values, &good_patch_marker_cone);
//		// MainConicalPatch(cloud_p, coefficients->values, &good_patch_marker_cone, patch_count);
//		if (good_patch_marker_cone == 0)
//		{
//			std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
//			std::cerr << "Input 0 if you want to change the tolerence and try cone recognition again." << std::endl;
//			std::cerr << "Input 1 if you want to try other recognition methods." << std::endl;
//			int check_again = 1;
//			std::cin >> check_again;
//			cin.clear();
//			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//			if (check_again == 0)
//			{
//				continue;
//			}
//			else
//			{
//				break;
//			}
//		}
//		(*patch_data)[*patch_count] = temp_patch_data;
//		//patch_data[patch_count] = MainConicalPatch(cloud_p, coefficients->values);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
//		*sourceCloud = *cloud_p;
//		(*sourceClouds)[*patch_count] = sourceCloud;
//		(*patch_count)++;
//
//
//		// Update the cloud of points still to fit in a patch
//		extract.setNegative(true);
//		extract.filter(*cloud_f);
//		(*cloud).swap(cloud_f);
//		extract_normals.setNegative(true);
//		extract_normals.setInputCloud(*cloud_normals);
//		extract_normals.setIndices(inliers);
//		extract_normals.filter(*cloud_normals_f);
//		(*cloud_normals).swap(cloud_normals_f);
//
//		// Project inliers to the recognized parametric 
//		pcl::ProjectInliers<pcl::PointXYZ> proj;
//		proj.setModelType(pcl::SACMODEL_CONE);
//		proj.setInputCloud(cloud_p);
//		proj.setModelCoefficients(coefficients);
//		proj.filter(*cloud_projected);
//		(*cloud_p).swap(*cloud_projected);
//
//		// Check patch validity:
//		/*check_patch_validity()
//		{
//
//		}*/
//
//		// Print the coefficients of the found cylinder model:	
//		std::cerr << std::endl;
//		std::cerr << "coef cone: " << std::endl;
//		std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
//		std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
//		std::cerr << "semi-angle:   " << coefficients->values[6] << std::endl;
//		std::cerr << std::endl;
//
//		//outputCloudOnExcel(coefficients, load_file, "cone");
//
//
//
//		// INFORMATION TO SAVE SOMEWHERE
//
//
//		std::cerr << "------ Remaining points: " << (*cloud)->size() << " data points." << std::endl << std::endl;
//		//std::cerr << "------ Remaining normals: " << cloud_normals->size() << " data points." << std::endl;
//
//	}
//}


int SinglePatchPartition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, int threshold_inliers,
	                     int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds, Models_recognition_results *results_single_patch_recognition)
{
	std::cerr << std::endl << "Single patch recognition:" << std::endl;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients()), coefficients_cyl(new pcl::ModelCoefficients()), coefficients_cone(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices()), inliers_cyl(new pcl::PointIndices()), inliers_cone(new pcl::PointIndices());
	int cloud_size = (*cloud)->points.size();
	int num_plane, num_cyl, num_cone, num_max;


	(*results_single_patch_recognition).model_with_maximum_points = "plane";
	num_plane = PlaneRecognition(*cloud, threshold_inliers, &inliers_plane, &coefficients_plane);
	(*results_single_patch_recognition).coefficients_plane = coefficients_plane;
	(*results_single_patch_recognition).inliers_plane = inliers_plane;
	num_max = num_plane;
	if (num_max != cloud_size)
	{
		num_cyl = CylinderRecognition(*cloud, *cloud_normals, threshold_inliers, &inliers_cyl, &coefficients_cyl);
		(*results_single_patch_recognition).coefficients_cylinder = coefficients_cyl;
		(*results_single_patch_recognition).inliers_cylinder = inliers_cyl;
		if (num_max < num_cyl)
		{
			(*results_single_patch_recognition).model_with_maximum_points = "cylinder";
			num_max = num_cyl;
		}

		if (num_max != cloud_size)
		{
			num_cone = ConeRecognition(*cloud, *cloud_normals, threshold_inliers, &inliers_cone, &coefficients_cone);
			(*results_single_patch_recognition).coefficients_cone = coefficients_cone;
			(*results_single_patch_recognition).inliers_cone = inliers_cone;
			if (num_max < num_cone)
			{
				(*results_single_patch_recognition).model_with_maximum_points = "cone";
				num_max = num_cone;
			}

			if (num_max != cloud_size)
			{
				std::cerr << "The whole part can not be recognized as one single patch." << std::endl;
				return 0;
			}
			else
			{
				std::cerr << "The whole part can be recognized as a cone." << std::endl;
				return FindConeBorder(cloud, cloud_normals, (*results_single_patch_recognition).inliers_cone, (*results_single_patch_recognition).coefficients_cone, patch_count, patch_data, sourceClouds);
			}
		}
		else
		{
			std::cerr << "The whole part can be recognized as a cylinder." << std::endl;
			return FindCylinderBorder(cloud, cloud_normals, (*results_single_patch_recognition).inliers_cylinder, (*results_single_patch_recognition).coefficients_cylinder, patch_count, patch_data, sourceClouds);
		}
	}
	else
	{
		std::cerr << "The whole part can be recognized as a plane." << std::endl;
		return FindPlaneBorder(cloud, cloud_normals, (*results_single_patch_recognition).inliers_plane, (*results_single_patch_recognition).coefficients_plane, patch_count, patch_data, sourceClouds);
	}

	//model_with_maximum_points[0] = 1; //plane
	//model_with_maximum_points[1] = PlaneRecognition(*cloud, threshold_inliers, &inliers, &coefficients);

	//if (model_with_maximum_points[model_with_maximum_points[0]] != cloud_size)
	//{	
	//	model_with_maximum_points[2] = CylinderRecognition(*cloud, *cloud_normals, threshold_inliers, &inliers, &coefficients);
	//	if (model_with_maximum_points[model_with_maximum_points[0]] < model_with_maximum_points[2])
	//	{
	//		model_with_maximum_points[0] = 2; //cylinder
	//	}	

	//	if (model_with_maximum_points[model_with_maximum_points[0]] != cloud_size)
	//	{
	//		model_with_maximum_points[3] = ConeRecognition(*cloud, *cloud_normals, threshold_inliers, &inliers, &coefficients);
	//		if (model_with_maximum_points[model_with_maximum_points[0]] < model_with_maximum_points[3])
	//		{
	//			model_with_maximum_points[0] = 3;  //cone
	//		}
	//
	//		if (model_with_maximum_points[model_with_maximum_points[0]] != cloud_size)
	//		{
	//			std::cerr << "The whole part can not be recognized as one single patch." << std::endl;
	//			return 0;
	//		}
	//		else
	//		{
	//			std::cerr << "The whole part can be recognized as a cone." << std::endl;
	//			return FindConeBorder(cloud, cloud_normals, inliers, coefficients, patch_count, patch_data, sourceClouds);
	//		}
	//	}
	//	else
	//	{
	//		std::cerr << "The whole part can be recognized as a cylinder." << std::endl;
	//		return FindCylinderBorder(cloud, cloud_normals, inliers, coefficients, patch_count, patch_data, sourceClouds);
	//	}
	//}
	//else
	//{
	//	std::cerr << "The whole part can be recognized as a plane." << std::endl;
	//	return FindPlaneBorder(cloud, cloud_normals, inliers, coefficients, patch_count, patch_data, sourceClouds);
	//}

}


int TwoPatchesPatition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, int threshold_inliers, int *patch_count, 
	                   Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds, Models_recognition_results results_single_patch_recognition)
{
	std::cerr << std::endl << "Two patches recognition:" << std::endl;
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	//pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients()), coefficients_cylinder(new pcl::ModelCoefficients()), coefficients_cone(new pcl::ModelCoefficients());
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	//pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices()), inliers_cylinder(new pcl::PointIndices()), inliers_cone(new pcl::PointIndices());
	/*std::string flag;
	PlaneRecognition(*cloud, threshold_inliers, &inliers_plane, &coefficients_plane);
	CylinderRecognition(*cloud, *cloud_normals, threshold_inliers, &inliers_cylinder, &coefficients_cylinder);
	ConeRecognition(*cloud, *cloud_normals, threshold_inliers, &inliers_cone, &coefficients_cone);*/

	if (results_single_patch_recognition.model_with_maximum_points == "plane")
	{
		FindPlaneBorder(cloud, cloud_normals, results_single_patch_recognition.inliers_plane, results_single_patch_recognition.coefficients_plane, patch_count, patch_data, sourceClouds);
	}
	else
	{
		if (results_single_patch_recognition.model_with_maximum_points == "cylinder")
		{
			FindCylinderBorder(cloud, cloud_normals, results_single_patch_recognition.inliers_cylinder, results_single_patch_recognition.coefficients_cylinder, patch_count, patch_data, sourceClouds);
		}
		else
		{
			FindConeBorder(cloud, cloud_normals, results_single_patch_recognition.inliers_cone, results_single_patch_recognition.coefficients_cone, patch_count, patch_data, sourceClouds);
		}
	}

	return 0;
}