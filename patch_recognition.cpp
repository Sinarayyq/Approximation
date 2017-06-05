#include "patch_recognition.h"




void PlaneRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{
	// while at least 'threshold_inliers' points from the original cloud are still not fitted (e.g. 30%)
	while ((*cloud_filtered)->points.size() > threshold_inliers)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f(new pcl::PointCloud<pcl::Normal>);
		
		// Define 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt");
		

		// create the segmentation object for plane
		pcl::SACSegmentation<pcl::PointXYZ> seg;

		// set all the parameters for the segmentation object for plane
		setSegmentationParametersForPlane(seg);
		std::cerr << std::endl << "Plane recognition:" << std::endl;

		// segment the largest planar component from the remaining cloud
		seg.setInputCloud(*cloud_filtered);
		seg.segment(*inliers, *coefficients);
		int num_inliers = inliers->indices.size();

		if (num_inliers == 0)
		{
			std::cerr << std::endl;
			std::cerr << "no plane found." << num_inliers << std::endl << std::endl;
			break;
		}

		std::cerr << std::endl;
		std::cerr << "found a plane! number of inliers = " << num_inliers << std::endl << std::endl;
		if (num_inliers < threshold_inliers)
		{
			std::cerr << num_inliers << " < " << threshold_inliers << ". min num of inliers not reached ----->>> patch discarded" << std::endl << std::endl << std::endl;
			break;
		}

		// extract the inliers
		extract.setInputCloud(*cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		//std::cerr << "pointcloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		// visualize the fitted points
		//visualizePointCloud(cloud_p,"found planar patch");
		visualizePointCloud(*cloud_filtered, cloud_p, "found planar patch", xy);

		std::cerr << "Input 0 if you think the panel is good enough." << std::endl;
		std::cerr << "Input 1 if you want to change the tolerence and try plane recognition again." << std::endl;
		std::cerr << "Input 2 if you want to try other recognition methods." << std::endl;
		
		int check_again;
		std::cin >> check_again;
		cin.clear();
		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		if (check_again == 2)
		{
			break;
		}
		else
		{
			if (check_again == 1)
			{
				continue;
			}
		}



		bool good_patch_marker_plane = 1;
		Eigen::MatrixXf temp_patch_data = MainPlanarPatch(cloud_p, coefficients->values, &good_patch_marker_plane, patch_count);
		//MainPlanarPatch(cloud_p, coefficients->values, &good_patch_marker_plane, patch_count);
		if (good_patch_marker_plane == 0)
		{
			std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
			std::cerr << "Input 0 if you want to change the tolerence and try plane recognition again." << std::endl;
			std::cerr << "Input 1 if you want to try other recognition methods." << std::endl;
			int check_again = 1;
			std::cin >> check_again;
			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			if (check_again == 0)
			{
				continue;
			}
			else
			{
				break;
			}
			//continue;
		}
		(*patch_data)[*patch_count] = temp_patch_data;
		pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
		*sourceCloud = *cloud_p;
		(*sourceClouds)[*patch_count] = sourceCloud;
		(*patch_count)++;



		// update the cloud of points still to fit in a patch
		extract.setNegative(true);
		extract.filter(*cloud_f);
		(*cloud_filtered).swap(cloud_f);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(*cloud_normals);
		extract_normals.setIndices(inliers);
		extract_normals.filter(*cloud_normals_f);
		(*cloud_normals).swap(cloud_normals_f);

		//Project the inliers on the RANSAC parametric model.
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud_p);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		(*cloud_p).swap(*cloud_projected);

		
		//// check validity of the patch (discard strips and check if there are groups of fitted points to be separated)
		//if(checkpatchvalidity("plane", cloud_p, cloud_filtered))
		//{

		//}

		// print the coefficients of the found plane model:
		std::cerr << std::endl << "coef plane: " << std::endl;
		std::cerr << "Ax + By + Cz + D = 0" << std::endl;
		std::cerr << "A: " << coefficients->values[0] << std::endl;
		std::cerr << "B: " << coefficients->values[1] << std::endl;
		std::cerr << "C: " << coefficients->values[2] << std::endl;
		std::cerr << "D: " << coefficients->values[3] << std::endl;
		//cout << "load_file:" << load_file << endl;
		//outputCloudOnExcel(coefficients, load_file,"plane");


		std::cerr << std::endl;
		// information to save somewhere



		//pcl::pointcloud<pcl::PointXYZ>::ptr transformed_planar_patch = transformplanarpatchpoints(cloud_p, coefficients->values);
		//getminarearectofplanarpatch(cloud_p, coefficients->values);

		// make a pcd file containing the fitted points
		//std::stringstream ss;
		//ss << "c:\\extract_indices\\build\\debug\\output\\resulting_plane_" << i << ".pcd";
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

		std::cerr << "------ remaining points: " << (*cloud_filtered)->size() << " data points." << std::endl << std::endl;

	}
}


void CylinderRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{
	while ((*cloud_filtered)->points.size() > threshold_inliers) 
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f(new pcl::PointCloud<pcl::Normal>);
		
		// Define 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt");
		
		// create the segmentation object
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cyl;
		setSegmentationParametersForCylinder(seg_cyl);
		std::cerr << std::endl << "Cylinder recognition:" << std::endl;

		// segment the largest cylindrical component from the remaining cloud
		seg_cyl.setInputCloud(*cloud_filtered);
		seg_cyl.setInputNormals(*cloud_normals);
		seg_cyl.segment(*inliers, *coefficients);
		int num_inliers = inliers->indices.size();

		if (num_inliers == 0)
		{
			std::cerr << std::endl;
			std::cerr << "no cylinder found." << num_inliers << std::endl;
			break;
		}

		std::cerr << std::endl;
		std::cerr << "found a cylinder! number of inliers = " << num_inliers << std::endl;
		if (num_inliers < threshold_inliers)
		{
			std::cerr << num_inliers << " < " << threshold_inliers << ". min num of inliers not reached ----->>> patch discarded" << std::endl << std::endl << std::endl;
			break;
		}

		// extract the inliers
		extract.setInputCloud(*cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		

		visualizePointCloud(*cloud_filtered, cloud_p, "found cylindrical patch", xy);

		std::cerr << "Input 0 if you think the panel is good enough." << std::endl;
		std::cerr << "Input 1 if you want to change the tolerence and try cylinder recognition again." << std::endl;
		std::cerr << "Input 2 if you want to try other recognition methods." << std::endl;

		int check_again;
		std::cin >> check_again;
		cin.clear();
		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		if (check_again == 2)
		{
			break;
		}
		else
		{
			if (check_again == 1)
			{
				continue;
			}
		}



		// determine the possible shared border lines of the patch
		bool good_patch_marker_cylinder = 1;
		Eigen::MatrixXf temp_patch_data = MainCylindricalPatch(cloud_p, coefficients->values, &good_patch_marker_cylinder, patch_count);
		
		if (good_patch_marker_cylinder == 0)
		{
			std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
			std::cerr << "Input 0 if you want to change the tolerence and try cylinder recognition again." << std::endl;
			std::cerr << "Input 1 if you want to try other recognition methods." << std::endl;
			int check_again = 1;
			std::cin >> check_again;
			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			if (check_again == 0)
			{
				continue;
			}
			else
			{
				break;
			}
			//continue;
		}
		(*patch_data)[*patch_count] = temp_patch_data;
		pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
		*sourceCloud = *cloud_p;
		(*sourceClouds)[*patch_count] = sourceCloud;
		(*patch_count)++;

		// update the cloud of points still to fit in a patch
		extract.setNegative(true);
		extract.filter(*cloud_f);
		(*cloud_filtered).swap(cloud_f);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(*cloud_normals);
		extract_normals.setIndices(inliers);
		extract_normals.filter(*cloud_normals_f);
		(*cloud_normals).swap(cloud_normals_f);

		//Project the inliers on the RANSAC parametric model.
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType(pcl::SACMODEL_CYLINDER);
		proj.setInputCloud(cloud_p);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		(*cloud_p).swap(*cloud_projected);
	
		// check validity of the patch (discard strips and check if there are groups of fitted points to be separated)
		/*if(checkpatchvalidity("cylinder", cloud_p, cloud_filtered))
		{

		}*/


		// print the coefficients of the found cylinder model:
		std::cerr << std::endl;
		std::cerr << "coef cylinder: " << std::endl;
		std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
		std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
		std::cerr << "radius:       " << coefficients->values[6] << std::endl << std::endl;
		std::cerr << std::endl;


		std::cerr << "------ remaining points: " << (*cloud_filtered)->size() << " data points." << std::endl << std::endl;
		//std::cerr << "------ remaining normals: " << all_cloud_normals->size() << " data points." << std::endl;

	}
}


void ConeRecognition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
	int threshold_inliers, int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{
	while ((*cloud_filtered)->points.size() > threshold_inliers)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f(new pcl::PointCloud<pcl::Normal>);
		
		// Define 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
		
		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt");
		// Create the segmentation object for cone
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
		// Set all the parameters for cone segmentation object
		setSegmentationParametersForCone(seg_cone);
		std::cerr << std::endl << "Cone recognition:" << std::endl;

		// Segment the largest conical component from the remaining cloud
		seg_cone.setInputCloud(*cloud_filtered);
		seg_cone.setInputNormals(*cloud_normals);
		seg_cone.segment(*inliers, *coefficients);
		int num_inliers = inliers->indices.size();

		if (num_inliers == 0)
		{
			std::cerr << std::endl;
			std::cerr << "NO CONE FOUND." << num_inliers << std::endl;
			break;
		}

		std::cerr << std::endl;
		std::cerr << "FOUND A CONE! Number of inliers = " << num_inliers << std::endl;
		if (num_inliers < threshold_inliers)
		{
			std::cerr << num_inliers << " < " << threshold_inliers << ". Min num of inliers not reached ----->>> PATCH DISCARDED" << std::endl << std::endl << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(*cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);

		// Visualize the fitted points
		//visualizePointCloud(cloud_p,"found conical patch");
		visualizePointCloud(*cloud_filtered, cloud_p, "found conical patch", xy);
		
		std::cerr << "Input 0 if you think the panel is good enough." << std::endl;
		std::cerr << "Input 1 if you want to change the tolerence and try cone recognition again." << std::endl;
		std::cerr << "Input 2 if you want to try other recognition methods." << std::endl;

		int check_again;
		std::cin >> check_again;
		cin.clear();
		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		if (check_again == 2)
		{
			break;
		}
		else
		{
			if (check_again == 1)
			{
				continue;
			}
		}


		// Determine the possible shared border lines of the patch
		bool good_patch_marker_cone = 1;
		Eigen::MatrixXf temp_patch_data = MainConicalPatch(cloud_p, coefficients->values, &good_patch_marker_cone, patch_count);
		// MainConicalPatch(cloud_p, coefficients->values, &good_patch_marker_cone, patch_count);
		if (good_patch_marker_cone == 0)
		{
			std::cerr << "~~~(>_<)~~~ The patch is not good enough to fabricate so discarded ~~~(>_<)~~~" << std::endl << std::endl;
			std::cerr << "Input 0 if you want to change the tolerence and try cone recognition again." << std::endl;
			std::cerr << "Input 1 if you want to try other recognition methods." << std::endl;
			int check_again = 1;
			std::cin >> check_again;
			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			if (check_again == 0)
			{
				continue;
			}
			else
			{
				break;
			}
		}
		(*patch_data)[*patch_count] = temp_patch_data;
		//patch_data[patch_count] = MainConicalPatch(cloud_p, coefficients->values);
		pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
		*sourceCloud = *cloud_p;
		(*sourceClouds)[*patch_count] = sourceCloud;
		(*patch_count)++;


		// Update the cloud of points still to fit in a patch
		extract.setNegative(true);
		extract.filter(*cloud_f);
		(*cloud_filtered).swap(cloud_f);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(*cloud_normals);
		extract_normals.setIndices(inliers);
		extract_normals.filter(*cloud_normals_f);
		(*cloud_normals).swap(cloud_normals_f);

		// Project inliers to the recognized parametric 
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType(pcl::SACMODEL_CONE);
		proj.setInputCloud(cloud_p);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		(*cloud_p).swap(*cloud_projected);

		// Check patch validity:
		/*check_patch_validity()
		{

		}*/

		// Print the coefficients of the found cylinder model:	
		std::cerr << std::endl;
		std::cerr << "coef cone: " << std::endl;
		std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
		std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
		std::cerr << "semi-angle:   " << coefficients->values[6] << std::endl;
		std::cerr << std::endl;

		//outputCloudOnExcel(coefficients, load_file, "cone");



		// INFORMATION TO SAVE SOMEWHERE


		std::cerr << "------ Remaining points: " << (*cloud_filtered)->size() << " data points." << std::endl << std::endl;
		//std::cerr << "------ Remaining normals: " << cloud_normals->size() << " data points." << std::endl;

	}
}

std::string exePath()
{
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	std::string::size_type pos = std::string(buffer).find_last_of("\\/");
	return std::string(buffer).substr(0, pos);
}