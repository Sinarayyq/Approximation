#include "cloud_visualizer.h"



// -----Visualize Point Cloud-----
boost::shared_ptr<pcl::visualization::PCLVisualizer> visCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string window_label, camera_position camera_pos)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (window_label));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud_to_visualize, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addCoordinateSystem(500.0, "global");
	
	// Set camera position according to different input.
	switch (camera_pos)
	{
	case xy:
		viewer->setCameraPosition(
			0, 0, 1,//double pos_x, double pos_y, double pos_z,
			0, 0, -1,//double view_x, double view_y, double view_z,
			1, 0, 0);//double up_x, double up_y, double up_z, int viewport = 0
	case yz:		
		viewer->setCameraPosition(
			1, 0, 0,//double pos_x, double pos_y, double pos_z,                                    
			-1, 0, 0,//double view_x, double view_y, double view_z,
			0, 1, 0);//double up_x, double up_y, double up_z, int viewport = 0
	case xz:
		viewer->setCameraPosition(
			0, -1, 0,//double pos_x, double pos_y, double pos_z,                                    
			0, 1, 0,//double view_x, double view_y, double view_z,
			0, 0, 1);//double up_x, double up_y, double up_z, int viewport = 0
	}
	
	return (viewer);
}

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string viewer_window_label, camera_position camera_pos)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	// Make use of the 
	viewer = visCloud(cloud, viewer_window_label, camera_pos);
	
	// Reset camera according to the input data. Zoom out so that all data points can be viewed.
	viewer->resetCamera();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	viewer->close();

}

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string label_viewer_window, camera_position camera_pos)
{
	// Window setup
	pcl::visualization::PCLVisualizer viewer(label_viewer_window);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(500.0);
	viewer.setCameraPosition(0, 0, 1,//double pos_x, double pos_y, double pos_z,                                    
							 0, 0, 0,//double view_x, double view_y, double view_z,
							 0, 1, 0);//double up_x, double up_y, double up_z, int viewport = 0);

	// Add the first cloud: size 2, white(default)
	viewer.addPointCloud(cloud1, "cloud1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
	// Define color "red"
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);
	// Add the second cloud: size 3, red (defined)
	viewer.addPointCloud<pcl::PointXYZ>(cloud2, red, "cloud2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

	//Auto recenters the view.
	viewer.resetCamera();

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	viewer.close();
}

// -----Visualize Chains-----
void visualizeShapes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_indexes, std::vector<int> chain)
{
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer = shapesVis(cloud_to_visualize,label_viewer_window,hull_indexes,chain);

	//viewer->addCoordinateSystem (1000);
	viewer->resetCamera();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer->close();
	
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	std::string label_viewer_window,
	std::vector<int> hull_index,
	std::vector<int> chain)
{
	// -----Open 3D viewer and add point cloud-----
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(label_viewer_window));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addCoordinateSystem(500.0, "global");
	viewer->setCameraPosition(0, -1, 0,//double pos_x, double pos_y, double pos_z,                                    
							  0, 0, 0,//double view_x, double view_y, double view_z,
							  0, 0, 1);//double up_x, double up_y, double up_z, int viewport = 0););

	// -----Add shapes at cloud points-----

	int j = 0;
	for (j = 0; j < hull_index.size() - 1; j++)
	{
		viewer->addLine<pcl::PointXYZ>(cloud->points[hull_index[j]], cloud->points[hull_index[j + 1]], std::to_string(static_cast<long long>(j)));
	}


	std::vector<int> colors;
	for (int z = 0; z < chain.size() / 2; z++)
	{
		colors.push_back(255);
		colors.push_back(0);
		colors.push_back(0);
		colors.push_back(255);
	}
	for (int i = 0; i < chain.size(); i += 2)
	{
		viewer->addLine<pcl::PointXYZ>(cloud->points[chain[i]], cloud->points[chain[i + 1]], colors[i + 2], colors[i + 3], 0, std::to_string(static_cast<long long>(i + j + 1)));
	}
	return (viewer);
}

// -----Visualize Convex Hull-----
void visualizeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_indexes)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer = visConvex(cloud_to_visualize, label_viewer_window, hull_indexes);
	viewer->resetCamera();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> visConvex(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string label_viewer_window, std::vector<int> hull_index)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(label_viewer_window));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(500.0, "global");
	viewer->setCameraPosition(0, -1, 0,//double pos_x, double pos_y, double pos_z,                                    
		0, 0, 0,//double view_x, double view_y, double view_z,
		0, 0, 1);//double up_x, double up_y, double up_z, int viewport = 0););


				 //------------------------------------
				 //-----Add shapes at cloud points-----
				 //------------------------------------

	int j = 0;
	for (j = 0; j < hull_index.size() - 1; j++)
	{
		viewer->addLine<pcl::PointXYZ>(cloud->points[hull_index[j]], cloud->points[hull_index[j + 1]], std::to_string(static_cast<long long>(j)));
	}
	viewer->addLine<pcl::PointXYZ>(cloud->points[hull_index[j]], cloud->points[hull_index[0]], std::to_string(static_cast<long long>(j)));

	viewer->resetCamera();
	return (viewer);
}




