#include "cloud_visualizer.h"
#include <pcl/io/pcd_io.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <pcl/point_types.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	                                                           std::string label_viewer_window,
															   std::vector<int> hull_index, 
															   std::vector<int> chain)
{
	// --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (label_viewer_window));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (500.0, "global");
	viewer->setCameraPosition(0 , -1 , 0,//double pos_x, double pos_y, double pos_z,                                    
		                      0 , 0 , 0,//double view_x, double view_y, double view_z,
						      0 , 0 , 1);//double up_x, double up_y, double up_z, int viewport = 0););


  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------

	int j = 0;
	for (j = 0 ; j < hull_index.size() - 1 ; j++)
	{
		viewer->addLine<pcl::PointXYZ> (cloud->points[hull_index[j]],cloud->points[hull_index[j+1]], std::to_string(static_cast<long long>(j)));
	}
	//viewer->addLine<pcl::PointXYZ> (cloud->points[hull_index[j]],cloud->points[hull_index[0]], std::to_string(static_cast<long long>(j)));
	
	std::vector<int> colors;
	for (int z = 0 ;z < chain.size()/2 ; z++)
	{
		colors.push_back(255);
		colors.push_back(0);
		colors.push_back(0);
		colors.push_back(255);
	}
	for ( int i= 0; i < chain.size(); i+=2)
	{
		viewer->addLine<pcl::PointXYZ> (cloud->points[chain[i]],cloud->points[chain[i+1]],colors[i+2],colors[i+3],0, std::to_string(static_cast<long long>(i+j+1)));
	}
	return (viewer);
}

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string label_viewer_window)
{
	pcl::visualization::PCLVisualizer viewer (label_viewer_window);
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud,label_viewer_window);
	viewer.addCoordinateSystem (500.0);
	viewer.setCameraPosition( 0 , 0 , 1,//double pos_x, double pos_y, double pos_z,                                    
		                      0 , 0 , 0,//double view_x, double view_y, double view_z,
						      0 , 1 , 0);//double up_x, double up_y, double up_z, int viewport = 0);
	//viewer.updateCamera();

	while (!viewer.wasStopped ())
    {
	    viewer.spinOnce ();
    }
	viewer.close();
	
}

void visualizeTwoPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string label_viewer_window)
{
	
	pcl::visualization::PCLVisualizer viewer(label_viewer_window); 
    viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (500.0);
	viewer.setCameraPosition( 0 , 0 , 1,//double pos_x, double pos_y, double pos_z,                                    
		                      0 , 0 , 0,//double view_x, double view_y, double view_z,
						      0 , 1 , 0);//double up_x, double up_y, double up_z, int viewport = 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud2, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud2, red, "cloud2");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
	//viewer.addPointCloud(vertices, "vertices"); 

	viewer.addPointCloud(cloud1, "cloud1"); 

    while (!viewer.wasStopped()) 
    { 
		viewer.spinOnce(); 
    }
	viewer.close();
}

void visualizeShapes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_indexes, std::vector<int> chain)
{
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer = shapesVis(cloud_to_visualize,label_viewer_window,hull_indexes,chain);

	//viewer->addCoordinateSystem (1000);
	 while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer->close();
	
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesconvex(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string label_viewer_window,std::vector<int> hull_index)
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

	return (viewer);
}

void visualizeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string label_viewer_window, std::vector<int> hull_indexes)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer = shapesconvex(cloud_to_visualize, label_viewer_window, hull_indexes);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();

}



int visualizeCGAL(int argc, char **argv)
{
	argc = 1;
	//argv = (char **)malloc(sizeof(char**));
	*argv = "C:\\Alpha_shapes_2\\build\\Debug\\Alpha_shapes_2.exe";
	QApplication app(argc, argv);

	QGraphicsScene scene;
	scene.setSceneRect(0, 0, 100, 100);
	scene.addRect(QRectF(0, 0, 100, 100));
	scene.addLine(QLineF(0, 0, 100, 100));
	scene.addLine(QLineF(0, 100, 100, 0));

	QGraphicsView* view = new QGraphicsView(&scene);
	CGAL::Qt::GraphicsViewNavigation navigation;
	view->installEventFilter(&navigation);
	view->viewport()->installEventFilter(&navigation);
	view->setRenderHint(QPainter::Antialiasing);

	view->show();
	return app.exec();
}



