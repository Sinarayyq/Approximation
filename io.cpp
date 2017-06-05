
#include "io.h"

std::string PATH_HEAD = exe_Path() + "\\..\\..";

std::string exe_Path()
{
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	std::string::size_type pos = std::string(buffer).find_last_of("\\/");
	return std::string(buffer).substr(0, pos);
}

bool fillClouds(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals1)
{
	//// Fill in the cloud data (PREVIOUS VERSION WITHOUT NORMALS)
	//pcl::PCDReader reader;
	//reader.read (PATH_PCD_DOC, *cloud_blob);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path, *cloud) == -1) //* load the point coordinates from the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return false;
	}
	
	if (pcl::io::loadPCDFile<pcl::Normal> (pcd_path, *cloud_normals1) == -1) //* load the normal coordinates of the points from the file
	{
		PCL_ERROR ("Couldn't read the pcd file \n");
		return false;
	}

	return true;
}

void outputCloudOnExcel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud)
{
	//std::ifstream indata;
	std::ofstream outdata;
	std::string name_file = PATH_HEAD + "\\temp\\" + name_cloud + ".csv";
	outdata.open(name_file, ios::app);

	const int num_points = cloud->points.size();
	for (std::size_t i = 0; i < num_points; ++i)
	{
		outdata << cloud->at(i).x << "," << cloud->at(i).y << "," << cloud->at(i).z << endl;
	}
}

void outputCloudOnPTS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud)
{
	std::ofstream outdata;
	std::string name_file = PATH_HEAD + "\\temp\\" + name_cloud + ".pts";
	outdata.open(name_file, ios::app);

	const int num_points = cloud->points.size();
	for (std::size_t i = 0; i < num_points; ++i)
	{
		outdata << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << endl;
	}
}

void outputExcel(pcl::ModelCoefficients::Ptr coefficients, std::string name_cloud, std::string type)
{
	//ifstream indata;
	std::ofstream outdata;
	name_cloud = name_cloud.substr(0, name_cloud.size() - 4);
	//cout << "name_cloud:" << name_cloud <<endl;
	std::string name_file = name_cloud + ".csv";
	//cout << "name_file:" << name_file << endl;

	outdata.open(name_file, ios::app);
	if (type == "plane")
	{
		outdata << coefficients->values[0] << "," << coefficients->values[1] << "," << coefficients->values[2] << endl;
		outdata << coefficients->values[3] << endl << endl;
	}
	else
	{
		outdata << coefficients->values[0] << "," << coefficients->values[1] << "," << coefficients->values[2] << endl;
		outdata << coefficients->values[3] << "," << coefficients->values[4] << "," << coefficients->values[5] << endl;
		outdata << coefficients->values[6] << endl << endl;
	}
	
}


std::string outputCloudOnTXT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud, int count)
{
	std::ofstream outdata;
	std::stringstream ss;
	std::string str;
	ss << count;
	ss >> str;
	std::string name_file = PATH_HEAD + "\\temp\\" + str + "_"+ name_cloud + ".txt";

	outdata.open(name_file, ios::out);
	outdata.clear();

	const int num_points = cloud->points.size();
	outdata << num_points << endl;
	for (std::size_t i = 0; i < num_points; ++i)
	{
		//outdata << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << endl;
		outdata << cloud->at(i).x << " " << cloud->at(i).z << " " << endl;
	}
	return name_file;
}

std::string exportCloudAsPTS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud)
{
	std::ofstream outdata;
	std::stringstream ss;
	std::string name_file = exe_Path() + "\\..\\..\\temp\\" + name_cloud + ".pts";

	outdata.open(name_file, ios::out);
	outdata.clear();

	const int num_points = cloud->points.size();
	outdata << num_points << endl;
	for (std::size_t i = 0; i < num_points; ++i)
	{
		outdata << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << endl;
	}
	return name_file;
}



