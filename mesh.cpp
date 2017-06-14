/*Purpose:
obtain the coordinates of vertices and create a new point cloud from them;
generate random points in each trangle, the number of points being determined according to the area of each triangle and the preset density;
visualize the two sets of point cloud (vertices and random_points) in the same window with different colors.
*/
#include <fstream>
#include <string>
#include <vector>
#include <windows.h>
#include <math.h>
#include <iostream> 
#include "mesh.h"
#include "read_parameters.h"
#include "cloud_visualizer.h"
//#define NOMINMAX
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "io.h"
#include "read_parameters.h"

using namespace std;





//
//double uniform_deviate (int seed)
//{
//	double ran = seed * (1.0 / (RAND_MAX + 1.0));
//	return ran;
//}
//
////To generate ONE random point in ONE triangl; a/b/c represents the three vertices of selected triangle and 1/2/3 represents x/y/z coordinates respectively.
//void RandomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, 
//	                     std::vector<float> *random_point_x, std::vector<float> *random_point_y, std::vector<float> *random_point_z)
//{
//	float r1 = static_cast<float> (uniform_deviate (rand ()));
//    float r2 = static_cast<float> (uniform_deviate (rand ()));
//    float r1sqr = sqrtf (r1);
//    float OneMinR1Sqr = (1 - r1sqr);
//    float OneMinR2 = (1 - r2);
//    a1 *= OneMinR1Sqr;
//    a2 *= OneMinR1Sqr;
//    a3 *= OneMinR1Sqr;
//    b1 *= OneMinR2;
//    b2 *= OneMinR2;
//    b3 *= OneMinR2;
//    c1 = r1sqr * (r2 * c1 + b1) + a1;
//    c2 = r1sqr * (r2 * c2 + b2) + a2;
//    c3 = r1sqr * (r2 * c3 + b3) + a3;
//    (*random_point_x).push_back(c1);
//    (*random_point_y).push_back(c2);
//    (*random_point_z).push_back(c3);
//}
//
////Run iteration and generate random points in all triangles, the number of points determined by area and density
//void RandomPointGeneraterInTriangles(std::vector<float> x, std::vector<float> y, std::vector<float> z, 
//	                                 std::vector<float> *random_point_x, std::vector<float> *random_point_y, std::vector<float> *random_point_z, 
//	                                 std::vector<float> *random_point_normal_x, std::vector<float> *random_point_normal_y, std::vector<float> *random_point_normal_z,
//									 std::vector<Utils_sampling::Vec3> *verts, std::vector<Utils_sampling::Vec3> *nors, std::vector<int> *tris)
//{
//	//normal vector = (nx, ny, nz); x1/y1/z1 can be the coordinate of any vertex of the triangle
//	float Area;
//	double density = UNIT_AND_DENSITY_OF_SAMPLING_POINTS;
//	int number_of_random_points;
//	
//	//calculate the area and Surface equation 
//	for(int i = 0; i < x.size();i = i + 4)  
//    {  
//		//calculate the area and the number of random points in each triangle
//		//AB=(a1,b1,c1),AC=(a2,b2,c2)  
//		float a1,b1,c1,a2,b2,c2,a,b,c;
//		float normalx, normaly, normalz;
//		normalx = x[i];
//		normaly = y[i];
//		normalz = z[i];
//		for (int k = 0; k < 3; k++)
//		{
//			(*nors).push_back(Utils_sampling::Vec3(normalx, normaly, normalz));
//			(*verts).push_back(Utils_sampling::Vec3(x[i + 1 + k], y[i + 1 + k], z[i + 1 + k]));
//			(*tris).push_back((i/4)*3 + k);
//
//		}
//
//
//		a1 = x[i+2] - x[i+1];
//		b1 = y[i+2] - y[i+1];
//		c1 = z[i+2] - z[i+1];
//		a2 = x[i+3] - x[i+1];
//		b2 = y[i+3] - y[i+1];
//		c2 = z[i+3] - z[i+1];
//		a = b1 * c2 - c1 * b2;
//		b = c1 * a2 - a1 * c2;
//		c = a1 * b2 - b1 * a2;
//		Area = (sqrt( pow(a,2) + pow(b,2) + pow(c,2) ))/2;
//
//		//if(Area < 5000)
//		//{
//		//	number_of_random_points = 5;
//		//}
//		//else 
//		//{
//		number_of_random_points = density * Area;
//		if (number_of_random_points < 1)
//		{
//			number_of_random_points = 1;
//		}
//
//		//}
//		    /*cout<<"normalx = "<< normalx<< endl<< endl;
//			cout<<"normaly = "<< normaly<< endl<< endl;
//			cout<<"normalz = "<< normalz<< endl<< endl;
//		
//
//		    cout<<"random_point_normal_x = "<< random_point_normal_x[j] << endl<< endl;
//			cout<<"random_point_normal_y = "<< random_point_normal_y[j] << endl<< endl;
//			cout<<"random_point_normal_z = "<< random_point_normal_z[j] << endl<< endl;*/
//
//		//create random points and corresonding normals (just use the normal of the triangle)
//		for(int j = 0; j < number_of_random_points; j++) 
//		{
//			RandomPointTriangle(x[i + 1], y[i + 1], z[i + 1], x[i + 2], y[i + 2], z[i + 2], x[i + 3], y[i + 3], z[i + 3], random_point_x, random_point_y, random_point_z);
//						
//			(*random_point_normal_x).push_back(normalx);
//		    (*random_point_normal_y).push_back(normaly);
//		    (*random_point_normal_z).push_back(normalz);
//
//			/**/
//		}
//	}
//}
//
//// Create a new point cloud according to the 
//void CreatePointCloud(std::vector<float> coorx, std::vector<float> coory, std::vector<float> coorz, char *path)
//{
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//
//	//fill in the cloud data
//	cloud.width    = coorx.size() ;
//	cloud.height   = 1;
//	cloud.is_dense = false;
//    cloud.points.resize (cloud.width * cloud.height);
//	 
//
//
//    for (size_t i = 0; i < cloud.points.size (); ++i)
//    {
//		cloud.points[i].x = coorx[i];
//		cloud.points[i].y = coory[i];
//		cloud.points[i].z = coorz[i];
//    }
//
//	//create a new pcd file under an assigned path
//	ofstream fout( path );
//	if ( fout )// if successful
//	{ 
//		pcl::io::savePCDFileASCII (path, cloud);
//		fout.close(); // 
//	}
//		
//}
//
//void outputExcel(std::vector<Utils_sampling::Vec3> verts, std::vector<Utils_sampling::Vec3> nors)
//{
//	std::ifstream indata;
//	std::ofstream outdata;
//	std::string name_file = "C:\\Alpha_shapes_2\\poisson.csv";
//	outdata.open(name_file, ios::app);
//
//	const int num_points = verts.size();
//	for (std::size_t i = 0; i < num_points; ++i)
//	{
//		outdata << verts[i].x << "," << verts[i].y << "," << verts[i].z << "," << nors[i].x << "," << nors[i].y << "," << nors[i].z <<", " << "0" << std::endl;
//	}
//}
//
//void CreateNormalsOfAllPoints(std::vector<float> random_point_normal_x,std::vector<float> random_point_normal_y,std::vector<float> random_point_normal_z, char *path)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile(path, *cloud); 
//	//cout <<"cloud->points.size () = " << cloud->points.size () <<endl <<endl;
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
//	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	pcl::PointCloud<pcl::Normal>::Ptr random_point_normals(new pcl::PointCloud<pcl::Normal>); 
//	random_point_normals->resize(cloud->points.size());
//    //ne.setInputCloud (cloud);
//	//all_cloud_normals->points[0].normal_x = 0.0;
//   // ne.compute (*random_point_normals);
//	//cout <<"111random_point_normals->points.size () = " << random_point_normals->points.size () <<endl <<endl;
//	for ( int i = 0 ; i < random_point_normal_x.size (); i++)
//	{
//		//cout <<"random_point_normal_x[i] = " << random_point_normal_x[i] <<endl <<endl;
//		random_point_normals->points[i].normal_x = random_point_normal_x[i];
//		random_point_normals->points[i].normal_y = random_point_normal_y[i];
//		random_point_normals->points[i].normal_z = random_point_normal_z[i];
//		//cout <<"random_point_normals->points[i].normal_x = " << random_point_normals->points[i].normal_x <<endl <<endl;
//	}
//    //cout <<"random_point_normals->points.size () = " << random_point_normals->points.size () <<endl <<endl;
// 
//	pcl::copyPointCloud(*cloud, *cloud_with_normals); 
//	pcl::copyPointCloud(*random_point_normals, *cloud_with_normals); 
//
//	pcl::io::savePCDFileASCII(PATH_TEMPORARY_FILE+"cloud_with_normals.pcd", *cloud_with_normals);
//
//
//}

void CreatePointCloud(std::vector<Utils_sampling::Vec3> verts, std::vector<Utils_sampling::Vec3> nors, std::string path,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals)
{
	(*cloud_filtered)->width = verts.size();
	(*cloud_filtered)->height = 1;
	(*cloud_filtered)->is_dense = false;
	(*cloud_filtered)->points.resize((*cloud_filtered)->width * (*cloud_filtered)->height);
	for (size_t i = 0; i < (*cloud_filtered)->points.size(); ++i)
	{
		(*cloud_filtered)->points[i].x = verts[i].x;
		(*cloud_filtered)->points[i].y = verts[i].y;
		(*cloud_filtered)->points[i].z = verts[i].z;
	}


	(*cloud_normals)->resize((*cloud_filtered)->points.size());
	for (int i = 0; i < (*cloud_normals)->size(); i++)
	{
		(*cloud_normals)->points[i].normal_x = nors[i].x;
		(*cloud_normals)->points[i].normal_y = nors[i].y;
		(*cloud_normals)->points[i].normal_z = nors[i].z;
	}
}

void PrepareVertsAndNors(std::vector<float> coorX, std::vector<float> coorY, std::vector<float> coorZ,
	std::vector<Utils_sampling::Vec3> *verts, std::vector<Utils_sampling::Vec3> *nors, std::vector<int> *tris)
{
	int size = coorX.size();
	for (int i = 0; i < size; i = i + 4)
	{
		for (int k = 0; k < 3; k++)
		{
			(*nors).push_back(Utils_sampling::Vec3(coorX[i], coorY[i], coorZ[i]));
			(*verts).push_back(Utils_sampling::Vec3(coorX[i + 1 + k], coorY[i + 1 + k], coorZ[i + 1 + k]));
			(*tris).push_back((i / 4) * 3 + k);
		}
	}
}

bool ReadASCII(const char *cfilename, std::vector<float> *coorX, std::vector<float> *coorY, std::vector<float> *coorZ)
{


	int i = 0, j = 0, cnt = 0, pCnt = 4;
	char a[100];
	char str[100];
	double x = 0, y = 0, z = 0;

	std::ifstream in(cfilename, std::ifstream::in);

	if (!in)
	{
		return false;
	}


	do
	{
		i = 0;
		cnt = 0;
		in.getline(a, 100, '\n');

		while (a[i] != '\0')
		{
			//cout << "1 a[" << i << "]=" <<a[i] <<endl<<endl;
			if (!islower((int)a[i]) && !isupper((int)a[i]) && a[i] != ' ')
			{
				break;
			}

			cnt++;
			i++;
		}

		while (a[cnt] != '\0')
		{
			//cout << "3 a[" << cnt << "]=" <<a[cnt] <<endl<<endl;
			str[j] = a[cnt];
			cnt++;
			j++;
		}

		str[j] = '\0';
		j = 0;
		//cout << "str = " << str[0] <<endl<<endl;
		if (sscanf(str, "%lf%lf%lf", &x, &y, &z) == 3)
		{
			(*coorX).push_back(x);
			(*coorY).push_back(y);
			(*coorZ).push_back(z);
		}
		pCnt++;
		//cout << "-------------line end-----------------"<<endl <<endl; 
	} while (!in.eof());


	//cout << "******  ACSII FILES　******" << endl;  
	//  for(int i = 0; i < coorX.size();i++)  
	//  {  
	////if(i=
	//      cout << coorX[i] << " : " << coorY[i] << " : " << coorZ[i] << endl;  
	//  }  
	//cout << "coorX.size()=" << coorX.size() << endl;
	//cout << coorX.size() / 4 << " triangles." << endl; 


	return true;
}

bool ReadBinary(const char *cfilename, std::vector<float> *coorX, std::vector<float> *coorY, std::vector<float> *coorZ)
{
	char str[80];
	std::ifstream in(cfilename, std::ifstream::in | std::ifstream::binary);

	if (!in)
		return false;

	in.read(str, 80);
	//cout<< "str=" << str <<endl;

	//number of triangles  
	int triangles;
	in.read((char*)&triangles, sizeof(int));
	//cout<< "Number of triangles=" << triangles <<endl;

	if (triangles == 0)
		return false;
	//cout << "2222222222222"<<endl;
	for (int i = 0; i < triangles; i++)
	{
		float coorXYZ[12];
		in.read((char*)coorXYZ, 12 * sizeof(float));

		for (int j = 0; j < 4; j++)
		{
			(*coorX).push_back(coorXYZ[j * 3]);
			(*coorY).push_back(coorXYZ[j * 3 + 1]);
			(*coorZ).push_back(coorXYZ[j * 3 + 2]);
		}

		in.read((char*)coorXYZ, 2);
	}

	in.close();


	cout << "******BINARY FILES******" << endl;
	//  for (int i = 0; i < coorX.size();i++)  
	//  {  
	////if(i=
	//      cout << coorX[i] << " : " << coorY[i] << " : " << coorZ[i] << endl;  
	//  }  
	//cout << "coorX.size()=" << coorX.size() << endl;
	//cout << coorX.size() / 4 << " triangles." << endl; 



	return true;
}

bool ReadSTLFile(const char *cfilename, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals)
{
	if (cfilename == NULL)
		return false;

	std::ifstream in(cfilename, std::ifstream::in);

	if (!in)
		return false;

	char headStr1[100];
	char headStr2[100];

	in.getline(headStr1, 100, '\n');
	in.getline(headStr2, 100, '\n');

	std::vector<float> coorX;
	std::vector<float> coorY;
	std::vector<float> coorZ;

	if (headStr1[0] == 's')
	{
		int i;
		for (i = 0; i < 100; i++)
		{
			if (headStr2[i] != ' ')
			{
				break;
			}

		}
		if (headStr2[i] == 'f')
		{
			ReadASCII(cfilename, &coorX, &coorY, &coorZ);
		}
		else
		{
			ReadBinary(cfilename, &coorX, &coorY, &coorZ);
		}

	}
	else
	{
		ReadBinary(cfilename, &coorX, &coorY, &coorZ);
	}

	//std::vector<float> random_point_x;
	//std::vector<float> random_point_y;
	//std::vector<float> random_point_z;
	//std::vector<float> random_point_normal_x;
	//std::vector<float> random_point_normal_y;
	//std::vector<float> random_point_normal_z;
	std::vector<Utils_sampling::Vec3> verts;
	std::vector<Utils_sampling::Vec3> nors;
	std::vector<int> tris;
	std::vector<Utils_sampling::Vec3> samples_pos;
	std::vector<Utils_sampling::Vec3> samples_nor;
	/*RandomPointGeneraterInTriangles(coorX, coorY, coorZ, &random_point_x, &random_point_y, &random_point_z,
	&random_point_normal_x, &random_point_normal_y, &random_point_normal_z,
	&verts, &nors, &tris);*/
	PrepareVertsAndNors(coorX, coorY, coorZ, &verts, &nors, &tris);
	Utils_sampling::poisson_disk(POISSON_DISK_SAMPLING_RADIUS, 1000, verts, nors, tris, samples_pos, samples_nor);
	/*outputExcel(samples_pos, samples_nor);*/

	string path_temporary = PATH_TEMPORARY_FILE + "random_points.pcd";
	char* path_temporary_file = new char[path_temporary.size() + 1];
	strcpy(path_temporary_file, path_temporary.c_str());
	CreatePointCloud(samples_pos, samples_nor, path_temporary, cloud_filtered, cloud_normals);
	//outputCloudOnPTS(*cloud_filtered, "cloud_filtered");
	/*createpointcloud(random_point_x, random_point_y, random_point_z, path_temporary_file);

	createnormalsofallpoints(random_point_normal_x, random_point_normal_y, random_point_normal_z, path_temporary_file);

	pcl::visualization::PCLVisualizer * mesh_viewer_ = new pcl::visualization::PCLVisualizer("Mesh View");

	mesh_viewer_->addPolygonMesh(triangles, "mesh");

	mesh_viewer_->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor()->GetProperty()->SetInterpolationToPhong();*/

	return true;
}

