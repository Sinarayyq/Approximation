#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <limits>

#include <pcl/sample_consensus/method_types.h>
#include "read_parameters.h"

std::string PATH_PCD_DOC;
int PLANE_METHOD_TYPE;
int CYL_METHOD_TYPE;
int CONE_METHOD_TYPE;
double PLANE_TOL;
double CYL_TOL;
double CONE_TOL;
double MIN_INLIERS;
double MST_RATIO;
double MST_RATIO2;
int PLANE_MAX_NUM_ITER;
int CYL_MAX_NUM_ITER;
int CONE_MAX_NUM_ITER;
int K_FOR_NORMAL_SEARCH;
double CYL_WEIGHT_NORMAL_DISTANCE;
double CONE_WEIGHT_NORMAL_DISTANCE;
double CYL_MIN_RADIUS_LIMIT;
double CYL_MAX_RADIUS_LIMIT;
double CONE_MIN_RADIUS_LIMIT;
double CONE_MAX_RADIUS_LIMIT;
double CONE_MIN_OPENING_ANGLE;
double CONE_MAX_OPENING_ANGLE;
double TOLERANCE_FOR_ADJACENT_CHAINS;
double TOLERANCE_FOR_APPEX;
double POISSON_DISK_SAMPLING_RADIUS;
std::string PATH_TEMPORARY_FILE;
double PLANE_AREA_PERCENTAGE;
double CYL_AREA_PERCENTAGE;
double CONE_AREA_PERCENTAGE;

const int INF = std::numeric_limits<int>::max();

void setParameters(std::vector<std::string>& parameters)
{
	for (std::size_t i = 0; i < parameters.size(); ++i)
	{
		switch (i)
		{
		case 0: {PATH_PCD_DOC = parameters[i]; }
				break;
		case 1: {
			if (std::stoi(parameters[i]) == 0)
			{
				PLANE_METHOD_TYPE = pcl::SAC_RANSAC;
			}
			else if (std::stoi(parameters[i]) == 1)
			{
				PLANE_METHOD_TYPE = pcl::SAC_PROSAC;
			}
			else
			{
				PLANE_METHOD_TYPE = INF;
			}
		}
				break;
		case 2: {
			if (std::stoi(parameters[i]) == 0)
			{
				CYL_METHOD_TYPE = pcl::SAC_RANSAC;
			}
			else if (std::stoi(parameters[i]) == 1)
			{
				CYL_METHOD_TYPE = pcl::SAC_PROSAC;
			}
			else
			{
				CYL_METHOD_TYPE = INF;
			}
		}
				break;
		case 3: {
			if (std::stoi(parameters[i]) == 0)
			{
				CONE_METHOD_TYPE = pcl::SAC_RANSAC;
			}
			else if (std::stoi(parameters[i]) == 1)
			{
				CONE_METHOD_TYPE = pcl::SAC_PROSAC;
			}
			else
			{
				CONE_METHOD_TYPE = INF;
			}
		}
				break;
		case 4: {PLANE_TOL = std::stod(parameters[i]); }
				break;
		case 5: {CYL_TOL = std::stod(parameters[i]); }
				break;
		case 6: {CONE_TOL = std::stod(parameters[i]); }
				break;
		case 7: {MIN_INLIERS = std::stod(parameters[i]); }
				break;
		case 8: {MST_RATIO = std::stod(parameters[i]); }
				break;
		case 9: {MST_RATIO2 = std::stod(parameters[i]); }
				break;
		case 10: {PLANE_MAX_NUM_ITER = std::stoi(parameters[i]); }
				 break;
		case 11: {CYL_MAX_NUM_ITER = std::stoi(parameters[i]); }
				 break;
		case 12: {CONE_MAX_NUM_ITER = std::stoi(parameters[i]); }
				 break;
		case 13: {K_FOR_NORMAL_SEARCH = std::stoi(parameters[i]); }
				 break;
		case 14: {CYL_WEIGHT_NORMAL_DISTANCE = std::stod(parameters[i]); }
				 break;
		case 15: {CONE_WEIGHT_NORMAL_DISTANCE = std::stod(parameters[i]); }
				 break;
		case 16: {CYL_MIN_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 17: {CYL_MAX_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 18: {CONE_MIN_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 19: {CONE_MAX_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 20: {CONE_MIN_OPENING_ANGLE = std::stod(parameters[i]); }
				 break;
		case 21: {CONE_MAX_OPENING_ANGLE = std::stod(parameters[i]); }
				 break;
		case 22: {TOLERANCE_FOR_ADJACENT_CHAINS = std::stod(parameters[i]); }
				 break;
		case 23: {TOLERANCE_FOR_APPEX = std::stod(parameters[i]); }
				 break;
		case 24: {POISSON_DISK_SAMPLING_RADIUS = std::stoi(parameters[i]); }
				 break;
		case 25: {PATH_TEMPORARY_FILE = parameters[i]; }
				 break;
		case 26: {PLANE_AREA_PERCENTAGE = std::stod(parameters[i]); }
				 break;
		case 27: {CYL_AREA_PERCENTAGE = std::stod(parameters[i]); }
				 break;
		case 28: {CONE_AREA_PERCENTAGE = std::stod(parameters[i]); }
				 break;

		}
	}
}

bool readParameterFile(std::string parameterTxtFilePath)
{
	std::ifstream infile(parameterTxtFilePath);
	std::string line;
	std::vector<std::string> parameters;
	const size_t num_parameters = 29;
	//size_t n_lines = 0;

	while (std::getline(infile, line))
	{
		if (line[0] != '#')
		{
			parameters.push_back(line);
			//std::istringstream iss(line);
			//int a, b;
			//if (!(iss >> a >> b)) { break; } // error

			// process pair (a,b)
		}
		//++n_lines;
	}

	////Print parameters in cmd window:
	//for (std::size_t i = 0; i < parameters.size(); ++i)
	//{
	//	std::cerr << parameters[i] << std::endl;
	//}
	//std::cerr << "numero di paramentri: " << parameters.size() << std::endl;

	//Check if the parameter file contains the right number of parameters
	if (parameters.size() != num_parameters)
	{
		std::cerr << "ERROR in the text file of parameters!" << std::endl;
		return false;
	}
	//std::cerr << "The parameter file is correct." << std::endl;
	setParameters(parameters);
	return true;
}