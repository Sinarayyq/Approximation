//Lisa Chiang
//This functions are used to read the parameters from a txt file and to set the parameters of the algorithm.

//#pragma once //If it is not added there is an error. It could disappear whenever I use this header (?)

// This is start of the header guard.  READ_PARAMETERS_H can be any unique name.  
// By convention, we use the name of the header file.
#ifndef READ_PARAMETERS_H
#define READ_PARAMETERS_H

#include <string>
#include <vector>

extern std::string PATH_PCD_DOC;
extern int PLANE_METHOD_TYPE;
extern int CYL_METHOD_TYPE;
extern int CONE_METHOD_TYPE;
extern double PLANE_TOL;
extern double CYL_TOL;
extern double CONE_TOL;
extern double MIN_INLIERS;
extern double MST_RATIO;
extern double MST_RATIO2;
extern int PLANE_MAX_NUM_ITER;
extern int CYL_MAX_NUM_ITER;
extern int CONE_MAX_NUM_ITER;
extern int K_FOR_NORMAL_SEARCH;
extern double CYL_WEIGHT_NORMAL_DISTANCE;
extern double CONE_WEIGHT_NORMAL_DISTANCE;
extern double CYL_MIN_RADIUS_LIMIT;
extern double CYL_MAX_RADIUS_LIMIT;
extern double CONE_MIN_RADIUS_LIMIT;
extern double CONE_MAX_RADIUS_LIMIT;
extern double CONE_MIN_OPENING_ANGLE;
extern double CONE_MAX_OPENING_ANGLE;
extern double TOLERANCE_FOR_ADJACENT_CHAINS;
extern double TOLERANCE_FOR_APPEX;
extern double POISSON_DISK_SAMPLING_RADIUS;
extern std::string PATH_TEMPORARY_FILE;
extern double PLANE_AREA_PERCENTAGE;
extern double CYL_AREA_PERCENTAGE;
extern double CONE_AREA_PERCENTAGE;

extern const int INF;

// It reads and sets the method parameters from a txt file 
// INPUT: the path corresponding to the txt file
// OUTPUT: it returns true if the file contains the correct number of parameters (21), false otherwise
bool readParameterFile(std::string parameterTxtFilePath);

// It sets the method parameters in the corresponding variables 
// INPUT: a vector of strings coming from the txt file
void setParameters(std::vector<std::string>& parameters);

#endif