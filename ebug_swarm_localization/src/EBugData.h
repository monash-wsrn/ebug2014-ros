#if 1 //WHY?

#ifndef EBUGDATA_H
#define EBUGDATA_H
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <cmath>
#include <numeric>
#include <queue>
#include <bitset>
#include <algorithm>
#include <float.h>

#include"Eigen/Dense.h"

#define WIDTH 1280
#define HEIGHT 960
#define FOCAL_WIDTH (1*WIDTH)
#define MAX_BLOBS 256

#define SCALE 1.0f

#define PRINT_DEBUG 0

using namespace std;

struct eBug {
	int ID;
	float x_pos, y_pos;
	float angle;
};

struct
{
	float x, y, size;
	int colour;
} points[MAX_BLOBS];

struct myEllipse
{
	float x, y, rx, ry, t;
};

typedef unsigned char uint8;
typedef signed long int32;

std::string ExtractCurrentTimeStamp(std::string &st);
void ExtractBlobInformation(std::string &st);
void knn_graph_partition(int n_blobs, std::vector<eBug> &eBugsInfo, int &count);
void identify(std::vector<int> leds, std::vector<eBug> &eBugsInfo, int &count);
myEllipse fitEllipse(std::vector<int> &component);
float min_rounding(std::vector<float> &angles);
float ellipse_to_circle(float phi, float rz);
float circle_to_ellipse(float theta, float rz);
float min_rounding(std::vector<float> &angles);
Eigen::Vector3f eig(Eigen::Matrix3f &M);

#endif

#endif
