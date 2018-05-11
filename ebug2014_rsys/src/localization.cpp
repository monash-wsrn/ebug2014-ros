/*
  Tracking eBugs' locations and ID based on deBruijn Sequences      
  Originally written by Tony Grubman                    
  Modified by Erwin Mochtar Wijaya on 05/02/2014            
  Modified by Ahmet Sekercioglu on 22/03/2018        
  Modified by Ahmet Sekercioglu on 03/03/2018 to adapt to ROS environment. 
  Modified by Ahmet Sekercioglu on 11/04/2018 to adapt to ROS environment. 
                                   (track_ebugs.cpp,v 1.6)
*/

/*
  Remember, to compile  -std=c++11  is needed. 
*/

/*
  jeudi 22 mars 2018, 13:52:34 (UTC+0100) Ahmet
  * Eigen library Erwin used is too old.
  * Updated Eigen to Version 3.3.4
  * try_eigen.cpp: I used it for testing the eigen library and
    compilation command.
  * Renamed main.cpp as track_ebugs.cpp
  * Added #include"float.h" to eliminate the error  "FLT_MAX was
    not declared in this scope"
  * Moved the routines in the LedDetection.h in here, deleted LedDetection.h
*/

/*
  Monday 30 April 14:37:40 CEST 2018
  Tony's fix:
  
  It seems it was trying to match the wrong way around the circle (it
  should look for the sequences anticlockwise I think). I'm not sure
  why this is happening on one of the logs and not the other. It could
  be that the image coordinates are flipped in one axis.

  Anyway, this fixed it:
  403c403
  < 	if (rounded[l] != 3 && rounded[l] != seqs[j][15 - (k + l) % 16])
  ---
  > 	if (rounded[l] != 3 && rounded[l] != seqs[j][(k + l) % 16])
  
  You should check that the angle it reports is correct.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <cmath>
#include <numeric>
#include <array>
#include <queue>
#include <bitset>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "float.h"
#include "Eigen/Dense"
#include "debug.h" // My PRINTFDB macro,
                   // to turn on add -DDEBUG to the compilation line
#include "EBugData.h"

using namespace std;

// Todo We need to check these magic numbers. Some calibration is needed. 
#define WIDTH 1280
#define HEIGHT 960
#define FOCAL_WIDTH (1*WIDTH)
#define MAX_BLOBS 256

#define SCALE 1.0f

typedef unsigned char uint8;
typedef signed long int32;

struct {
  float x, y, size;
  uint8 colour;
} points[MAX_BLOBS];

struct myEllipse {
  float x, y, rx, ry, t;
};

// const cv::Scalar colours[] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 128, 0), cv::Scalar(255, 0, 0), cv::Scalar(0) };

// Function prototypes
void blobsCallback(const std_msgs::String::ConstPtr &msg);
static inline string ExtractCurrentTimeStamp(string &st);
static inline int ExtractBlobInformation(string &st, vector<eBug> &eBugs);
Eigen::Vector3f eig(Eigen::Matrix3f &M);
myEllipse fitEllipse(std::vector<uint8> &component);
void identify(std::vector<uint8> leds, std::vector<eBug> &eBugsInfo,
           	                                                int &count);
void knn_graph_partition(uint8 n_blobs, std::vector<eBug> &eBugsInfo,
     			                                        int &count);
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<std_msgs::String>("poses", 100);
    sub_ = n_.subscribe("blobs", 100, &SubscribeAndPublish::blobsCallback,this);
  } // SubscribeAndPublish()
  
  void blobsCallback(const std_msgs::String::ConstPtr &blobs_msg)
  {
    // We get the blobs, calculate the eBug poses and write them
    // into the "poses" topic.
    vector<eBug> eBugs;
    string strBlobsInfo, strTStamp;
    int n_blobs, count = 0;
    stringstream ss;
    std_msgs::String poses_msg;
    
    strBlobsInfo = blobs_msg->data.c_str();
    
    // Get current time-stamp and convert from string format to integer
    strTStamp = ExtractCurrentTimeStamp(strBlobsInfo);
    ROS_INFO("Packet received, timestamp: [%s]", strTStamp.c_str());
    
    eBugs.clear(); 
    n_blobs = ExtractBlobInformation(strBlobsInfo, eBugs); 
    if (n_blobs >= 5) {
      // Apply K-nearest neighbour algorithm to classify groups of blobs
      // as the eBugs
      knn_graph_partition(n_blobs, eBugs, count);
    }
    if (eBugs.size() == 0) {
      ROS_INFO("[%s] No eBug(s) detected.", strTStamp.c_str());
    } else {
      ss.str(""); 
      ROS_INFO("[%s] %d eBug(s) detected:",
	                            strTStamp.c_str(), (int) eBugs.size());
      ss << eBugs.size() << " ";
      for (int i = 0; i < (int) eBugs.size(); i++) {
	ROS_INFO("   %d %.0f %.0f %.0f", eBugs[i].ID,
		             eBugs[i].x_pos, eBugs[i].y_pos, eBugs[i].angle);
	ss << eBugs[i].ID << " " <<  eBugs[i].x_pos << " " <<
	                     eBugs[i].y_pos << " " <<  eBugs[i].angle << " ";
      } 
      poses_msg.data = ss.str();
      pub_.publish(poses_msg);
    } 
  } // blobsCallback()

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;;
}; // class SubscribeAndPublish

static inline string ExtractCurrentTimeStamp(string &st) 
{
  stringstream ss;
  char delim = ' ';
  string strFrameNo;
  string strTStamp;

  ss << st;

  ss >> strFrameNo; 
  ss >> strTStamp;  

  return strTStamp;
} // ExtractCurrentTimeStamp()

static inline int ExtractBlobInformation(string &st, vector<eBug> &eBugs)
{
  stringstream ss;
  string token;
  int x_pos, y_pos, radius, n_blobs, count;	
  unsigned char colour;
  
  ss << st;
  
  ss >> token;  // Discard the frame no
  ss >> token;  // Discard the timestamp

  int blob = 0;
  while (ss >> token) {  // Each blob is a 4-tuple 
    points[blob].x =  stof(token);
    ss >> token;
    points[blob].y =  stof(token);
    ss >> token;
    points[blob].colour = atoi(token.c_str());
    ss >> token;
    points[blob].size = stof(token);
    ++blob;
  } //
  n_blobs = blob; 
  PRINTFDB(("Blob\tX-Pos\tY-Pos\tColour\tRadius\n"));
  for (int blob = 0; blob < n_blobs; blob++) {
    PRINTFDB(("%d\t%.0f\t%.0f\t%d\t%.0f\n", blob, points[blob].x,
	      points[blob].y, points[blob].colour, points[blob].size));
    
  } 
  return n_blobs;
} // ExtractBlobInformation()

Eigen::Vector3f eig(Eigen::Matrix3f &M)
{
  using namespace Eigen;
  
  float q = M.trace() / 3;
  Matrix3f A = M - q*Matrix3f::Identity();
  float det = A.determinant();
  
  Matrix3f B = A*A;
  float p2 = B.trace() / 6;
  float p = sqrt(p2);
  det /= (p*p2);
  
#if 0
  float z = sqrt(det + 2);
  z = z / 2 + 1;
  if (det>-1) z = (z / 2 + .99985)*(1.0145 - .00725*det);
  else z = z / 1.75 + 0.921620629618769 - 0.0393396851906154*det;
#else
  float z = 2 * cos(acos(det / 2) / 3);
#endif

  float e0 = q + p*z;
  z /= 2;
  float e1 = q + p*(sqrt(3 - 3 * z*z) - z);
  
  Matrix3f E0 = M - e0*Matrix3f::Identity();
  Matrix3f E1 = M - e1*Matrix3f::Identity();
  
  return (E0*E1).col(0);
} // eig()

myEllipse fitEllipse(std::vector<uint8> &component)
{
  using namespace Eigen;
  
  float xoffset, yoffset;
  Vector3f vx, vy;
  {
    uint8 n;
    Matrix<float, 32, 1> x, y, size;
    float xrange, yrange;
    {
      n = component.size();
      if (n>32) n = 32;
      for (uint8 i = 0; i<n; i++) {
	x[i] = points[component[i]].x;
	y[i] = points[component[i]].y;
	size[i] = points[component[i]].size;
      }
      for (uint8 i = n; i<32; i++) x[i] = y[i] = size[i] = 0;
      
      float xmin = x.segment(0, n).minCoeff();
      float ymin = y.segment(0, n).minCoeff();
      float xmax = x.segment(0, n).maxCoeff();
      float ymax = y.segment(0, n).maxCoeff();
      
      xoffset = (xmax + xmin) / 2;
      yoffset = (ymax + ymin) / 2;
      xrange = (xmax - xmin) / 2;
      yrange = (ymax - ymin) / 2;
      
      for (uint8 i = 0; i<n; i++) {
	x[i] -= xoffset;
	x[i] /= xrange;
	y[i] -= yoffset;
	y[i] /= yrange;
      }
    }

    Matrix3f A, B, C, D;
    
    {
      Matrix<float, 32, 1> xx = x.array()*x.array(), xy = x.array()*y.array(), yy = y.array()*y.array();
      Matrix<float, 32, 1> xxs = xx.array()*size.array(), xys = xy.array()*size.array(), yys = yy.array()*size.array();
      A << xx.dot(xxs), xx.dot(xys), xx.dot(yys),
	xx.dot(xys), xy.dot(xys), xy.dot(yys),
	xx.dot(yys), xy.dot(yys), yy.dot(yys);
      C << xxs.sum(), xys.sum(), x.dot(size),
	xys.sum(), yys.sum(), y.dot(size),
	x.dot(size), y.dot(size), size.sum();
      B << x.dot(xxs), x.dot(xys), x.dot(yys),
	x.dot(xys), x.dot(yys), y.dot(yys),
	C(0, 0), C(0, 1), C(1, 1);
      D << 0, 0, -0.5,
	0, 1, 0,
	-0.5, 0, 0;
    }
    
    C = -C.inverse()*B;
    D *= (A + B.transpose()*C);
    
    vx = eig(D);
    vy = C*vx;
    
    vx[0] *= yrange*yrange;
    vx[1] *= xrange*yrange;
    vx[2] *= xrange*xrange;
    vy[0] *= xrange*yrange*yrange;
    vy[1] *= xrange*xrange*yrange;
    vy[2] *= xrange*xrange*yrange*yrange;
  }
  
  float t = 0.5*atan2(vx[1], vx[0] - vx[2]);
  float cost = cos(t);
  float sint = sin(t);
  
  float cos_squared = cost*cost, cos_sin = cost*sint, sin_squared = sint*sint;
  float Au = vy[0] * cost + vy[1] * sint;
  float Av = -vy[0] * sint + vy[1] * cost;
  float Auu = vx[0] * cos_squared + vx[1] * cos_sin + vx[2] * sin_squared;
  float Avv = vx[0] * sin_squared - vx[1] * cos_sin + vx[2] * cos_squared;
  
  float tuCentre = -Au / (2 * Auu);
  float tvCentre = -Av / (2 * Avv);
  float wCentre = vy[2] - Auu*tuCentre*tuCentre - Avv*tvCentre*tvCentre;
  float uCentre = tuCentre*cost - tvCentre*sint;
  float vCentre = tuCentre*sint + tvCentre*cost;
  
  float Ru = sqrt(fabs(wCentre / Auu));
  float Rv = sqrt(fabs(wCentre / Avv));
  
  uCentre += xoffset;
  vCentre += yoffset;
  
  if (t<0) t += 4 * atan(1);
  
  return{ uCentre, vCentre, Ru, Rv, t };
} // fitEllipse()

const float pi = 4 * atan(1);

inline float ellipse_to_circle(float phi, float rz) 
{
  // Projects ellipse angle onto 3D circle
  float K = rz*cos(phi);
  return 2 * atan2(sqrt(1 - K*K) - cos(phi), K + sin(phi));
} // ellipse_to_circle()

inline float circle_to_ellipse(float theta, float rz) 
{
  // Projects circle angle onto ellipse in image
  return atan2(rz + sin(theta), cos(theta));
} // circle_to_ellipse()

inline float min_rounding(std::vector<float> &angles) 
{
  // Minimise rounding when selecting 16 evenly placed points on circle
  float angle = 0.0, min = FLT_MAX;
  for (auto j : angles)  {
    float r = 0;
    for (auto k : angles) {
      float x = (k - j) * 8 / pi + 16;
      x -= round(x);
      r += x*x;
    }
    if (r<min) {
      min = r;
      angle = j;
    }
  }
  return angle;
} // min_rounding()

using std::array;

const array<array<uint8, 16>, 15> seqs{ {
	{ { 2, 2, 1, 0, 0, 2, 0, 2, 1, 2, 2, 1, 2, 2, 0, 1 } },
	{ { 0, 1, 0, 0, 0, 1, 0, 2, 1, 2, 0, 1, 2, 1, 0, 1 } },
	{ { 2, 0, 1, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 1, 0 } },
	{ { 0, 2, 0, 1, 0, 2, 2, 0, 2, 2, 2, 0, 1, 0, 1, 2 } },
	{ { 1, 1, 2, 1, 2, 0, 2, 2, 1, 0, 1, 2, 2, 2, 1, 1 } },
	{ { 0, 1, 2, 0, 1, 1, 2, 2, 2, 0, 0, 1, 2, 1, 1, 0 } },
	{ { 1, 0, 0, 2, 2, 0, 1, 1, 0, 2, 2, 2, 1, 2, 0, 0 } },
	{ { 2, 1, 1, 0, 2, 0, 2, 2, 0, 0, 0, 2, 1, 2, 1, 0 } },
	{ { 1, 2, 0, 0, 2, 2, 1, 1, 2, 0, 2, 1, 0, 0, 1, 1 } },
	{ { 0, 1, 1, 0, 1, 2, 1, 2, 2, 2, 2, 0, 2, 1, 1, 1 } },
	{ { 0, 1, 2, 0, 0, 0, 0, 0, 1, 2, 2, 0, 2, 0, 0, 2 } },
	{ { 0, 0, 1, 0, 1, 1, 1, 0, 2, 1, 0, 1, 1, 2, 0, 1 } },
	{ { 0, 1, 0, 2, 0, 0, 0, 1, 1, 2, 1, 1, 2, 2, 1, 1 } },
	{ { 1, 1, 1, 2, 2, 0, 0, 2, 1, 0, 2, 2, 1, 2, 1, 2 } },
	{ { 0, 2, 1, 1, 2, 1, 0, 0, 0, 0, 2, 0, 0, 1, 1, 0 } },
	} };

void identify(std::vector<uint8> leds, std::vector<eBug> &eBugsInfo, int &count)
{
  float centre_x, centre_y;
  myEllipse e = fitEllipse(leds); // Fit ellipse to all leds in component

  float s = sin(e.t);
  float c = cos(e.t);
  vector<uint8> good;
  for (auto i : leds)	{
    float x0 = points[i].x - e.x;
    float y0 = points[i].y - e.y;
    float x = (c*x0 + s*y0) / e.rx;
    float y = (-s*x0 + c*y0) / e.ry;
    float r2 = x*x + y*y;
    // Select only points that fit the ellipse well:
    if (r2>0.9 && r2<1.1) good.push_back(i); 
  } // for

  if (good.size() < 5) {
    PRINTFDB(("The minimum number of points to fit the ellipse is not met.\n"));
    return;
  }
  e = fitEllipse(good); //refit ellipse to only good points
  
  centre_x = WIDTH - e.x;
  centre_y = e.y;
  //Point2f centre(WIDTH - e.x, e.y);
  
  s = sin(e.t);
  c = cos(e.t);
  
  float rz = max(e.rx, e.ry) / FOCAL_WIDTH;
  vector<float> angles(good.size());
  for (uint8 i = 0; i<good.size(); i++)	{
    float x0 = points[good[i]].x - e.x;
    float y0 = points[good[i]].y - e.y;
    float x = (c*x0 + s*y0) / e.rx;
    float y = (-s*x0 + c*y0) / e.ry;
    angles[i] = ellipse_to_circle(-atan2(y, x) - pi / 2, rz);
  }
  float base_angle = min_rounding(angles);
  array<uint8, 16> rounded;
  fill(rounded.begin(), rounded.end(), 3);
  array<float, 16> size{};
  for (uint8 i = 0; i<good.size(); i++)	{
    int z = round((angles[i] - base_angle) * 8 / pi + 16);
    z %= 16;
    auto &p = points[good[i]];
    if (size[z]<p.size) size[z] = p.size, rounded[z] = p.colour;
  } // for
  
  bool matched = false;
  pair<uint8, uint8> id;
  for (uint8 j = 0; j<seqs.size(); j++) {
    for (uint8 k = 0; k<16; k++) {
      // Try to match observed sequence to one from the table
      uint8 l;
      for (l = 0; l<16; l++)
	if (rounded[l] != 3 && rounded[l] != seqs[j][(k + l) % 16])
	  break;	
      if (l == 16) {
	matched ^= true;
	if (!matched) break; //ensure there is only one match
	id = make_pair(j, k);
      } // if
    } // for
  } // for
  if (!matched) return;
  
  uint8 led0 = 15 - id.second;
  // Find angle of led0:
  float phi = circle_to_ellipse(led0*pi / 8 + base_angle, rz) + pi / 2; 
  
  float x = cos(phi)*e.rx;
  float y = -sin(phi)*e.ry;
  
  // Find the point(x2,y2) on eBug's boundary which represents led0
  // (where eBug is facing)
  float x_diff = (-x*cos(e.t) + y*sin(e.t)) * SCALE;
  float y_diff = (x*sin(e.t) + y*cos(e.t)) * SCALE;
  float angle = -atan2(y_diff, x_diff) * 180 / pi;
  if (angle < 0) angle += 360;
  
  eBugsInfo.push_back({ id.first, centre_x*SCALE, centre_y*SCALE, angle });
  
  count++;
} // identify()

void knn_graph_partition(uint8 n_blobs, std::vector<eBug> &eBugsInfo, int &count) 
{
  // Constructs 2-nearest neighbour graph and returns connected components
  using namespace std;
  
  static array<uint8, MAX_BLOBS> numbers;
  static bool first = true;
  if (first) iota(numbers.begin(), numbers.end(), 0);
  first = false;
  
  array<vector<uint8>, MAX_BLOBS> knngraph;

  for (uint8 i = 0; i < n_blobs; i++) {
    // Determine all edges in 2-nearest neighbour graph
    array<uint8, 3> neighbours;
    static array<int32, MAX_BLOBS> dists;
    for (uint8 j = 0; j<n_blobs; j++) {
      int32 x = points[j].x - points[i].x;
      int32 y = points[j].y - points[i].y;
      dists[j] = x*x + y*y; //scale y coordinate by 5 in distance calculation
    }
    partial_sort_copy(numbers.begin(), numbers.begin() + n_blobs,
		      neighbours.begin(), neighbours.end(),
		      [](uint8 a, uint8 b){return dists[a]<dists[b]; });
    
    knngraph[i].insert(knngraph[i].end(), neighbours.begin() + 1,
		       neighbours.end());
    for (uint8 j = 1; j<neighbours.size(); j++)
      knngraph[neighbours[j]].push_back(i);
  } // for
  
  static bitset<MAX_BLOBS> done;
  done.reset();
  
  for (uint8 i = 0; i<n_blobs; i++) {
    // Partition graph into connected components
    if (done[i]) continue;
    vector<uint8> component;
    queue<uint8> q;
    q.push(i);
    while (!q.empty()) {
      // Breadth-first spanning tree search
      auto v = q.front();
      q.pop();
      
      if (!done[v]) done[v] = true, component.push_back(v);
      for (auto j : knngraph[v])
	if (!done[j]) q.push(j);
    } // while
    if (component.size() >= 5) {
      identify(component, eBugsInfo, count);
    }
  }
} // knn_graph_partition()

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
} // main()

