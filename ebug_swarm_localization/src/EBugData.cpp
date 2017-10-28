#include "EBugData.h"

using namespace std;
using std::array;

const float pi = 4 * atan(1);
const array<array<int, 16>, 15> seqs{ {
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

void knn_graph_partition(int n_blobs, std::vector<eBug> &eBugsInfo, int &count) //constructs 2-nearest neighbour graph and returns connected components
{
	using namespace std;
	static array<int, MAX_BLOBS> numbers;
	static bool first = true;
	if (first) iota(numbers.begin(), numbers.end(), 0);
	first = false;
	// printf("nblobs: %i, %d, %f, %s\n", n_blobs, n_blobs, n_blobs, n_blobs);
	// printf("nblobs: %i\n", n_blobs);
	// cout << "Vediamo che cazzo Arriva..nblobs: " << n_blobs << " count: " << count << " size: " << eBugsInfo.size() << " MAX_BLOBS: " << MAX_BLOBS  << "\n";
	array<vector<int>, MAX_BLOBS> knngraph;
	for (int i = 0; i<n_blobs; i++) //determine all edges in 2-nearest neighbour graph
	{
		array<int, 3> neighbours;
		static array<int32, MAX_BLOBS> dists;
		for (int j = 0; j<n_blobs; j++)
		{
			int32 x = points[j].x - points[i].x;
			int32 y = points[j].y - points[i].y;
			dists[j] = x*x + y*y; //scale y coordinate by 5 in distance calculation
		}
		partial_sort_copy(numbers.begin(), numbers.begin() + n_blobs, neighbours.begin(), neighbours.end(),
			[](int a, int b){return dists[a]<dists[b]; });

		knngraph[i].insert(knngraph[i].end(), neighbours.begin() + 1, neighbours.end());
		for (int j = 1; j<neighbours.size(); j++) knngraph[neighbours[j]].push_back(i);
	}
	static bitset<MAX_BLOBS> done;
	done.reset();
	
	for (int i = 0; i<n_blobs; i++) //partition graph into connected components
	{
		if (done[i]) continue;
		vector<int> component;
		queue<int> q;
		q.push(i);
		while (!q.empty()) //breadth-first spanning tree search
		{
			auto v = q.front();
			q.pop();

			if (!done[v]) done[v] = true, component.push_back(v);
			for (auto j : knngraph[v])
				if (!done[j]) q.push(j);
		}
		if (component.size() >= 5)
		{
			identify(component, eBugsInfo, count);
			//cout << endl << "Component size = " << component.size() << endl;
		}
	}
}


void identify(std::vector<int> leds, std::vector<eBug> &eBugsInfo, int &count)
{
	float centre_x, centre_y;
	myEllipse e = fitEllipse(leds); //fit ellipse to all leds in component

	float s = sin(e.t);
	float c = cos(e.t);
	vector<int> good;
	for (auto i : leds)
	{
		float x0 = points[i].x - e.x;
		float y0 = points[i].y - e.y;
		float x = (c*x0 + s*y0) / e.rx;
		float y = (-s*x0 + c*y0) / e.ry;
		float r2 = x*x + y*y;
		if (r2>0.9 && r2<1.1) good.push_back(i); //select only points that fit the ellipse well
	}

	if (good.size() < 5)
	{
#if PRINT_DEBUG
		cout << endl << "THE MIN NUMBER OF POINTS TO FIT THE ELLIPSE IS NOT MET\n";
#endif
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
	for (int i = 0; i<good.size(); i++)
	{
		float x0 = points[good[i]].x - e.x;
		float y0 = points[good[i]].y - e.y;
		float x = (c*x0 + s*y0) / e.rx;
		float y = (-s*x0 + c*y0) / e.ry;
		angles[i] = ellipse_to_circle(-atan2(y, x) - pi / 2, rz);
	}
	float base_angle = min_rounding(angles);
#if PRINT_DEBUG
	cout << "\nX = " << centre_x*SCALE << "\tY = " << centre_y*SCALE << "\tAngle = " << base_angle << endl;
#endif
	array<int, 16> rounded;
	fill(rounded.begin(), rounded.end(), 3);
	array<float, 16> size{};
	for (int i = 0; i<good.size(); i++)
	{
		int z = round((angles[i] - base_angle) * 8 / pi + 16);
		z %= 16;
		auto &p = points[good[i]];
		if (size[z]<p.size) size[z] = p.size, rounded[z] = p.colour;
	}

	bool matched = false;
	pair<int, int> id;
	for (int j = 0; j<seqs.size(); j++) for (int k = 0; k<16; k++) //try to match observed sequence to one from the table
	{
		int l;
		for (l = 0; l<16; l++) if (rounded[l] != 3 && rounded[l] != seqs[j][15 - (k + l) % 16]) break;
		if (l == 16)
		{
#if 0
			cout << endl << "match detected" << endl;
#endif
			matched ^= true;
			if (!matched) break; //ensure there is only one match
			id = make_pair(j, k);
		}
	}

	if (!matched) return;

	int led0 = 15 - id.second;
	float phi = circle_to_ellipse(led0*pi / 8 + base_angle, rz) + pi / 2; //find angle of led0

	float x = cos(phi)*e.rx;
	float y = -sin(phi)*e.ry;

	//Find the point(x2,y2) on eBug's boundary which represents led0 (where eBug is facing)
	float x_diff = (-x*cos(e.t) + y*sin(e.t)) * SCALE;
	float y_diff = (x*sin(e.t) + y*cos(e.t)) * SCALE;
	float angle = -atan2(y_diff, x_diff) * 180 / pi;
	if (angle < 0) angle += 360;

	eBugsInfo.push_back({ id.first, centre_x*SCALE, centre_y*SCALE, angle });

	count++;
}

myEllipse fitEllipse(std::vector<int> &component)
{
	using namespace Eigen;

	float xoffset, yoffset;
	Vector3f vx, vy;
	{
		int n;
		Matrix<float, 32, 1> x, y, size;
		float xrange, yrange;
		{
			n = component.size();
			if (n>32) n = 32;
			for (int i = 0; i<n; i++)
			{
				x[i] = points[component[i]].x;
				y[i] = points[component[i]].y;
				size[i] = points[component[i]].size;
			}
			for (int i = n; i<32; i++) x[i] = y[i] = size[i] = 0;

			float xmin = x.segment(0, n).minCoeff();
			float ymin = y.segment(0, n).minCoeff();
			float xmax = x.segment(0, n).maxCoeff();
			float ymax = y.segment(0, n).maxCoeff();

			xoffset = (xmax + xmin) / 2;
			yoffset = (ymax + ymin) / 2;
			xrange = (xmax - xmin) / 2;
			yrange = (ymax - ymin) / 2;

			for (int i = 0; i<n; i++)
			{
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
}

inline float min_rounding(std::vector<float> &angles) //minimise rounding when selecting 16 evenly placed points on circle
{
	float angle = 0.0, min = FLT_MAX;
	for (auto j : angles)
	{
		float r = 0;
		for (auto k : angles)
		{
			float x = (k - j) * 8 / pi + 16;
			x -= round(x);
			r += x*x;
		}
		if (r<min)
		{
			min = r;
			angle = j;
		}
	}
	return angle;
}



inline float ellipse_to_circle(float phi, float rz) //projects ellipse angle onto 3D circle
{
	float K = rz*cos(phi);
	return 2 * atan2(sqrt(1 - K*K) - cos(phi), K + sin(phi));
}

inline float circle_to_ellipse(float theta, float rz) //projects circle angle onto ellipse in image
{
	return atan2(rz + sin(theta), cos(theta));
}


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
}
inline void ExtractBlobInformation(std::string &st) //pass by reference/alias // Don't ask why static inline
{
  vector<eBug> eBugs;
  unsigned char colour;
  int strIdx1, strIdx2, strIdx3, strIdx4, x_pos, y_pos, radius, n_blobs, count;	
  string strTemp, blobInfo, numKey = "0123456789", openBracket = "(", closeBracket = ")";

  strIdx1 = st.find_first_of(openBracket);
  strIdx2 = st.find(closeBracket,strIdx1);

  n_blobs = 0; count = 0;
  if (strIdx1 >= 0 && strIdx2 >= 0)
    {
      do 
	{
	  //get "one" blob information
	  strTemp = st.substr(strIdx1, strIdx2 - strIdx1 + 1);
	  //get each attribute of one blob < x_pos, y_pos, colour, size>
	  strIdx3 = strTemp.find_first_of(numKey);
	  strIdx4 = strTemp.find(",", strIdx3);
	  x_pos = stoi(strTemp.substr(strIdx3, strIdx4 - strIdx3)); //cout << strIdx1 <<"#" << strIdx2 <<"*";

	  strIdx3 = strTemp.find_first_of(numKey, strIdx4);
	  strIdx4 = strTemp.find(",", strIdx3);
	  y_pos = stoi(strTemp.substr(strIdx3, strIdx4 - strIdx3)); //cout << strIdx1 << "#" << strIdx2 << "*";

	  strIdx3 = strTemp.find_first_of(numKey, strIdx4);
	  strIdx4 = strTemp.find(",", strIdx3);
	  colour = stoi(strTemp.substr(strIdx3, strIdx4 - strIdx3)); //cout << strIdx1 << "#" << strIdx2 << "*";

	  strIdx3 = strTemp.find_first_of(numKey, strIdx4);
	  strIdx4 = strTemp.find(")", strIdx3);//ends with closing bracket
	  radius = stoi(strTemp.substr(strIdx3, strIdx4 - strIdx3)); //cout << strIdx1 << "#" << strIdx2 << "*";

	  points[n_blobs].x      = (float)x_pos;
	  points[n_blobs].y      = (float)y_pos;
	  points[n_blobs].size   = (float)radius;
	  points[n_blobs].colour = colour;

	  n_blobs++;
#if PRINT_DEBUG
	  cout << setw(10) << left << n_blobs;
	  cout << setw(15) << left << x_pos; 
	  cout << setw(15) << left << y_pos;
	  cout << setw(10) << left << (int)colour;
	  cout << setw(8) << left << radius << endl;
#endif
	  //remove one blob information and move to next blob
	  st.erase(0, strIdx2 + 1);
	  strIdx1 = st.find_first_of(openBracket);
	  strIdx2 = st.find_first_of(closeBracket, strIdx1);
	} while (strIdx1 >= 0 && strIdx2 >= 0);
    }
  if (n_blobs >= 5) //apply K-nearest neighbour algorithm to classify groups of blobs as the eBugs
    {
      knn_graph_partition(n_blobs, eBugs, count);
    }
  cout << "size = " << eBugs.size() << endl;
  for (int i = 0; i < (int)eBugs.size(); i++)
    {
      cout << endl << "---------------eBugs Detected---------------------" << endl;
      cout << "ID =  " << eBugs[i].ID << "\t" << "x = " << eBugs[i].x_pos << "\t";
      cout << "y = " << eBugs[i].y_pos << "\t" << "angle = " << eBugs[i].angle << endl;
    }
}

inline std::string ExtractCurrentTimeStamp(std::string &st) //pass by reference/alias
{
  int strIdx;
  string strTemp;
#if USE_NEW_STRING_FORMAT
  strIdx = st.find_first_of("0123456789");
  strIdx = st.find_first_of(" ",strIdx);
#else
  strIdx = st.find_first_of(",");
#endif
  if (strIdx >= 0)
    {
      strTemp = st.substr(0, strIdx);
      st.erase(0, strIdx + 1);
      return strTemp;
    }
  else
    {
      return "\0";
    }
}
