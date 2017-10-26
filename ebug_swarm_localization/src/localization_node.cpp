#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/dataArray.h"

#include "EBugData.h"
using namespace std;

#define PRINT_DEBUG 0
#define USE_NEW_STRING_FORMAT 0

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




ros::Publisher *pubPtr;

void dataReceived(const ebug_swarm_msg::dataArray::ConstPtr& arrayIn){
    ROS_INFO_STREAM("Localization node: blobs arrived and poses sent. Timestamp: " 
        << arrayIn->timeStamp);
    ebug_swarm_msg::dataArray arrayOut;
    arrayOut = *arrayIn;
    pubPtr->publish(arrayOut);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_node");

    /******Variable definitions******/
  //vector<eBug> eBugData;
  int test;
  unsigned long long timeStamp;
  ifstream eBugLogFile;
  std::string strBlobsInfo, strTemp;

  //Open log file
#if USE_NEW_STRING_FORMAT == 1
  eBugLogFile.open("1ebug_stationary_20sec_frames.txt");
#else
  eBugLogFile.open("3ebugs_60sec_85cm_reassembled.log");
#endif
  if (eBugLogFile.fail())
    {
      cout << "\n cannot open the log file!" << endl;
      exit(1);
    }
        
  //Reading from the Log File
  while (eBugLogFile.good())
    {
      //obtain all blobs position and colours at current time-stamp
      getline(eBugLogFile, strBlobsInfo); 
      //1) Get current time-stamp and convert from string format to integer
      strTemp = ExtractCurrentTimeStamp(strBlobsInfo);
#if USE_NEW_STRING_FORMAT == 0
      if (strTemp != "\0")
	timeStamp = stoi(strTemp);
      else // EOF
	continue;
#endif
      //2) Get blobs position, colour and size information
      ExtractBlobInformation(strBlobsInfo);
#if PRINT_DEBUG
#if USE_NEW_STRING_FORMAT == 0
      cout << endl << setw(20) << left << "Time Stamp = " << timeStamp << endl; 
#else//#if USE_NEW_STRING_FORMAT == 1
      cout << endl << setw(20) << left << strTemp << endl;
#endif //#if USE_NEW_STRING_FORMAT == 0
      cout << setw(10) << left << "Blob";
      cout << setw(15) << left << "X-Position" << setw(15) << left << "Y-Position" << setw(10) << left << "Colour" << setw(8) << left << "Radius" << endl;
#endif
    }

  //End of Program
  eBugLogFile.close();
  



  
    ros::NodeHandle nh;
    pubPtr = new ros::Publisher(
        nh.advertise<ebug_swarm_msg::dataArray>("poses",1000));    
    ros::Subscriber sub = nh.subscribe("blobs", 1000, &dataReceived);    
    ros::spin();
    return 0;
}
