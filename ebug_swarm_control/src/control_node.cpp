#include "ebug_swarm_control/control_node.h"

using namespace std;

ros::Publisher *pubPtr;
std::string position;

void dataReceived(const ebug_swarm_msg::dataArray& arrayIn){
    ROS_INFO_STREAM("Control node: Commands received and blobs sent. Timestamp: " 
        << arrayIn.timeStamp);

    // Read From File
    /******Variable definitions******/
    //vector<eBug> eBugData;
    ifstream eBugLogFile;
    std::string strBlobsInfo, strTemp;

    //Open log file
    eBugLogFile.open(position);

    if (eBugLogFile.fail())
      {
	cout << "\n cannot open the log file!" << endl;
	exit(1);
      }
        
    //Reading from the Log File
    while (getline(eBugLogFile, strBlobsInfo))
      {

	ebug_swarm_msg::frame frameOut;
	ebug_swarm_msg::blob blobOut;
	//getline(eBugLogFile, strBlobsInfo); 


	// unsigned char colour;
	int colour;
	int strIdx1, strIdx2, strIdx3, strIdx4, x_pos, y_pos, radius, n_blobs, count;	
	string strTemp, blobInfo, numKey = "0123456789", openBracket = "(", closeBracket = ")";
	string frame_id;
	
	int strIdxfIF = 0;
	int strIdxfIDo = 0;
	strIdxfIF = strBlobsInfo.find_first_of("0123456789");
	strIdxfIDo = strBlobsInfo.find_first_of(" ", strIdxfIF );
	frame_id = strBlobsInfo.substr(strIdxfIF, strIdxfIDo-strIdxfIF);

	strBlobsInfo.erase(strIdxfIF, strIdxfIDo-strIdxfIF + 1);


        string time_stamp;
	strIdxfIF = strBlobsInfo.find_first_of("0123456789");
	strIdxfIDo = strBlobsInfo.find_first_of(" ", strIdxfIF );
        time_stamp = strBlobsInfo.substr(strIdxfIF, strIdxfIDo-strIdxfIF);
	
	strBlobsInfo.erase(0, strIdxfIDo-strIdxfIF + 1);
	
	strIdx1 = strBlobsInfo.find_first_of(openBracket);
	strIdx2 = strBlobsInfo.find(closeBracket,strIdx1);

	// cout << "precast check. Frame ID: " << frame_id << " timestamp: " << time_stamp << endl;

	frameOut.frame_id = atoi(frame_id.c_str());
	frameOut.timeStamp = atoi(time_stamp.c_str());

	// cout << "Dentro al tunnell" << endl;
	// cout << "blobsize Begin " << frameOut.blobs.size() << endl;
	
	n_blobs = 0; count = 0;
	if (strIdx1 >= 0 && strIdx2 >= 0)
	  {
	    do 
	      {
		// cout << "blobsize before " << frameOut.blobs.size() << endl;
		//get "one" blob information
		strTemp = strBlobsInfo.substr(strIdx1, strIdx2 - strIdx1 + 1);
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
		strIdx4 = strTemp.find(")", strIdx3);//ends with closing bracketOA
		radius = stoi(strTemp.substr(strIdx3, strIdx4 - strIdx3)); //cout << strIdx1 << "#" << strIdx2 << "*";
		
		blobOut.x      = (float)x_pos;
		blobOut.y      = (float)y_pos;
		blobOut.radius   = (float)radius;
		blobOut.color = (float)colour;

		n_blobs++;

		// cout << "Peggio: " << blobOut.x << " e " << blobOut.y << " e " << blobOut.radius << " e " << blobOut.color  << endl;

		frameOut.blobs.push_back(blobOut);
		// cout << "blobsize after " << frameOut.blobs.size() << endl;
		//remove one blob information and move to next blob
		strBlobsInfo.erase(0, strIdx2 + 1);
		strIdx1 = strBlobsInfo.find_first_of(openBracket);
		strIdx2 = strBlobsInfo.find_first_of(closeBracket, strIdx1);
		// cout << "prima uscita" << strIdx1  << " e " << strIdx2 << endl;
	      } while (strIdx1 >= 0 && strIdx2 >= 0);
	  }
	// cout << "little framedump. N_blobs: " << n_blobs <<  " FrameID: " << frameOut.frame_id << " TimeStamp: " << frameOut.timeStamp <<  " Blob Size: " << frameOut.blobs.size() << endl;
	pubPtr->publish(frameOut);

      }
    // cout << "fuori dal tunnel" << endl;
    //End of Program
    eBugLogFile.close();


    
}

int main(int argc, char** argv)
{
  int i=0;
  printf("\ncmdline args count=%i", argc);
  
  /*  First argument is executable name only */
  printf("\nexe name=%s", argv[0]);
  
  for (i=1; i<= argc; i++) {
    printf("\narg%d=%s", i, argv[i]);
  }

  if (argc>0) {
    position = argv[1];
  } else {
    position = "~/catkin_ws/src/ebug_swarm_control/src/newlog.txt";
  }

  cout << "Log file position: " << position << endl;
  
  ros::init(argc, argv, "controlnode");
  
  ros::NodeHandle nh;
    
  //     pubPtr = new ros::Publisher(
  //         nh.advertise<sensor_msgs::Image>("imagecontrol",1000));    
  
  pubPtr = new ros::Publisher(
			      nh.advertise<ebug_swarm_msg::frame>("blobs",1000));    
  
  
  
  
  ros::Subscriber sub = nh.subscribe("commands", 1000, &dataReceived);
  ros::spin();
  return 0;
}
