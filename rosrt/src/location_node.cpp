#include<location.h>

using namespace std;

locationNode::locationNode():
	nh_("~")
{
	rss_avg_sub_ = nh_.subscribe("/rss_avg", 1, &locationNode::rssRead, this);
	rss_loc_pub_ = nh_.advertise<ros_start::PointRss>("/rss_loc", 1);
}

void locationNode::rssRead(const ros_start::RssAvg &rss) //Read, Find Avg. Send Top 5 Candidates
{
	wifiCount = rss.rss.size()-1;
	for (int i = 0; i<wifiCount; i++)
	{
		rss_temp_.resize(wifiCount);
		//Filter, cut off ?
		if (rss.rss[i].rss != 0){
			rss_temp_[i].name = rss.rss[i].name;
			rss_temp_[i].sum_rss -= rss.rss[i].rss;
			rss_temp_[i].sum_dist += rss.rss[i].dist;
			rss_temp_[i].count++;
			locationNode::findAverage(rss_temp_[i]);
		}
		else{
			
		}
		rss_arr_.resize(wifiCount);
		rss_arr_[i] = rss_temp_[i];
	}
	//memcpy(&rss_arr_, &rss_temp_, sizeof(rss_temp_));
	for (int i = 0; i < wifiCount; i++)                     //Loop for ascending ordering
	{
		for (int j = 0; j < wifiCount; j++)             //Loop for comparing other values
		{
			if (rss_arr_[j].mean_rss > rss_arr_[i].mean_rss)                //Comparing other array elements
			{	
				rssData tmp = rss_arr_[i];         //Using temporary variable for storing last value
				rss_arr_[i] = rss_arr_[j];            //replacing value
				rss_arr_[j] = tmp;             //storing last value
			} 
		}
	}
	ready = true;
}

void locationNode::findAverage(rssData &rss_in_) //find all average in location msg in this function
{
	rss_in_.mean_rss = float(rss_in_.sum_rss)/float(rss_in_.count);
	rss_in_.mean_dist = float(rss_in_.sum_dist)/float(rss_in_.count);
}

void locationNode::process()
{
	ros_start::PointRss location; 
	location.header.stamp=ros::Time::now();
	location.location = loc_name;
	double elapsed = (location.header.stamp - begin_time).toSec();
	// if (first_write){
	// 	fprintf(pFile, "time,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",rss_arr_[0].name.c_str(),rss_arr_[1].name.c_str(),rss_arr_[2].name.c_str(),rss_arr_[3].name.c_str(),rss_arr_[4].name.c_str(),rss_arr_[5].name.c_str(),rss_arr_[6].name.c_str(),rss_arr_[7].name.c_str(),rss_arr_[8].name.c_str(),rss_arr_[9].name.c_str());
	// 	fprintf(qFile, "time,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",rss_arr_[0].name.c_str(),rss_arr_[1].name.c_str(),rss_arr_[2].name.c_str(),rss_arr_[3].name.c_str(),rss_arr_[4].name.c_str(),rss_arr_[5].name.c_str(),rss_arr_[6].name.c_str(),rss_arr_[7].name.c_str(),rss_arr_[8].name.c_str(),rss_arr_[9].name.c_str());
	// 	first_write = false;
	// 	cout<<rss_arr_[0].name<<endl;
	// }
	if (!writing_finish){
	if (elapsed < 1000)
		{
			fprintf(pFile, "%f,",elapsed);
			fprintf(qFile, "%f,",elapsed);
			for(int i=0; i<wifiCount; i++){
				location.ssid.push_back(rss_arr_[i].name);
				location.mean_rss.push_back(rss_arr_[i].mean_rss);
				location.mean_dist.push_back(rss_arr_[i].mean_dist);
				fprintf(pFile, "%f",rss_arr_[i].mean_dist);
				fprintf(qFile, "%f",rss_arr_[i].mean_rss);
				if (i != wifiCount-1){
					fprintf(pFile, ",");
					fprintf(qFile, ",");
				}
			}
			fprintf(pFile, "\n");
			fprintf(qFile, "\n");
		}
		else{
			cout << "finish writing" << endl;
			fprintf(pFile, "time,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",rss_arr_[0].name.c_str(),rss_arr_[1].name.c_str(),rss_arr_[2].name.c_str(),rss_arr_[3].name.c_str(),rss_arr_[4].name.c_str(),rss_arr_[5].name.c_str(),rss_arr_[6].name.c_str(),rss_arr_[7].name.c_str(),rss_arr_[8].name.c_str(),rss_arr_[9].name.c_str());
			fprintf(qFile, "time,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",rss_arr_[0].name.c_str(),rss_arr_[1].name.c_str(),rss_arr_[2].name.c_str(),rss_arr_[3].name.c_str(),rss_arr_[4].name.c_str(),rss_arr_[5].name.c_str(),rss_arr_[6].name.c_str(),rss_arr_[7].name.c_str(),rss_arr_[8].name.c_str(),rss_arr_[9].name.c_str());
		 	fclose (pFile);
	 	  	fclose (qFile);
			writing_finish = true;
		}
	}
	rss_loc_pub_.publish(location);
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "location_node");

  //cout << "Input location name :";
  //cin >> location_name;
  //Ask for location input e.g. name ---HERE
  if (argc == 2)
  {
  	char* filename = argv[1];
  	char* extension = ".txt";
  	char* distname = "dist";
  	char* rssname = "rss";
  	char* subfolder = "./data/";
  	char distfile[strlen(subfolder)+strlen(filename)+strlen(extension)+strlen(distname)+1];
  	char rssfile[strlen(subfolder)+strlen(filename)+strlen(extension)+strlen(rssname)+1];
  	snprintf( distfile, sizeof( distfile ), "%s%s%s%s",subfolder, filename, distname, extension );
  	snprintf( rssfile, sizeof( rssfile ), "%s%s%s%s",subfolder, filename, rssname, extension );
  	loc_name = argv[1];
  	pFile = fopen(distfile,"w");
  	qFile = fopen(rssfile,"w");
  }
  else{
  pFile = fopen("./data/log_dist.txt","w");
  qFile = fopen("./data/log_rss.txt","w");
  }
  locationNode object;
  ros::Rate rate(20);
  begin_time = ros::Time::now();
  while(ros::ok())
  {
  	if(ready){
  	object.process();
    ready = false;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

