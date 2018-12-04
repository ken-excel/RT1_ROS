#include <wifi.h>

using namespace std;

wifiNode::wifiNode():
	nh_("~")
{
	rss_sub_ = nh_.subscribe("/rss", 1, &wifiNode::rssRead, this);
	rss_pub_ = nh_.advertise<ros_start::RssAvg>("/rss_avg", 1);
	//Publisher, service, etc.
}

void wifiNode::rssRegis(string addr, float avg, int frequency)
{
	//initialize
	bool newRegis = true; 
	for (int i = 0; i < data_count; i++)
	{
		db_out_.rss[i].rss = NULL;
		db_out_.rss[i].dist = NULL;	
	}
	//
	if (data_count != 0){
		for(int i = 0; i < data_count; i++){
			//cout<<addr.c_str()<<" and "<<rss_out_.rss[i].name.c_str()<<endl;
			if(strcmp(addr.c_str(),rss_out_.rss[i].name.c_str()) == 0){	
				//cout<<"---Matching";
				rss_out_.rss[i].rss = avg;
				rss_out_.rss[i].freq = frequency;
				rss_out_.rss[i].dist = pow(10,((-avg - 20*log10(frequency) + 27.55)/20));
				db_out_.rss[i].rss = avg;
				newRegis = false;
				break;
			}
		}
	}
	if (newRegis){
		ros_start::RssDatumAvg new_rss;
		new_rss.id = data_count;
		new_rss.name = addr;
		new_rss.rss = avg;
		new_rss.freq = frequency;
		new_rss.dist = pow(10,((-avg - 20*log10(frequency) + 27.55)/20));
		rss_out_.rss.push_back(new_rss);
		db_out_.rss.push_back(new_rss);
		data_count++;
	}
	else{
		//intentional blank
	}
}

void wifiNode::rssRead(const rss::RssData &rss)
{
	int i = rss.mac_address.size();
	for (int t = 0; t<i; t++){
		//cout<<t<<" "<<rss.mac_address[t]<<" "<<rss.freq[t]<<" Signal:";
		int j = rss.data[t].rss.size(); 
		float sum = 0;
		string name = rss.mac_address[t];
		for (int m = 0; m<j; m++){
			//cout<<" "<<int(rss.data[t].rss[m]);
			sum+=float(rss.data[t].rss[m]);
		}
		float avg = sum/j;
		int frequency = rss.freq[t];
		//cout<<"Sending Wifi:"<<name<<endl;
		wifiNode::rssRegis(name,avg,frequency);
	}
	data_ready = true;
}

void wifiNode::process()
{
	rss_out_.header.stamp=ros::Time::now();
	double elapsed = (rss_out_.header.stamp - begin_time).toSec();
	if (elapsed < 10000){
		outputFile<<elapsed<<",";
		for (int i = 0; i < db_out_.rss.size(); i++){
			outputFile<<float(db_out_.rss[i].rss);
			if (i != db_out_.rss.size()-1) outputFile<<",";
			else outputFile<<endl;
		}
	}
	cout<<"Publishing..."<<""<<endl;
	rss_pub_.publish(rss_out_);
	data_ready = false;
}

void wifiNode::shutdown()
{
	outputFile<<"Time,";
	for (int i = 0; i < db_out_.rss.size(); i++){
		outputFile<<db_out_.rss[i].name;
		if (i != db_out_.rss.size()-1) outputFile<<",";
		else outputFile<<endl;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_node");

  wifiNode object;
  ros::Rate rate(20);
  begin_time = ros::Time::now();
  if (argc == 2)
  {
  	char* extension = ".txt";
  	char* loc_name = argv[1];
  	char* subfolder = "./data/";
  	char filename[strlen(subfolder)+strlen(loc_name)+strlen(extension)+1];
  	snprintf( filename, sizeof( filename ), "%s%s%s", subfolder, loc_name, extension );
  	outputFile.open (filename);
  	outputFile<<"Location: "<<argv[1]<<endl;
  }
  else outputFile.open("./data/log_raw_rss.txt");
  while(ros::ok())
  {
  	if(data_ready) object.process();
    ros::spinOnce();
    rate.sleep();
  }
  object.shutdown();
  outputFile.close();
  return 0;
}

