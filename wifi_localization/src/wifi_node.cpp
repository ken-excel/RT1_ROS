#include <wifi.h>

using namespace std;

// Kalman filter
//reference https://www.wouterbulten.nl/blog/tech/kalman-filters-explained-removing-noise-from-rssi-signals/		
int R = 1, Q = 3, A = 1, B = 0, C = 1;
//double cov = 0, x = 0;
//bool first_data = true;

void kalman_filter(wifi_nav::RssDatumAvg &rss, double u) 
{
	if (rss.first_data) {
	    rss.x = (1 / C) * rss.rss;
   		rss.cov = (1 / C) * Q * (1 / C);
	    rss.first_data = false;
	}
    else {

      // Compute prediction
      double predX = (A * rss.x) + (B * u);
      double predCov = ((A * rss.cov) * A) + R;

      // Kalman gain
      double K = predCov * C * (1 / ((C * predCov * C) + Q));

      // Correction
      rss.x = predX + K * (rss.rss - (C * predX));
      rss.cov = predCov - (K * C * predCov);
    }
    //return rss;
}
	  

wifiNode::wifiNode():
	nh_("~")
{
	rss_sub_ = nh_.subscribe("/rss", 1, &wifiNode::rssRead, this);
	rss_pub_ = nh_.advertise<wifi_nav::RssAvg>("/rss_avg", 1);
	//Publisher, service, etc.
}

void wifiNode::rssRegis(string addr, float avg, int frequency)
{
	//initialize
	bool newRegis = true; 
	// for (int i = 0; i < data_count; i++)
	// {
	// 	rss_out_.rss[i].rss = NULL;
	// 	rss_out_.rss[i].dist = NULL;	
	// }
	//
	if (data_count != 0){
		for(int i = 0; i < data_count; i++){
			//cout<<addr.c_str()<<" and "<<rss_out_.rss[i].name.c_str()<<endl;
			if(strcmp(addr.c_str(),rss_out_.rss[i].name.c_str()) == 0){	
				//cout<<"---Matching"<<endl;
				rss_out_.rss[i].rss = avg;
				rss_out_.rss[i].timeout = 0;
				rss_out_.rss[i].freq = frequency;
				rss_out_.rss[i].dist = pow(10,((-avg - 20*log10(frequency) + 27.55)/20));
				kalman_filter(rss_out_.rss[i], 0);
				//Data[i].rss = avg;
				newRegis = false;
				break;
			}
		}
	}
	if (newRegis){
		//cout<<"---NewRegis"<<endl;
		wifi_nav::RssDatumAvg new_rss;
		new_rss.id = data_count;
		new_rss.name = addr;
		new_rss.rss = avg;
		new_rss.freq = frequency;
		new_rss.timeout = 0;
		new_rss.first_data = true;
		new_rss.dist = pow(10,((-avg - 20*log10(frequency) + 27.55)/20));
		kalman_filter(new_rss, 0); 
		rss_out_.rss.push_back(new_rss);
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
		cout<<t<<" "<<rss.mac_address[t]<<" "<<rss.freq[t]<<" Signal:";
		int j = rss.data[t].rss.size(); 
		float sum = 0;
		string name = rss.mac_address[t];
		for (int m = 0; m<j; m++){
			//cout<<" "<<int(rss.data[t].rss[m]);
			sum+=float(rss.data[t].rss[m]);
		}
		float avg = sum/j;
		int frequency = rss.freq[t];
		cout<<avg<<endl;
		wifiNode::rssRegis(name,avg,frequency);
	}
	data_ready = true;
}

void wifiNode::process()
{
	rss_out_.header.stamp=ros::Time::now();
	// double elapsed = (rss_out_.header.stamp - begin_time).toSec();
	// outputFile<<elapsed<<",";
	// for (int i = 0; i < rss_out_.rss.size(); i++){
	// 	outputFile<<float(rss_out_.rss[i].rss);
	// 	if (i != rss_out_.rss.size()-1) outputFile<<",";
	// 	else outputFile<<endl;
	// }
	//cout<<"Publishing..."<<""<<endl;

	for (int i = 0; i < rss_out_.rss.size(); i++){
		if (rss_out_.rss[i].timeout > 20){
			rss_out_.rss[i].x = -100; //signal lost
			rss_out_.rss[i].rss = -100;
			rss_out_.rss[i].first_data = true; //reset kalman filter
		}
		rss_out_.rss[i].timeout ++;
	}

//	cout<<rss_out_<<endl;
	rss_pub_.publish(rss_out_);
	data_ready = false;
}

void wifiNode::shutdown()
{
	outputFile<<"Time,";
	for (int i = 0; i < rss_out_.rss.size(); i++){
		outputFile<<rss_out_.rss[i].name;
		if (i != rss_out_.rss.size()-1) outputFile<<",";
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

