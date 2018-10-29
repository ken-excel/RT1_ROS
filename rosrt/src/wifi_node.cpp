#include <wifi.h>

using namespace std;

wifiNode::wifiNode():
	nh_("~")
{
	rss_sub_ = nh_.subscribe("/rss", 1, &wifiNode::rssRead, this);
	//Publisher, service, etc.
}

void wifiNode::rssRegis(string addr, float avg)
{
	bool newRegis = true;
	if (data_count != 0){
		for(int i = 0; i < data_count; i++){
			if(!strcmp(addr.c_str(),Data[i].name.c_str())){
				Data[i].rss = avg;
				newRegis = false;
				break;
			}
		}
	}
	if (newRegis){
		//cout<<"New Wifi Found: "<<addr<<endl;
		Data[data_count].name = addr;
		Data[data_count].rss = avg;
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
		int sum = 0;
		string name = rss.mac_address[t];
		for (int m = 0; m<j; m++){
			//cout<<" "<<int(rss.data[t].rss[m]);
			sum+=int(rss.data[t].rss[m]);
		}
		float avg = sum/j;
		//cout<<avg<<endl;
		//cout<<"Regis"<<name<<" "<<avg<<endl;
		wifiNode::rssRegis(name,avg);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_node");

  wifiNode object;
  ros::Rate rate(20);
  while(ros::ok())
  {
  	cout<<"#Batch====#"<<endl;
  	for(int i = 0; i < data_count; i++){
		cout<<i<<Data[i].name<<Data[i].rss<<endl;
	}
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

