#include <locfind.h>

using namespace std;

bool publish = false;
int instance = 0;

locationFind::locationFind():
	nh_("~")
{
	rss_sub_avg_ = nh_.subscribe("/rss_avg", 1, &locationFind::rssRead, this);
	rss_sub_loc_ = nh_.subscribe("/rss_loc", 1, &locationFind::rssStaticRead, this);
}

void locationFind::rssRead(const wifi_nav::RssAvg &rss)
{
	if (mode == "moving")
	{
		cout<<mode<<endl;
		cout<<instance<<endl;
		if(instance<100) outputFile<<instance<<endl;
		for(int i=0; i<db.size(); i++){
			db[i].rms = locationFind::rssComp(db[i],rss);
			cout<<db[i].loc<<db[i].rms<<endl;
			if (instance<100)outputFile<<db[i].loc<<db[i].rms<<endl;
		}
		double minimum = db[0].rms;
		int db_min = 0;
		for(int i=0; i<db.size(); i++){
			if(db[i].rms < minimum){
				minimum = db[i].rms;
				db_min = i;
			}
				
		}
		if(publish){
			db[db_min].count++;
			cout << "Location:" << db[db_min].loc << endl << endl;
			if (instance<100)outputFile << "Location:" << db[db_min].loc << endl << endl;
			publish = false;
			instance++;
		}
		if (instance == 100){
			cout << "Result (" << "From " << instance << " tests" << ")" << endl;
			for(int i=0; i<db.size(); i++){
				cout<<db[i].loc<<" Suggested time:"<<db[i].count << " times"<<endl;
				outputFile<<db[i].loc<< "Suggested time: "<<db[i].count << " times"<<endl;
			}
		}
	}
	else{
		//do nothing
	}
}

void locationFind::rssStaticRead(const wifi_nav::PointRss &rss_loc)
{
	if (mode == "static")
	{
		cout<<mode<<endl;
		cout<<instance<<endl;
		if(instance<100) outputFile<<instance<<endl;
		for(int i=0; i<db.size(); i++){
			db[i].rms = locationFind::rssStaticComp(db[i],rss_loc);
			cout<<db[i].loc<<db[i].rms<<endl;
			if (instance<100)outputFile<<db[i].loc<<db[i].rms<<endl;
		}
		double minimum = db[0].rms;
		int db_min = 0;
		for(int i=0; i<db.size(); i++){
			if(db[i].rms < minimum){
				minimum = db[i].rms;
				db_min = i;
			}
				
		}
		if(publish){
			db[db_min].count++;
			cout << "Location:" << db[db_min].loc << endl << endl;
			if (instance<100)outputFile << "Location:" << db[db_min].loc << endl << endl;
			publish = false;
			instance++;
		}
		if (instance == 100){
			cout << "Result (" << "From " << instance << " tests" << ")" << endl;
			for(int i=0; i<db.size(); i++){
				cout<<db[i].loc<<" Suggested time:"<<db[i].count << " times"<<endl;
				outputFile<<db[i].loc<< "Suggested time: "<<db[i].count << " times"<<endl;
			}
		}
	}
	else{
		//do nothing
	}
}

double locationFind::rssComp(database rss_db, wifi_nav::RssAvg rss_in)
{
	bool matching = false;
	double root;
	double sum;
	double rms;
	int count;
	for (int i=0; i<rss_db.ssid.size(); i++)
	{
		int j;
		for(j=0; j<rss_in.rss.size(); j++)
		{
			if(strcmp(rss_in.rss[j].name.c_str(),rss_db.ssid[i].c_str()) == 0){
				matching = true;
				break;
			} 
		}
		if (matching)
		{
			if (criteria == "rss") root = pow(rss_in.rss[j].rss+rss_db.rss[i],2);
			else if (criteria == "dist") root = pow(rss_in.rss[j].dist-rss_db.rss[i],2);
			else{
				cout << "Criteria not found" << endl;
				return 1;
			}
		}
		else
		{
			if (criteria == "rss") root = pow(rss_db.rss[i]+min_rss,2);
			else if (criteria == "dist") root = pow(rss_db.rss[i]-min_dist,2);
			else{
				cout << "Criteria not found" << endl;
				return 1;
			}
		}
		sum += root;
		count++;
	}
	double avg = sum/count;
	rms = sqrt(avg);
	return rms;
}

double locationFind::rssStaticComp(database rss_db, wifi_nav::PointRss rss_in)
{
	bool matching = false;
	double root;
	double sum;
	double rms;
	int count;
	for (int i=0; i<rss_db.ssid.size(); i++)
	{
		int j;
		for(j=0; j<rss_in.ssid.size(); j++)
		{
			if(strcmp(rss_in.ssid[j].c_str(),rss_db.ssid[i].c_str()) == 0){
				matching = true;
				break;
			} 
		}
		if (matching)
		{
			if (criteria == "rss") root = pow(rss_in.mean_rss[j]-rss_db.rss[i],2);
			else if (criteria == "dist") root = pow(rss_in.mean_dist[j]-rss_db.rss[i],2);
			else{
				cout << "Criteria not found" << endl;
				return 1;
			}
		}
		else
		{
			if (criteria == "rss") root = pow(rss_db.rss[i]-min_rss,2);
			else if (criteria == "dist") root = pow(rss_db.rss[i]-min_dist,2);
			else{
				cout << "Criteria not found" << endl;
				return 1;
			}
		}
		sum += root;
		count++;
	}
	double avg = sum/count;
	rms = sqrt(avg);
	return rms;
}

void locationFind::readDatabase(istream &input)
{
	string line;
	int count = 1;
	database dbTemp;
	string locTemp;
	string ssidTemp;
	string rssTemp;
	while(getline(input, line)){
		stringstream ss(line);
		cout<<count<<">>"<<line<<endl;
		if(count ==1){
			getline(ss, locTemp, '\n');
			cout<<locTemp<<endl;
			dbTemp.loc = locTemp.c_str();
		}
		else if (count ==2){
			while(getline(ss, ssidTemp, ',')){
				cout<<ssidTemp<<endl;
				dbTemp.ssid.push_back(ssidTemp.c_str());
			}
		}
		else if (count ==3){
			while(getline(ss, rssTemp, ',')){
				cout<<rssTemp<<endl;
				dbTemp.rss.push_back(atof(rssTemp.c_str()));
			}
		}
		if(count==3){
			db.push_back(dbTemp);
			count = 1;
			dbTemp.ssid.clear();
			dbTemp.rss.clear();
		}
		else count++;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "location_find");

  locationFind object;
  object.readDatabase(dbFile);
  if (argc == 4)
  {
  	mode = argv[2];	
  	criteria = argv[3];
  	char* extension = ".txt";
  	char* loc_name = argv[1];
  	char* exp_type = argv[2];
  	char* cri_type = argv[3];
  	char* subfolder = "./result/";
  	char filename[strlen(subfolder)+strlen(loc_name)+strlen(exp_type)+strlen(cri_type)+strlen(extension)+3];
  	snprintf( filename, sizeof( filename ), "%s%s_%s_%s%s", subfolder, loc_name, exp_type, cri_type, extension );
  	outputFile.open (filename);
  	outputFile<<"Location: "<<argv[1]<<endl;
  	outputFile<<"Experiment Type: "<<argv[2]<<endl;
  	outputFile<<"Criteria Type: "<<argv[3]<<endl<<endl;
  }
  else {
  	outputFile.open ("result_default.txt");
  	mode = "moving";
  	criteria = "rss";
  }
  cout<<mode<<criteria<<endl;
  ros::Rate rate(0.5);
  while(ros::ok())
  {
  	publish = true;
    ros::spinOnce();
    rate.sleep();
  }
  outputFile.close();
  return 0;
}




