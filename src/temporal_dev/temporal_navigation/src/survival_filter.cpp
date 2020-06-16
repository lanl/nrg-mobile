/*********************************************************************
*
*  © (or copyright) 2020. Triad National Security, LLC.
*  All rights reserved.
*  This program was produced under U.S. Government contract 
*  89233218CNA000001 for Los AlamosNational Laboratory (LANL), 
*  which is operated by Triad National Security, LLC for the U.S.
*  Department of Energy/National Nuclear Security Administration. 
*  All rights in the program are reserved by Triad National 
*  Security, LLC, and the U.S. Department of Energy/National Nuclear
*  Security Administration. The Government is granted for itself 
*  and others acting on its behalf a nonexclusive, paid-up, 
*  irrevocable worldwide license in this material to reproduce, 
*  prepare derivative works, distribute copies to the public, 
*  perform publicly and display publicly, and to permit others 
*  to do so.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Meredith Symmank
*
* Description: Predicts which parts of the environment will survive 
*   to the next timestep and updates prior beliefs and likelihoods
*********************************************************************/

#include "temporal_navigation/survival_filter.h"


namespace survival_filter
{
SurvivalFilter::SurvivalFilter(ros::NodeHandle& _n) : 
	node_(_n)
{
	ROS_INFO("Starting survival filter");
	init_filter_svr_ = node_.advertiseService("/survival_filter/init_filter", &SurvivalFilter::initFilter,this);
	survival_filter_svr_ = node_.advertiseService("/survival_filter/run_filter", &SurvivalFilter::runFilter,this);
	temp_day_ = 0;
	while(ros::ok());
    ROS_INFO_THROTTLE(10,"Process Complete. Waiting on shutdown.");
}; // default constructor

SurvivalFilter::~SurvivalFilter() {}; // default destructor	


bool SurvivalFilter::initFilter(temporal_navigation::InitFilter::Request& request, temporal_navigation::InitFilter::Response& response) { 
	int n_prob = request.n_prob;
	int n_scan = request.n_scan;
	n_states_ = n_prob + n_scan;


	p_evidence_old_.clear();
    p_like_old_.clear();
    L_y_old_.clear() ;
    prior_old_vec_.clear();
    N_vals_.clear();
	p_like_vec_.clear(); 
	L_y_vec_.clear(); 
	p_evidence_vec_.clear(); 
	prior_vec_.clear(); 
	
	ROS_INFO("Reading in values");
	ROS_INFO("%d old prob points and %d observed scan points for %d total size",n_prob,n_scan,n_states_);
	readData(n_scan, n_prob);
    // ROS_INFO("L_old 2: %f", L_y_old_.at(0));
	previous_number_ = p_evidence_old_.size();
	ROS_INFO("Previously %d points", previous_number_);
	
	// ROS_INFO("3");
	return true;

};

bool SurvivalFilter::runFilter(temporal_navigation::FilterData::Request& request
	, temporal_navigation::FilterData::Response& response) {
	std::vector<temporal_navigation::OccuPoint> sensor_data = request.sensor_data;
    std::vector<temporal_navigation::ProbPoint> posterior_vec;
	float p_like_new; // likelihoods for all points in sensor data
	float L_y_new; // lower partial evidences for all points in sensor data (might not need to keep track of this)
	float p_evidence_new; // evidences for all points in the sensor data
	temporal_navigation::ProbPoint posterior_new;
	int index = 0;
	// ROS_INFO("Testing %d", L_y_old_.size());
    // ROS_INFO("L_old 2: %f", L_y_old_.at(0));
	for (auto& s : sensor_data) {
		// ROS_INFO("y at %d [%f,%f]: %f",index, s.x, s.y, s.occupancy);
		t_new_ = s.time;
		int persist_val = request.persist_vec.at(index);
		if (persist_val > 30.0) {
			p_evidence_old_.at(index) = 0.99;
			p_like_old_.at(index) = 0.99;
			L_y_old_.at(index) = 0.0;
		}
		calcPrior();
		calcPartEvidence(index,s,L_y_new);
		// ROS_INFO("4");
		calcLikelihood(index,s,p_like_new);
		// ROS_INFO("5");
		calcEvidence(index, p_evidence_new);
		// ROS_INFO("6");
		predictPosterior(index, p_evidence_new, p_like_new, L_y_new, posterior_new, s);
		// ROS_INFO("7");
		if (p_evidence_new < 0.0005 && persist_val > 1.0) p_evidence_new = 0.0;
		if (p_like_new < 0.0005 && persist_val > 1.0) p_like_new = 0.0;
		if (L_y_new < 0.0005 && persist_val > 1.0) L_y_new = 0.0;
		if (prior_new_ < 0.0005 && persist_val > 1.0) prior_new_ = 0.0;

		p_evidence_vec_.push_back(p_evidence_new);
		p_like_vec_.push_back(p_like_new);
		L_y_vec_.push_back(L_y_new);
		prior_vec_.push_back(prior_new_);
		posterior_vec.push_back(posterior_new);
		index++;
	}
	packValues(posterior_vec, request.persist_vec);
	// ROS_INFO("8");
	response.posterior = posterior_vec;
	ROS_INFO("Returning %d posterior points",response.posterior.size());
	temp_day_++;
	return true;
}

void SurvivalFilter::readData(int n_scan, int n_prob) {
	std::vector<float> zeroes_vec(n_states_,0.0);
	std::vector<float> ones_vec(n_states_,1.0);
	std::vector<int> zeros_int(n_states_,0);
    char dir_name[150];
  	sprintf(dir_name, "%s/data/survival_values",ros::package::getPath("temporal_navigation").c_str());
	// ROS_INFO("A1");
  	std::string dir = dir_name;
	DIR *dp;
    struct dirent *dirp;
    struct dirent *temp_dirp;
	// ROS_INFO("A2");
    if((dp  = opendir(dir.c_str())) == NULL) {
		// ROS_INFO("A3");
    	ROS_ERROR("No such directory");
    	// Default start values
        p_evidence_old_ = ones_vec;
        p_like_old_ = ones_vec;
        L_y_old_ = zeroes_vec;
        N_vals_ = zeros_int;
        prior_old_vec_ = ones_vec;
        return;
    }
	// ROS_INFO("A4");
    std::string filename;
    int file_count = 0;
    char list_dir[100]; 
    char current_latest[100] = "";
    struct dirent *latest_dirp = dirp;
    std::vector<std::string> tired_of_this;
    while ((dirp = readdir(dp)) != NULL) {
    	sprintf(list_dir,"%s",dirp->d_name);
    	if (strcmp(list_dir, "bad_dir")==0){continue;}
    	else if(strcmp(list_dir, ".")==0){continue;}
    	else if(strcmp(list_dir, "..")==0){continue;}
        else {
        	tired_of_this.push_back(std::string(list_dir));
        	temp_dirp = dirp;  //should get the last file in the directory if all goes well
        	file_count++;
        }
    }
    
    // ROS_INFO("File path:%s", file_path.c_str());
	// ROS_INFO("A5");
    if (file_count > 0) {
    	std::vector<std::string>::iterator result = std::max_element(tired_of_this.begin(), tired_of_this.end());
    	int index = std::distance(tired_of_this.begin(),result);
    	std::string file_path = tired_of_this.at(index);
	    filename = dir + "/" + file_path.c_str();
		ROS_INFO("Filename: %s",filename.c_str());
	    closedir(dp);

	    std::ifstream file;
	    std::string line;
	    std::string value;
	    std::vector<float> values_vec;
	    // ROS_INFO("A5.5");
	  	file.open(filename.c_str());
	  	// ROS_INFO("A5.9");
	  	if (file.is_open()) {
		  // ROS_INFO("A6");
		  while (std::getline(file,line)) {
		  	std::istringstream iss(line);
      		std::string value;
      		while (iss >> value) {
		      values_vec.push_back(std::stof(value));
		  	}
		  	p_evidence_old_.push_back(values_vec.at(0));
		  	p_like_old_.push_back(values_vec.at(1));
		  	L_y_old_.push_back(values_vec.at(2));
		  	// ROS_INFO("L_old val: %f", values_vec.at(2));
		  	N_vals_.push_back(values_vec.at(3)+1);
		  	prior_old_vec_.push_back(values_vec.at(4));
		  	values_vec.clear();
		  }
		  file.close();	
	   }
	} else {
		// ROS_INFO("A7");
		p_evidence_old_ = ones_vec;
		p_like_old_ = ones_vec;
		L_y_old_ = zeroes_vec;
		N_vals_ = zeros_int;
		prior_old_vec_ = ones_vec;
	}
	int N_val = N_vals_.front();

	// ROS_INFO("A8");
	while (n_states_ > N_vals_.size()) {
		// ROS_INFO("A9");
		p_evidence_old_.push_back(0.1);
		p_like_old_.push_back(0.01);
		L_y_old_.push_back(0.0);
		N_vals_.push_back(N_val);
		prior_old_vec_.push_back(1.0);
	}

}; // Variables that we could expect to know if we ran this in a continuous demo

void SurvivalFilter::calcPrior() {
	float E1_u = calcExponentIntegral(lambda_high_*t_new_);
	float E1_l = calcExponentIntegral(lambda_low_*t_new_);
	float lower = std::log(lambda_high_/lambda_low_);
	float St = (E1_l-E1_u)/lower;
	prior_new_ = St;

	// ROS_INFO("E1_u: %f E1_l: %f lower: %f t: %f prior: %f", E1_u,E1_l,lower,t_new_,prior_new_);
}; // 

float SurvivalFilter::calcExponentIntegral(float x) {  
	return x;
}

void SurvivalFilter::calcPartEvidence(int index, temporal_navigation::OccuPoint& sensor_data, float& L_y_new) {
	float F_new = 1-prior_new_;
	float F_old = 1-prior_old_vec_.at(index);
	// t_new_++;
	// calcPrior();
	// float F_new = 1-prior_new_;
	// t_new_--;
	// calcPrior();
	// float F_old = 1-prior_new_;
	L_y_new = std::pow(sensor_PF_,sensor_data.occupancy)*std::pow((1.0-sensor_PF_),1.0-(sensor_data.occupancy)) * (L_y_old_.at(index) + (p_like_old_.at(index)) * (F_new-F_old));
	// ROS_INFO("L val: %f, Old L val: %f, occ: %f F_new: %f F_old: %f p(y|t)old: %f", L_y_new,L_y_old_.at(index),sensor_data.occupancy, F_new, F_old, p_like_old_.at(index));
	
	// Happens when a point goes out of probability and the timer resets
	if (L_y_new < 0) 
		L_y_new = 0.0;
	
	// ROS_INFO("Y count: %d Total size: %d", count, n_states);
}; // to get L_y_new needs sensor_PF_ sensor_data L_y_old_ p_like_old_ F_new F_old

void SurvivalFilter::calcLikelihood(int index, temporal_navigation::OccuPoint& sensor_data,float& p_like_new) {
	p_like_new = std::pow(sensor_PM_,1-sensor_data.occupancy)*std::pow((1-sensor_PM_),sensor_data.occupancy)*p_like_old_.at(index);
}; // to get p_like_new needs sensor_PM_ sensor_data p_like_old_

void SurvivalFilter::calcEvidence(int index, float& p_evidence_new) {
	p_evidence_new = L_y_old_.at(index) + p_like_old_.at(index)*prior_old_vec_.at(index);
	// ROS_INFO("P evidence: %f", p_evidence_new);
}; // to get p_evidence_new use 

void SurvivalFilter::predictPosterior(int index, float& p_evidence_new, float& p_like_new, 
	                                  float& L_y_new, temporal_navigation::ProbPoint& posterior_new,
	                                  temporal_navigation::OccuPoint& sensor_data) {
	float posterior_prob;
	float posterior_x;
	float posterior_y;
	float posterior_z;
	temporal_navigation::ProbPoint temp_post;
	posterior_x = sensor_data.x;
	posterior_y = sensor_data.y;
	posterior_z = sensor_data.z;
	if (p_evidence_old_.at(index) == 0) {
		posterior_prob = 0.0;
	} else {
		posterior_prob = (p_like_new*prior_new_/p_evidence_new);
	}
	// ROS_INFO("Index: %d Y: %f Posterior: %f likelihood: %f prior: %f evidence: %f",index,sensor_data.occupancy,posterior_prob,p_like_old_.at(index),prior_old_,p_evidence_old_.at(index));
	temp_post.loc.x = posterior_x;
	temp_post.loc.y = posterior_y;
	temp_post.loc.z = posterior_z;
	temp_post.probability = posterior_prob;
	temp_post.timer = int(t_new_ - 1.0);
	temp_post.persistence =-1;
	posterior_new = temp_post;
}

void SurvivalFilter::packValues(std::vector<temporal_navigation::ProbPoint>& posterior, std::vector<int>& persist_vec) {
  // std::time_t rawtime;
  // struct tm* timeinfo;
  // char date[80];
  // char time[100];
  // char output_date[100];
  char output_file[150];
  char output_file2[150];
  std::vector<temporal_navigation::ProbPoint> second_post_vec;
  // Determines the date
  // std::time(&rawtime);
  // timeinfo = std::localtime (&rawtime);
  // strftime (date, 80, "%Y%m%d",timeinfo);
  // strftime (time, 100, "%H%M",timeinfo);
  //   // Designates a path name based on the current date
  // sprintf(output_date, "%s_%s_%d",date,time,N_vals_.at(0));
  sprintf(output_file, "%s/data/survival_values/scan_%d.txt",ros::package::getPath("temporal_navigation").c_str(),temp_day_);
  sprintf(output_file2, "%s/data/survival_values/aascan_details_%d.txt",ros::package::getPath("temporal_navigation").c_str(),temp_day_);

  ROS_INFO("Exporting survival data of size: %lu to %s", p_like_vec_.size(), output_file);
  std::ofstream filename;
  std::ofstream filename2;

  filename.open(output_file);
  filename2.open(output_file2);
  int check_counter = 0;
  ROS_INFO("N states: %d vs number of posterior pts: %d", n_states_, posterior.size());
  for(int i = 0; i < n_states_; i++) {
 	  	// ROS_INFO("P 2 evidence: %f", p_evidence_vec_.at(i));
  	char output_line[150];
  	if (i == previous_number_) {
  		//debug case
 		sprintf(output_line,"\n\nNew data set");
    	filename2 << output_line << std::endl;
  	}
  	else if (posterior.at(i).probability > 0.000005){
 		sprintf(output_line,"N=>(x,y,z): %d=>(%2.2f, %2.2f, %2.2f) seen for %d/not seen for t= %d\n\t\tP(Y):%f P(Y|T): %f Ly: %f P(X): %f P(X|T): %f",
  			N_vals_.at(i), posterior.at(i).loc.x, posterior.at(i).loc.y, posterior.at(i).loc.z,persist_vec.at(i), posterior.at(i).timer,
  			p_evidence_vec_.at(i), p_like_vec_.at(i), L_y_vec_.at(i), prior_vec_.at(i), posterior.at(i).probability);
    	filename2 << output_line << std::endl;
    	filename << p_evidence_vec_.at(i) << ", " << p_like_vec_.at(i) << ", " << L_y_vec_.at(i) << ", " << N_vals_.at(i) << ", " << prior_vec_.at(i) << ", " << posterior.at(i).probability << std::endl;
    	check_counter++;
    	posterior.at(i).persistence = persist_vec.at(i);
    	second_post_vec.push_back(posterior.at(i));
  	} 
  }
  filename2.close();
  filename.close();
  posterior.clear();
  for (auto& pt : second_post_vec) {
  	posterior.push_back(pt);
  }

  ROS_INFO("Data exported: %d", check_counter);
}

}; // end namespace

int main( int argc, char** argv ) {
  ros::init(argc,argv, "survival_filter");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  survival_filter::SurvivalFilter* sf = new survival_filter::SurvivalFilter(nh);


  // Process holds here until ROS is shutdown

  ros::shutdown();
  ros::waitForShutdown();  
  delete sf;
  

  return 0;
}
