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

#ifndef SURVIVAL_FILTER_H
#define SURVIVAL_FILTER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>

#include <sensor_msgs/PointCloud.h>

#include <signal.h>
#include <cmath>
#include <ctime>
#include <numeric>
#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <temporal_navigation/FilterData.h>
#include <temporal_navigation/InitFilter.h>
#include <temporal_navigation/OccuPoint.h>
#include <geometry_msgs/Point.h>


namespace survival_filter
{
class SurvivalFilter {
public:
	SurvivalFilter(ros::NodeHandle& _n); // default constructor
	~SurvivalFilter(); // default destructor
private:	
	bool initFilter(temporal_navigation::InitFilter::Request& request, temporal_navigation::InitFilter::Response& response);
	void readData(int n_scan, int n_prob); // Variables are all members
	void calcPrior(void);
	float calcExponentIntegral(float x);
	bool runFilter(temporal_navigation::FilterData::Request& request, temporal_navigation::FilterData::Response& response);
	void calcPartEvidence(int index, temporal_navigation::OccuPoint& sensor_data, float& L_y_new); // to get L_y_new needs sensor_PF_ y_new L_y_old p_like_old F_new F_old
	void calcLikelihood(int index, temporal_navigation::OccuPoint& sensor_data,  float& p_like_new) ; // to get p_like_new needs Pm y_new p_like_old
	void calcEvidence(int index, float& p_evidence_new); // to get p_evidence_new use 
	void predictPosterior(int index, float& p_evidence_new, float& p_like_new, float& L_y_new, temporal_navigation::ProbPoint& posterior_new, temporal_navigation::OccuPoint& sensor_data);
	void packValues(std::vector<temporal_navigation::ProbPoint>& posterior, std::vector<int>& persist_vec);


	const float sensor_PM_ = 0.1; // TODO: make a ros dynamic param
	const float sensor_PF_ = 0.1; // TODO: make a ros dynamic param
	const float lambda_low_ = 0.01; // TODO: make a ros dynamic param Default 0.01
	const float lambda_high_ = 1.0; // TODO: make a ros dynamic param Default 10.0

	std::vector<float> p_evidence_old_;
    std::vector<float> p_like_old_;
    std::vector<float> L_y_old_ ;
    std::vector<float> prior_old_vec_;
    std::vector<int> N_vals_;
    

    float prior_new_;
    float t_new_;
    int n_states_;
    int temp_day_;
    int previous_number_;


	std::vector<float> p_like_vec_; // likelihoods for all points in sensor data
	std::vector<float> L_y_vec_; // lower partial evidences for all points in sensor data (might not need to keep track of this)
	std::vector<float> p_evidence_vec_; // evidences for all points in the sensor data
	std::vector<float> prior_vec_; // priors for all points in the sensor data

    char output_path_[100];

	// Node handle
 	ros::NodeHandle& node_;
	ros::ServiceServer survival_filter_svr_;
	ros::ServiceServer init_filter_svr_;

};

}

#endif
