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
* Description: Computes an average localization score of a robut over
*   the course of a single scan.
*********************************************************************/
#include "temporal_navigation/localization_compute.h"



namespace localization_compute
{
LocalizationCompute::LocalizationCompute(ros::NodeHandle& _n) :
  node_(_n)
{
  ROS_INFO("Localization computing now");
  loc_score_sub_ = node_.subscribe("/localization_score", 10, &LocalizationCompute::computeLocalizationScore, this);

  loc_counter_ = 0;

  score_sum_ = 0;

}

LocalizationCompute::~LocalizationCompute()
{
}

void LocalizationCompute::computeLocalizationScore(const std_msgs::Float64::ConstPtr& msg) {
  double score = msg->data;
  // std::cout << msg<< std::endl;
  ROS_INFO_THROTTLE(10,"Score: %f Avg: %f", score, score_sum_/double(loc_counter_));
  loc_counter_++;
  score_sum_ +=score;
}


void LocalizationCompute::exportFinalLocScore(void) {
  std::time_t rawtime;
  struct tm* timeinfo;
  char date[80];
  char time[100];
  char output_date[100];
  char output_file[100];
  char second_file[100];

  // Determines the date
  std::time(&rawtime);
  timeinfo = std::localtime (&rawtime);
  strftime (date, 80, "%Y%m%d",timeinfo);
  strftime (time, 100, "%H%M",timeinfo);
    // Designates a path name based on the current date
  sprintf(output_date, "%s_%s",date,time);
  sprintf(second_file, "%s/data/locScores/localization_%s.txt",ros::package::getPath("temporal_navigation").c_str(),output_date);

  std::ofstream second_filename;

  second_filename.open(second_file);
  second_filename << "Score sum: " << score_sum_ << " Score Count: " << loc_counter_ << " Avg Loc: " << score_sum_/(double)loc_counter_ << std::endl;
  second_filename.close();

  ROS_INFO("Data exported");
  g_shutdown_requested = 1;
}

} // end namespace localization_compute


int main( int argc, char** argv ) {
  ros::init(argc,argv, "localization_compute", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  pmt = new localization_compute::LocalizationCompute(nh);

  // Made my own sigint handler so I can export the map no matter what causes the shutdown
  signal(SIGINT, mySigintHandler);

  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
   
  // Process holds here until ROS is shutdown

  while (!g_shutdown_requested);
  delete pmt;
  ros::shutdown();
  ros::waitForShutdown();  
  

  return 0;
}
  
