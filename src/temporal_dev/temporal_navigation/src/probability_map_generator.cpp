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
* Description: Captures a full point map of an area after a scan, 
*   filtering out duplicate observations and noise. 
*********************************************************************/

#include "temporal_navigation/probability_map_generator.h"



namespace probability_map_generator
{
ProbabilityMapGenerator::ProbabilityMapGenerator(ros::NodeHandle& _n) :
  node_(_n)
{
  pointcloud_sub_ = node_.subscribe("/rosarnl_node/S3Series_1_pointcloud", 1, (boost::function <void(const sensor_msgs::PointCloudConstPtr&)>) boost::bind(&ProbabilityMapGenerator::pointCloudCB, this, _1));
  // loc_score_sub_ = node_.subscribe<std_msgs::Float64>("/rosarnl_node/localization_score", 1, &ProbabilityMapGenerator::computeLocalizationScore, this);
  point_map_pub_ = node_.advertise<sensor_msgs::PointCloud>("/probability_map_generator/point_map", 1, true);
  timer_ = node_.createTimer(ros::Duration(2), (boost::function <void(const ros::TimerEvent&)>) boost::bind(&ProbabilityMapGenerator::timerCB, this, _1));
  message_throttle_ = 0;
  loc_counter_ = 0;
  score_sum_ = 0;
}

ProbabilityMapGenerator::~ProbabilityMapGenerator()
{
}

// void ProbabilityMapGenerator::computeLocalizationScore(std_msgs::Float64 msg) {
//   double score = msg.data;
//   ROS_INFO_THROTTLE(5,"Score: %f Avg: %f", score, score_sum_/double(loc_counter_));
//   loc_counter_++;
//   score_sum_ +=score;
// }

void ProbabilityMapGenerator::timerCB(const ros::TimerEvent& e) {
  point_map_.header.stamp = ros::Time::now();
  point_map_pub_.publish(point_map_); 
}

void ProbabilityMapGenerator::pointCloudCB(const sensor_msgs::PointCloudConstPtr &msg) {
  sensor_msgs::PointCloud pc = *msg;
  filterPointCloud(pc);
}

void ProbabilityMapGenerator::filterPointCloud(sensor_msgs::PointCloud pc) {
  sensor_msgs::PointCloud pc_single;
  sensor_msgs::PointCloud pc_averaged;
  point_map_.header = pc.header;
  
  // Loops through all points in the cloud and approximates to the nearest centimeter
  for(std::vector<geometry_msgs::Point32>::const_iterator i = pc.points.begin(); i != pc.points.end(); ++i) {
      geometry_msgs::Point32 approx_point;
      approx_point.x = round((*i).x*100)/100.0; // round to nearest cm then convert to mm
      approx_point.y = round((*i).y*100)/100.0;
      approx_point.z = round((*i).z*100)/100.0;
      pc_single.points.push_back(approx_point);
  }
  point_map_collected_.push_back(pc_single);

  // Collect a number of point clouds before filtering
  if (message_throttle_ > 5) {  //TODO: Make this a rosparam
    message_throttle_ = 0;
    pc_averaged = getUniquePoints(point_map_collected_);
    point_map_collected_.clear();  // get rid of the last set of point clouds
    std::copy(pc_averaged.points.begin(),pc_averaged.points.end(),std::back_inserter(point_map_.points));
  }
  message_throttle_++;
}


sensor_msgs::PointCloud ProbabilityMapGenerator::getUniquePoints(std::vector<sensor_msgs::PointCloud> pc_vector) {
  sensor_msgs::PointCloud pcloud;
  pcloud.header = ((pc_vector.back())).header;
  pcloud.channels = ((pc_vector.back())).channels;
  std::vector<geometry_msgs::Point32> filtered_points;

  // Loops through the collected point clouds
  for(std::vector<sensor_msgs::PointCloud>::const_iterator i5 = pc_vector.begin(); i5 != pc_vector.end(); ++i5) {

    // combines the points in each point cloud into one giant vector
    std::copy((*i5).points.begin(), (*i5).points.end(), std::back_inserter(filtered_points)); 
  }

  // ordered map to sort the filtered points and find the frequency
  std::map<geometry_msgs::Point32,int,PointComparator> point_freq_map;
  

  for (auto const & p : filtered_points) {
    ++point_freq_map[p];
  }
  
  for (auto const & pfm : point_freq_map) {
    // if the point occurs at least 5 times in the combined points, add it to the final list
    if (pfm.second > 4) {   //TODO: Make this a ros param
      pcloud.points.push_back(pfm.first);
    }
  }
  return pcloud; // contains only the points that occur at frequencies above a threshold

}



void ProbabilityMapGenerator::exportFinalPointCloud(void) {
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
  sprintf(output_file, "%s/data/pointscans/tunnel_%s.txt",ros::package::getPath("temporal_navigation").c_str(),output_date);
  // sprintf(second_file, "%s/data/locScores/localization_%s.txt",ros::package::getPath("temporal_navigation").c_str(),output_date);

  ROS_INFO("Exporting pointcloud of size: %lu to %s", point_map_.points.size(), output_file);
  std::ofstream filename;
  std::ofstream second_filename;

  filename.open(output_file);
  for(std::vector<geometry_msgs::Point32>::const_iterator it = point_map_.points.begin(); it != point_map_.points.end(); ++it) {
    filename << (*it).x*1000 << ", " << (*it).y*1000 << ", " << (*it).z*1000 << std::endl;
  }
  filename.close();

  // second_filename.open(second_file);
  // second_filename << "Score sum: " << score_sum_ << " Score Count: " << loc_counter_ << " Avg Loc: " << score_sum_/(double)loc_counter_ << std::endl;
  // second_filename.close();

  ROS_INFO("Data exported");
  g_shutdown_requested = 1;
}

} // end namespace probability_map_generator




int main( int argc, char** argv ) {
  ros::init(argc,argv, "probability_map_generator", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  pmt = new probability_map_generator::ProbabilityMapGenerator(nh);

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
  
  
