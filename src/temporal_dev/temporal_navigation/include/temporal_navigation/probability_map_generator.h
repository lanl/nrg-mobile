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
#ifndef PROBABILITY_MAP_GENERATOR_H
#define PROBABILITY_MAP_GENERATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float64.h>

#include <signal.h>
#include <cmath>
#include <ctime>
#include <numeric>
#include <iostream>
#include <fstream>

sig_atomic_t volatile g_shutdown_requested = 0;

namespace probability_map_generator
{
class ProbabilityMapGenerator
{
public:

  /*
   * Constructor
   * @param _n - reference to the node handle
   */
  ProbabilityMapGenerator(ros::NodeHandle& _n);

  /*
   * Destructor
   */
  ~ProbabilityMapGenerator();

  /*
   * Exports the collected laser scan points to a text file
   */
  void exportFinalPointCloud(void);

private:
  /*
   * Subscriber callback for the LIDAR point cloud data.
   *
   * @param msg - current laser scan point cloud
   */
  void pointCloudCB(const sensor_msgs::PointCloudConstPtr &msg);

  // void computeLocalizationScore(std_msgs::Float64 msg);


  /*
   * Handles the point cloud data. Throttles the data inflow
   *
   * @param pc - current laser scan point cloud
   */
  void filterPointCloud(sensor_msgs::PointCloud pc);
  
  /*
   * Eliminates non-unique data from the overall point map
   *
   * @param pc_vector - vector of several laser scan point clouds
   */
  sensor_msgs::PointCloud getUniquePoints(std::vector<sensor_msgs::PointCloud> pc_vector);

  /*
   * Publishes the current overall point map at a set interval
   *
   * @param e - timer event for publishing the cloud
   */
  void timerCB(const ros::TimerEvent& e);

  // Node handle
  ros::NodeHandle& node_;

  // Subscriber for receiving laser scan pointclouds
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber loc_score_sub_;



  // Publisher for advertising the overall point map
  ros::Publisher point_map_pub_;

  // Pointcloud that contains all filtered points detected 
  sensor_msgs::PointCloud point_map_;

  // Vector that collects a number of processed point clouds to filter simultaneously
  std::vector<sensor_msgs::PointCloud> point_map_collected_;

  // Regulates intervals for publishing the overall point map
  ros::Timer timer_;

  // Tracks the number of point clouds collected before filtering
  int message_throttle_;

  int loc_counter_;

  double score_sum_;

};
} // end probability_map_generator namespace

// This is sketchy as hell, but I'm stumped otherwise as how to use my member function on shutdown
probability_map_generator::ProbabilityMapGenerator* pmt;  

void mySigintHandler(int sig)
{
  pmt->exportFinalPointCloud();
  g_shutdown_requested = 1;
}

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  pmt->exportFinalPointCloud();
  g_shutdown_requested = 1;

  result = ros::xmlrpc::responseInt(1, "", 0);
}

/*
 *  Comparator for sorting a vector of geometry_msgs::Points
 */
class PointComparator
{
public:
  bool operator()(const geometry_msgs::Point32 a, const geometry_msgs::Point32 b) {
    return pointEval(a) > pointEval(b); 
  }
private:
  double pointEval(geometry_msgs::Point32 point) {
    return point.x*100.0 + point.y + point.z; // Heavily weights the x coordinate
  }
};

#endif
