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
#ifndef localization_compute_H
#define localization_compute_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>

#include <std_msgs/Float64.h>

#include <signal.h>
#include <cmath>
#include <ctime>
#include <numeric>
#include <iostream>
#include <fstream>

sig_atomic_t volatile g_shutdown_requested = 0;

namespace localization_compute
{
class LocalizationCompute
{
public:

  /*
   * Constructor
   * @param _n - reference to the node handle
   */
  LocalizationCompute(ros::NodeHandle& _n);

  /*
   * Destructor
   */
  ~LocalizationCompute();

  /*
   * Exports the collected laser scan points to a text file
   */
  void exportFinalLocScore(void);

private:




  // Node handle
  ros::NodeHandle& node_;

  // Subscriber for receiving laser scan pointclouds
  ros::Subscriber loc_score_sub_;




  void computeLocalizationScore(const std_msgs::Float64::ConstPtr& msg);


  int loc_counter_;

  double score_sum_;

  };
} // end localization_compute namespace

// This is sketchy as hell, but I'm stumped otherwise as how to use my member function on shutdown
localization_compute::LocalizationCompute* pmt;  

void mySigintHandler(int sig)
{
  pmt->exportFinalLocScore();
  g_shutdown_requested = 1;
}

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  pmt->exportFinalLocScore();
  g_shutdown_requested = 1;

  result = ros::xmlrpc::responseInt(1, "", 0);
}


#endif
