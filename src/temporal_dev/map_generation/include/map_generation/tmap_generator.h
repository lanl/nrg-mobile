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
* Description: Generates simulated spatio-temporal map data.
*********************************************************************/
#ifndef TMAP_GENERATOR_H
#define TMAP_GENERATOR_H

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
#include <random>
#include <dirent.h>
#include <opencv2/opencv.hpp>				// image processing
#include <geometry_msgs/Point.h>
#include "map_generation/TemporalObject.h"
#include "map_generation/InitMapGen.h"
#include <std_srvs/Empty.h>
#include "bayesian_stats/SampleDist.h"

namespace tmap_generator
{
class TMapGenerator {
public:
	TMapGenerator(ros::NodeHandle& _n); // default constructor
	~TMapGenerator(); // default destructor
private:

	bool initMap(map_generation::InitMapGen::Request& req, map_generation::InitMapGen::Response& res);
	bool createInitStaticMap(std::string path);
	bool createInitStaticMap(void);
	bool createObjects(std::vector<int> types);
	bool initObjType(map_generation::TemporalObject& obj);
	bool nextMapStep(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool drawFromObjectDist(map_generation::TemporalObject& obj);
	void initializeLoc(map_generation::TemporalObject& obj);
	void updateObj(map_generation::TemporalObject& obj);
	void updateLoc(map_generation::TemporalObject& obj);
	bool intersects(map_generation::TemporalObject& obj); 
	void drawObj(map_generation::TemporalObject& obj, cv::Mat& image); 
	void textOutput(void);

  	// Node handle
  	ros::NodeHandle& node_;
  	ros::ServiceServer init_map_gen_srv_;
  	ros::ServiceServer run_map_gen_srv_;
  	// ros::ServiceClient get_dist_client_;
  	int n_row_;
  	int n_col_;
  	int res_;
  	int scale_;
  	int obj_x_size_;
	int obj_y_size_;
  	int map_count_;


  	std::string dir_path_;
	std::vector<map_generation::TemporalObject> t_objs_;
};

}

#endif