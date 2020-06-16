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
#include "map_generation/tmap_generator.h"


namespace tmap_generator
{
TMapGenerator::TMapGenerator(ros::NodeHandle& _n) : 
	node_(_n)
{
	ROS_INFO("Starting map generator");
	init_map_gen_srv_ = node_.advertiseService("/tmap_generator/initialize_map"
                                             , &TMapGenerator::initMap,this);
	run_map_gen_srv_ = node_.advertiseService("/tmap_generator/get_next_map"
                                             , &TMapGenerator::nextMapStep,this);

  // get_dist_client_ = node_.serviceClient<bayesian_stats::SampleDist>("bayesian_stats/sample_from_distribution");

  // Get directory path by project name
  char dir_name[150];
  sprintf(dir_name, "%s",ros::package::getPath("map_generation").c_str());
  dir_path_ = dir_name;

	while(ros::ok());
    ROS_INFO_THROTTLE(10,"Process Complete. Waiting on shutdown.");
}; // default constructor

TMapGenerator::~TMapGenerator() {}; // default destructor	

bool TMapGenerator::initMap(map_generation::InitMapGen::Request& req
                          , map_generation::InitMapGen::Response& res) {
  bool success = true;
  t_objs_.clear();
  if (req.map_file !="") {
    success = createInitStaticMap(req.map_file); 
  } else {
    success = createInitStaticMap();
  }
  success = createObjects(req.type_array);
  map_count_= 0;
  return success;
}

bool TMapGenerator::createInitStaticMap(std::string path) {
  // TODO: Implement a filepath reader that will create a text file for the data
  //       that calls everything in the map static at first glance

  // TODO: Eventually implement an unknown category that will be updated in later
  //       iterations
  return true;
}

bool TMapGenerator::createInitStaticMap() {
  res_= 10;
  n_row_ = 60;
  n_col_ = 60;
  scale_ = 10;
  obj_x_size_ = 1;
  obj_y_size_ = 1;
  char map_out_path[150];
  cv::Mat blank(n_row_*res_,n_col_*res_,CV_8UC3,cv::Scalar(255,255,255));
  cv::imwrite(dir_path_+"/images/blank.png",blank);
  cv::rectangle(blank,cv::Point(0,0),cv::Point(res_,n_row_*res_)
                ,cv::Scalar(0,0,0),cv::FILLED);
  cv::rectangle(blank,cv::Point(0,0),cv::Point(n_col_*res_,res_)
                ,cv::Scalar(0,0,0),cv::FILLED);
  cv::rectangle(blank,cv::Point(n_col_*res_,0),cv::Point(n_col_*res_-res_,n_row_*res_)
                ,cv::Scalar(0,0,0),cv::FILLED);
  cv::rectangle(blank,cv::Point(0,n_row_*res_),cv::Point(n_col_*res_,n_row_*res_-res_)
                ,cv::Scalar(0,0,0),cv::FILLED);
  cv::imwrite(dir_path_+"/images/static_walls.png",blank);
  sprintf(map_out_path,"%s/images/output/map%d.png",dir_path_,map_count_);
  cv::imwrite(map_out_path,blank);
  return true;

}

bool TMapGenerator::createObjects(std::vector<int> types) {
  int type_count = 0;
  bool success = true;
  if (types.empty()) {
    ROS_ERROR("No types specified");
    return false;
  }
  int id = 0;
  for(auto& t : types) {
    // if (t == 0){
    //   ROS_ERROR("At least one type was requested with zero instances");
    //   return false;
    // }
    for (int j = 0;j<t;j++) {
      map_generation::TemporalObject obj;
      obj.type = type_count;
      obj.up_status = -1;
      geometry_msgs::Point default_p; 
      default_p.x = -1.0; default_p.y = -1.0; default_p.z = -1.0;
      obj.prev_loc = default_p;
      obj.loc = default_p;
      obj.id = id;
      success = initObjType(obj);
      t_objs_.push_back(obj);
      id++;
    }
    type_count++;
  }
  return success;
}

bool TMapGenerator::initObjType(map_generation::TemporalObject& obj) {
  std::ifstream file;
  std::string line;
  std::string value;
  std::vector<double> values_vec;

  char filepath[150];
  sprintf(filepath,"%s/data/type_vals/type%d.txt",dir_path_.c_str(),obj.type);
  
  file.open(filepath);
  // ROS_INFO("A5.9");
  if (file.is_open()) {
    // ROS_INFO("A6");
    while (std::getline(file,line)) {
      std::istringstream iss(line);
      std::string value;
      while (iss >> value) {
        values_vec.push_back(std::stod(value));
      }
      if(values_vec.empty()) {
        ROS_ERROR("File %s opened, but no values inside",filepath);
        return false;
      }
      obj.up_alpha = values_vec.at(0);
      obj.up_beta = values_vec.at(1);
      obj.down_alpha = values_vec.at(2);
      obj.down_beta = values_vec.at(3);
      obj.loc_var = 1;
      if(std::find(values_vec.begin(), values_vec.end(),0.0) != values_vec.end()) {
        /* v contains x */
        ROS_WARN("Detected zero value Weibull parameter(s)");
      } else {
        obj.up_status = 1.0;
      }
        
      values_vec.clear();
    }
    file.close(); 
  } else {
    ROS_ERROR("Unable to open %s", filepath);
    return false;
  }
  return true;
}

bool TMapGenerator::nextMapStep(std_srvs::Empty::Request& req
                              , std_srvs::Empty::Response& res) {
  // ROS_INFO("Stepping into next map");

  cv::Mat base_im;
  char map_out_path[150];
  base_im = cv::imread(dir_path_+"/images/static_walls.png");
  map_count_++;
  bool success;
  for (auto& obj : t_objs_) {
  	// ROS_INFO("Checking object count: %d", t_objs_.size());
    if (obj.loc.z < 0.0) {
      initializeLoc(obj);
      // ROS_INFO("Initializing object location at [%f,%f,%f]",obj.loc.x, obj.loc.y, obj.loc.z);
    } else {
      if (obj.up_status) {
        updateLoc(obj);
      }
      // ROS_INFO("Updated object %d location to [%f, %f, %f]",obj.id,obj.loc.x, obj.loc.y, obj.loc.z);
    }
    updateObj(obj);
    if (obj.up_status) {
      // ROS_INFO("Drawing object\n");
      drawObj(obj,base_im);
    }
  }
  sprintf(map_out_path,"%s/images/output/map%d.png",dir_path_.c_str(),map_count_);
  // ROS_INFO("Writing map to: %s",map_out_path);
  cv::imwrite(map_out_path,base_im);
  textOutput();
  return true;
}

bool TMapGenerator::drawFromObjectDist(map_generation::TemporalObject& obj) {
 /* bayesian_stats::SampleDist sd;
  if (obj.up_status) {
    sd.request.alpha = obj.up_alpha;
    sd.request.beta = obj.up_beta;
  } else if (obj.down_status) {
    sd.request.alpha = obj.down_alpha;
    sd.request.beta = obj.down_beta;
  } else {
    ROS_ERROR("Object cannot be sampled as it is no longer present in the map");
    return false;
  }

  bool serviceExists;
  while(!serviceExists) {
    serviceExists = get_dist_client_.waitForExistence(ros::Duration(5));
    ROS_INFO("Waiting for distribution sampling service to start.");
  }
  if (get_dist_client_.call(sd)) {
    if (sd.response.pred_st <= 0) {
      ROS_WARN("Sample predicts zero time state persistence for object");
    }
    if (obj.up_status) {
      obj.up_time = sd.response.pred_st;
    } 
    if (obj.down_status) {
      obj.down_time = sd.response.pred_st;
    }
    ROS_INFO("Sample collected");
  } else {
    return false;
    ROS_ERROR("Failed to get a sample distribution from server");
  } */
  if (obj.up_status < 0) {
    ROS_ERROR("Parameter values incorrectly set and/or modified.");
    return false;
  }
  // ROS_INFO("Samping object of type: %d", obj.type);
  double survival_time;
  if (obj.up_status) {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::weibull_distribution<double> dist(obj.up_alpha,obj.up_beta);
    // ROS_INFO("Sampling up St for alpha=%f beta=%f", obj.up_alpha,obj.up_beta);
    survival_time = dist(generator);
  } else {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::weibull_distribution<double> dist(obj.down_alpha,obj.down_beta);
    // ROS_INFO("Sampling down St for alpha=%f beta=%f",obj.down_alpha,obj.down_beta);
    survival_time = dist(generator);
  } 
  obj.survival_time = std::round(survival_time);
  // ROS_INFO("St: %f (%d)", survival_time, obj.survival_time);
  return true;
}

void TMapGenerator::initializeLoc(map_generation::TemporalObject& obj) {
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_int_distribution<int> dist_row(1,n_row_-2);
  std::uniform_int_distribution<int> dist_col(1,n_col_-2);
  obj.loc.x = std::round(dist_col(generator));
  obj.loc.y = std::round(dist_row(generator));
  obj.loc.z = 0.0;
  obj.prev_loc.x = -1.0;
  obj.prev_loc.y = -1.0;
  obj.prev_loc.z = -1.0;
  if (intersects(obj)){
    initializeLoc(obj);
  }
}

void TMapGenerator::updateObj(map_generation::TemporalObject& obj) {
  obj.survival_time--;
  if(obj.survival_time > 0) {
    // Continue as normal
  } else {
    // We need to flip the up/down status
    if(obj.up_status) {
      // It was up, and it has now gone to down status
      obj.up_status = 0;
      drawFromObjectDist(obj);
    } else {
      // It was down, but now it is up
      obj.up_status = 1;
      drawFromObjectDist(obj);
    }
  }
}

void TMapGenerator::updateLoc(map_generation::TemporalObject& obj) {
  obj.prev_loc = obj.loc;
  // ROS_INFO("Obj %d prev loc was [%f,%f,%f]", obj.id, obj.loc.x, obj.loc.y, obj.loc.z);
  std::random_device rd;
  std::mt19937 generator(rd());
  double mean_x = obj.loc.x;
  double mean_y = obj.loc.y;
  double stddev = std::sqrt(obj.loc_var);
  // ROS_INFO("Mean X: %f Mean Y: %f StdDev: %f", mean_x, mean_y, stddev);
  std::normal_distribution<double> dist_x(mean_x,stddev);
  std::normal_distribution<double> dist_y(mean_y,stddev);
  double value_x = 0;
  double value_y = 0;
  int intersection;
  while (value_x < 1 || value_x > n_col_-2) {
    value_x = std::round(dist_x(generator));
  }
  while (value_y < 1 || value_y > n_row_-2) {
    value_y = std::round(dist_y(generator));
  }
  obj.loc.x = value_x;
  obj.loc.y = value_y;
  obj.loc.z = 0;
  // ROS_INFO("Trying to place obj %d at [%f,%f,%f]",obj.id, obj.loc.x, obj.loc.y, obj.loc.z);
  if (intersects(obj)) {
    updateLoc(obj);
  }
}

bool TMapGenerator::intersects(map_generation::TemporalObject& obj) {
  for (auto& o : t_objs_) {
    if (obj.id - o.id != 0) {
      if (obj.loc.x == o.loc.x && obj.loc.y == o.loc.y) {
        // ROS_INFO("Current Obj %d intersects with Obj %d", obj.id, o.id);
        return true;
      }
    }
  }
  return false;
}

void TMapGenerator::drawObj(map_generation::TemporalObject& obj, cv::Mat& image) {
  cv::Scalar color;
  switch(obj.type) {
    case 0 : 
      // ROS_INFO("Type 0 object. Color red");
      color = cv::Scalar(0,0,255);
      break;
    case 1 : 
      // ROS_INFO("Type 1 object. Color green");
      color = cv::Scalar(0,255,0);
      break;       // and exits the switch
    case 2 : 
      // ROS_INFO("Type 2 object. Color blue");
      color = cv::Scalar(255,0,0);
      break;
    case 3 : 
      // ROS_INFO("Type 2 object. Color ?");
      color = cv::Scalar(255,255,0);
      break;

  }
  cv::rectangle(image,cv::Point(obj.loc.x*res_,obj.loc.y*res_),
                      cv::Point((obj.loc.x+obj_x_size_)*res_,obj.loc.y*res_+1),
                      color,cv::FILLED);
  cv::rectangle(image,cv::Point(obj.loc.x*res_,obj.loc.y*res_),
                      cv::Point(obj.loc.x*res_+1,(obj.loc.y+obj_y_size_)*res_),
                      color,cv::FILLED);
  cv::rectangle(image,cv::Point((obj.loc.x+obj_x_size_)*res_,obj.loc.y*res_),
                      cv::Point((obj.loc.x+obj_x_size_)*res_-1,(obj.loc.y+obj_y_size_)*res_),
                      color,cv::FILLED);
  cv::rectangle(image,cv::Point((obj.loc.x+obj_x_size_)*res_,(obj.loc.y+obj_y_size_)*res_),
                      cv::Point(obj.loc.x*res_,(obj.loc.y+obj_y_size_)*res_-1),
                      color,cv::FILLED);
}

void TMapGenerator::textOutput(void) {
  // Gives the bounding box around the shape and it's type
  char dir_name[150];
  std::string dir_path;
  sprintf(dir_name, "%s",ros::package::getPath("map_generation").c_str());
  dir_path = dir_name;
  std::ofstream out_file;
  char out_path[150]; 
  sprintf(out_path, "%s/data/output_data/output_day_%d.txt",
                      dir_path.c_str(), map_count_);
  out_file.open(out_path);

  // For the static walls (type -1)
  out_file << "0.0, 0.0, 600.0, 10.0, -1";
  out_file << "0.0, 0.0, 10.0, 600.0, -1";
  out_file << "590.0, 0.0, 600.0, 600.0, -1";
  out_file << "0.0, 590.0, 600.0, 600.0, -1";
  for (auto& obj : t_objs_) {
    char out_data[200];   
    // ROS_INFO("test prob"); 
    if (obj.up_status) {
      sprintf(out_data, "%3.2f, %3.2f, %3.2f, %3.2f, %d\n", obj.loc.x*res_,obj.loc.y*res_,
                                                           (obj.loc.x+obj_x_size_)*res_,
                                                           (obj.loc.y+obj_y_size_)*res_, 
                                                            obj.type);
      out_file << out_data;
    }   
    // ROS_INFO("Testing: %s", out_data);
  } // end for
} // end textOutput




}; // end namespace

int main( int argc, char** argv ) {
  ros::init(argc,argv, "tmap_generator");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  tmap_generator::TMapGenerator* tmg = new tmap_generator::TMapGenerator(nh);


  // Process holds here until ROS is shutdown

  ros::shutdown();
  ros::waitForShutdown();  
  delete tmg;
  

  return 0;
}
