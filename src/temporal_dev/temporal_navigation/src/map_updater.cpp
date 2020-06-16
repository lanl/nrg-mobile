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
* Description: Updates robot maps when changes occur.
*********************************************************************/
#include "temporal_navigation/map_updater.h"

namespace map_updater
{
MapUpdater::MapUpdater(ros::NodeHandle& _n) :
    node_(_n)
{
  ROS_INFO_NAMED("map_updater","map_updater: Starting Map Updater");

  // Get directory path by project name
  char dir_name[150];
  sprintf(dir_name, "%s",ros::package::getPath("temporal_navigation").c_str());
  dir_path_ = dir_name;

  // setting the resolution
  // in the future, this can be read in from a .yaml, but the default
  // is generally 20 anyway
  resolution_ = 20;

  // Setting up ros communications
  
  map_update_srv_ = node_.advertiseService("/map_updater/run_map_update", 
                                           &MapUpdater::initiateMapUpdate,this);
  


  // Just waiting for things to happen
  while(ros::ok()) {
    ROS_INFO_THROTTLE_NAMED(100,"map_updater","map_updater: Process running normally. Waiting for input.");
  }
  ROS_INFO_THROTTLE_NAMED(10,"map_updater","map_updater: Process Complete. Waiting on shutdown.");
}

MapUpdater::~MapUpdater() {}; // Default destructor


bool MapUpdater::initiateMapUpdate(temporal_navigation::MapUpdate::Request& req, 
                                   temporal_navigation::MapUpdate::Response& res) {
  ObservationMap o_map;
  PersistenceMap p_map;
  Point3D static_pts;
  o_map = req.semantic_objs;
  if(o_map.size()==0) {
    // This means data was not fed into the service with the request
    // I expect this to be abnormal when running
    try {
      getObsData(o_map,req.day);
      ROS_INFO_NAMED("map_updater:", "map_updater: Obtained observed data");
    }
    catch (...) {
      ROS_ERROR_NAMED("map_updater:","map_updater: Unable to obtain observed data");
      return false;
    }
  }
  ROS_INFO_NAMED("map_updater:","map_updater: Observed %d objects in most recent map"
                 , o_map.size());

  char persistence_path[150]; 
  sprintf(persistence_path, "%s/data/persistence_data/data/p_map_%d.txt",
                                                  dir_path_.c_str(), req.day);
  std::string p_path = persistence_path;
  if (!req.day)
    readPersistenceData(p_path, p_map);
  else
    createPersistenceMap(p_path,p_map,o_map);
  compObs2Persist(p_map, o_map);
  return true;
}


bool MapUpdater::getObsData(ObservationMap& o_map, int day) {
  // I'm gonna need to read a text file or database
  // This might end up with me drawing stuff eventually
  // This could also turn into me looking at the uncertainties in the location

  // If no data was fed in (i. e. I'm running sim data), we should grab it from file
  char dir_name[150];
  sprintf(dir_name, "%s/data/output_data/output_day_%d.txt",ros::package::getPath("map_generation").c_str(),day);
  std::string data_path = dir_name;

  std::ifstream data_file;
  data_file.open(data_path);


  std::string line;
  if (data_file.is_open()) {
    ROS_INFO_NAMED("map_updater", "map_updater: Check 1");
    while (std::getline(data_file,line)) {

      // Every new line, convert it into a string stream
      // This makes the string searchable
      std::stringstream ss(line);
      std::string returnedLine = ss.str();
      std::istringstream remainder(returnedLine);
      std::string value;
      std::vector<double> data_pt_vals;
      while(std::getline(remainder,value,',')){
        // ROS_INFO_NAMED("map_updater","map_updater: Line: %s", value.c_str());
        
        double data_pt_val = std::stod(value);
        data_pt_vals.push_back(data_pt_val);
      }
      // ROS_INFO_NAMED("map_updater_depr","Size of data_pt_vals: %d",
                      // data_pt_vals.size());
      temporal_navigation::ObservedObject data_pt;
      data_pt.p1.x = approxToResolution(data_pt_vals[0]);
      data_pt.p1.y = approxToResolution(data_pt_vals[1]);
      data_pt.p1.z = 0.0;
      data_pt.p2.x = approxToResolution(data_pt_vals[2]);
      data_pt.p2.y = approxToResolution(data_pt_vals[3]);
      data_pt.p2.z = 0.0;
      std::string type_label;
      // switch (data_pt_vals[4]) {
      //   case -1:
      //     //static
      //     type_label = "black";
      //     break;
      //   case 0:
      //     type_label = "red"
      //     break;
      //   case 1:
      //     type_label = "green"
      //     break;
      //   case 2:
      //     type_label = "blue"
      //     break;
      //   case 3: 
      //     type_label = "other"
      //     break;
      //   default:
      //     type_label = "unknown"
      //     break;

      data_pt.semantic_type.label = type_label;
      // }
      // ROS_INFO_NAMED("map_updater_depr",
      //                 "map_updater_depr: Considering Prob: [%3.1f,%3.1f]",
      //                 data_pt.loc.x,data_pt.loc.y);
      // ROS_INFO_NAMED("map_updater_depr","\t\tProbability: %3.1f Occurrence: %3.1f",
      //                 data_pt.probability,data_pt.occurrence);
      o_map.push_back(data_pt);
    }
  } else {
    ROS_ERROR_NAMED("map_updater:","map_updater: Could not locate data file %s", data_path.c_str());
    // This is ultimate failure
    return false;
  }
  return true;
}

int MapUpdater::approxToResolution(double coord) {
  int temp_coord = coord;
  int rem = temp_coord % resolution_;
  if (rem > resolution_ / 2)
    temp_coord = temp_coord - rem + resolution_;
  else
    temp_coord = temp_coord-rem;
  return temp_coord;
}


void MapUpdater::readPersistenceData(std::string& path, PersistenceMap& p_map) {
  // Reading my txt file or database
  // Couldlook at uncertainties in the location at this point

}

void MapUpdater::createPersistenceMap(std::string& path, 
                                      PersistenceMap& p_map,
                                      ObservationMap& o_map) {
    // If it's the first round, the persistence map needs to be initialized 
    // from the observed_pts
    for (auto& o_obj : o_map) {
      temporal_navigation::PersistentObject p_obj;
      p_obj.obsv_obj = o_obj;
      p_obj.up_time = 1;   // It was observed, therefore it is up
      p_obj.down_time = 0;
      p_obj.confidence = 0.1;
      // Those were the easy ones. 
      // Now to figure out temporal type stuff
      // I might have to assume based on semantic types at first
      temporal_navigation::TemporalType t_type;
      // this is super just for now because this is hideous
      // Probably will eventually have a data file on each different type
      // It will be updatable for when changes occur
      // switch (o_obj.semantic_type.label) {
      //   case "black":
      //     //static best guess
      //     t_type.label = "static";
      //     t_type.up_alpha = 2.0;
      //     t_type.down_alpha = 1.5;
      //     t_type.up_beta = 2000.0;
      //     t_type.down_beta = 3.0;
      //     break;
      //   case "red":
      //     // persistent but not static
      //     t_type.label = "persistent";
      //     t_type.up_alpha = 2.0;
      //     t_type.down_alpha = 1.5;
      //     t_type.up_beta = 50.0;
      //     t_type.down_beta = 3.0;
      //     break;
      //   case "green":
      //     // periodic (guessing weekly-ish)
      //     t_type.label = "periodic";
      //     t_type.up_alpha = 2.0;
      //     t_type.down_alpha = 2.0;
      //     t_type.up_beta = 7.0;
      //     t_type.down_beta = 7.0;
      //     break;
      //   case "blue":
      //     // sporadic (shows up, but is gone longer)
      //     t_type.label = "sporadic";
      //     t_type.up_alpha = 2.0;
      //     t_type.down_alpha = 3.5;
      //     t_type.up_beta = 5.0;
      //     t_type.down_beta = 30.0;
      //     break;
      //   case "other":
      //     // fleeting (shows up and disappears quickly)
      //     t_type.label = "fleeting";
      //     t_type.up_alpha = 1.0;
      //     t_type.down_alpha = 4.0;
      //     t_type.up_beta = 1.0;
      //     t_type.down_beta = 15.0;
      //     break;
      //   case "unknown":
      //     // just in case
      //     t_type.label = "unknown";
      //     t_type.up_alpha = 2.0;
      //     t_type.down_alpha = 2.0;
      //     t_type.up_beta = 2.0;
      //     t_type.down_beta = 2.0;
      //     break;
      // }
      p_obj.temporal_type = t_type;
      p_map.push_back(p_obj);
    }
  }

void MapUpdater::compObs2Persist(PersistenceMap& p_map, ObservationMap& o_map) {
  // Here be the comparison
  // If I have yet to reason about location uncertainty, this is the place for it
  // This might also be where I have to consider up/down switches
  // In the future, I want to reason about the likelihood of the object getting observed as
  //    one thing if it might actually be another
  // Update uncertainties if the match is poor in location or type


  
}


void MapUpdater::runSurvivalModel(PersistenceMap& p_map) {
  // Pass the persistence map to PyMC3
  // When it returns the type parameters should be updated
  // Call for the new persitence map 
}

void MapUpdater::updatePersistMap(PersistenceMap& p_map) {
  // Now that I have updated type parameters for every class, I can update the 
  //   objects in the persistence map
  // This could also be a time to switch up/down depending on what experimental data
  //    shows
 
}

void MapUpdater::exportPersistMap(int& type, int& day, double& static_thresh,
                                  PersistenceMap& p_map,
                                  std::vector<Point3D>& static_pts) {
  // This won't be needed if I do a database
  // I worry that the database would be enormous though
}

void MapUpdater::updateStaticNavMap(int& day, int& type, 
                          std::vector<Point3D>& static_pts) {
  // From the updated persistence map, I can draw a new static navigation map
  // This will likely be based on both the up/down time, and a measure of the certainty 
  //    of that predicted time
  // I haven't yet determined how to estimate certainty, but I will look at that as I 
  //    draft the bsa PyMC3 node 
}


}; // end namespace map_updater

int main( int argc, char** argv ) {
  ros::init(argc,argv, "map_updater");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  map_updater::MapUpdater* mu = new map_updater::MapUpdater(nh);


   
  // Process holds here until ROS is shutdown

  ros::shutdown();
  ros::waitForShutdown();  
  delete mu;

  return 0;
}