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

#ifndef map_updater_H
#define map_updater_H


#include <math.h>               // general mathy stuff
#include <algorithm>            // for finding things in vectors
#include <iostream>             // I/O file stuff
#include <fstream>              // I/O file stuff
#include <sstream>              // I/O file stuff


#include <opencv2/opencv.hpp>       // image processing

#include <ros/ros.h>            // all of ros
#include <ros/package.h>        // how I find the directory path
#include <std_srvs/Empty.h>      // Empty ros service
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>

#include <temporal_navigation/MapConvert.h> // custom msgs types for I/O paths
#include <temporal_navigation/MapUpdate.h>  // custom msgs types for I/O paths
#include <temporal_navigation/FilterData.h>
#include <temporal_navigation/InitFilter.h>
#include <temporal_navigation/PersistentObject.h>
#include <temporal_navigation/ObservedObject.h>
#include <temporal_navigation/SemanticType.h>
#include <temporal_navigation/TemporalType.h>
#include <temporal_navigation/Point3D.h>

namespace map_updater
{

typedef std::vector<temporal_navigation::PersistentObject> PersistenceMap;
typedef std::vector<temporal_navigation::ObservedObject> ObservationMap;
class MapUpdater
{
public:

  /*
   * Constructor
   * @param _n - reference to the node handle
   */
  MapUpdater(ros::NodeHandle& _n);

  /*
   * Destructor
   */
  ~MapUpdater();

private:

  // Node handle
  ros::NodeHandle& node_;

  // Services for different map processing requests
  ros::ServiceClient update_arnl_client_;
  ros::ServiceClient update_map_client_;
  ros::ServiceServer map_update_srv_;
  // Oft used throughout member variables
  std::string dir_path_;  // path to the project directory found on launch
  int resolution_;  // of the map (usually around 20)


  
  // Primary functions
  /*
   * Service callback for initiating a call to the BSA method
   *
   * @param req - input for MapUpdate type which is the day/survey id
   * @param res - output for MapUpdate type which is empty
   */  
  bool initiateMapUpdate(temporal_navigation::MapUpdate::Request& req, 
                    temporal_navigation::MapUpdate::Response& res);

  

  /*
   * Subscriber callback for updating the static map when the probability 
   *  map dictates that a point has fallen under the threshold for 
   *  an obstacle
   *
   * @param msg - input and output filenames for the before and after
   */    
  bool getObsData(ObservationMap& o_map,int day);


  // Helper functions
  /*
   * Takes in the input image and rotates it about its major axis. Shoots  
   *  for landscape orientation as the preference
   *
   * @param image - input color image passed by reference
   * @param grey_im - input grey image passed by reference
   */    
  int approxToResolution(double coord);

  /*
   * Uses Probabilistic Hough Line detection in order to find the lines
   *  in a filtered image and converts the lines into ARNL format
   *
   * @param input_path - input path for the image
   * @param linePoints - final output vector of line points in ARNL format 
   *           to be filled by the function
   * @param minMaxLines - vector of points that collects the end points for
   *            the min and max lines in x and y for the image
   */  
  void readPersistenceData(std::string& path, 
                    PersistenceMap& prob_pts);

  /*
   * Generally cleans up the lines in the map. Specifically, it helps 
   *    findArnlLines to filter the input image such that smaller disjointed
   *  lines are not counted and longer lines with small breaks are joined 
   *  together. 
   *
   * @param input_path - input path for the image
   */    
  void createPersistenceMap(std::string& path, PersistenceMap& p_map,ObservationMap& o_map);

  /*
   * Uses the filtered image to find the non-zero pixels in the map and
   *  converts them into map points in ARNL format
   *
   * @param minMaxPoints - output vector of min/max row/col points to be 
   *             filled by the function
   * @param rowPoints - final output vector of row (y) points in ARNL format 
   *          to be filled by the function
   * @param colPoints - final output vector of col (x) points in ARNL format 
   *          to be filled by the function
   */ 
  void compObs2Persist(PersistenceMap& p_map, ObservationMap& o_map);

  /*
   * Writes the ARNL formatted lines and points into the output .map file
   *  converts them into map points in ARNL format
   *
   * @param input_path - input path for the image .png
   * @param output_path - output path for the ARNL .map
   * @param linePoints - line points in ARNL format
   * @param minMaxLines - end points for the min and max lines in x and y
   * @param minMaxPoints - vector of min/max row/col points 
   * @param rowPoints - vector of row (y) points in ARNL format
   * @param colPoints - vector of col (x) points in ARNL format
   * @param newHeader - boolean input to specify whether a new header should
   *          be written
   */ 
  void runSurvivalModel(PersistenceMap& p_map);
  
  /*
   * Copies the header that is already in the file and writes it again.
   *  Done in the case of an updating map, not a new conversion.
   *
   * @param outfile - output file for the .map
   * @param infile - input file of .map that has the copyable header
   */      
  void updatePersistMap(PersistenceMap& p_map);

  /*
   * Draws a .png image based on the read in .map values. This file is 
   *  readable by ROS.
   *
   * @param points - vector of xy points from ARNL that can be pixelized
   * @param linePoints - vector of endpoints of lines in ARNL
   * @param minPos - min x and y of map
   * @param maxPos - max x and y of map 
   * @param resolution -  resolution of pixels to points
   * @param output_path - desired filename of output .png for ROS
   *           
   */ 
  void exportPersistMap(int& type, int& day, double& static_thresh,
                                  PersistenceMap& p_map,
                                  std::vector<Point3D>& static_pts);

  /*
   * Creates the .yaml file that ROS accesses in order to find its map
   *
   * @param output_path - desired filename of output .yaml for ROS
   * @param resolution -  resolution of pixels to points
   *           
   */ 
  void updateStaticNavMap(int& day, int& type, 
                          std::vector<Point3D>& static_pts);




};


}

#endif