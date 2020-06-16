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
* Description: Image processing on map files.
*********************************************************************/

#ifndef MAP_CONVERTER_H
#define MAP_CONVERTER_H


#include <math.h>         					// general mathy stuff
#include <ros/ros.h>						// all of ros
#include <ros/package.h>					// how I find the directory path
#include <opencv2/core.hpp>				// image processing
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <temporal_navigation/MapConvert.h>	// custom srvs types for I/O paths
#include <algorithm>						// for finding things in vectors
#include <iostream>							// I/O file stuff
#include <fstream>							// I/O file stuff
#include <sstream>							// I/O file stuff
#include <random>

namespace map_converter
{
class MapConverter
{
public:

  /*
   * Constructor
   * @param _n - reference to the node handle
   */
  MapConverter(ros::NodeHandle& _n);

  /*
   * Destructor
   */
  ~MapConverter();

private:

  // Node handle
  ros::NodeHandle& node_;

  // Subscriber for different map processing requests
  ros::ServiceServer ros_convert_srv_;
  ros::ServiceServer arnl_convert_srv_;
  ros::ServiceServer update_srv_;

  // Oft used throughout member variables
  std::string dir_path_;  // path to the project directory found on launch
  int resolution_;  // of the map (usually around 20)

  
  // Primary functions
  /*
   * Service callback for converting from ros png to arnl .map file
   *
   * @param req - input and output filenames for the two types
   */  
  bool parseROSMap(temporal_navigation::MapConvert::Request& req,
                   temporal_navigation::MapConvert::Response& res);

  /*
   * Service callback for converting from arnl.map to ros yaml/png file
   *
   * @param req - input and output filenames for the two types
   */    
  bool parseARNLMap(temporal_navigation::MapConvert::Request& req,
                    temporal_navigation::MapConvert::Response& res);

  /*
   * Service callback for updating the static map when the probability 
   *	map dictates that a point has fallen under the threshold for 
   * 	an obstacle
   *
   * @param req - input and output filenames for the before and after
   */    
  bool updateMap(temporal_navigation::MapConvert::Request& req,
                 temporal_navigation::MapConvert::Response& res);


  // Helper functions
  /*
   * Takes in the input image and rotates it about its major axis. Shoots  
   *	for landscape orientation as the preference
   *
   * @param image - input color image passed by reference
   * @param grey_im - input grey image passed by reference
   */    
  void straightenImage(cv::Mat& image, cv::Mat& grey_im);

  /*
   * Uses Probabilistic Hough Line detection in order to find the lines
   * 	in a filtered image and converts the lines into ARNL format
   *
   * @param input_path - input path for the image
   * @param linePoints - final output vector of line points in ARNL format 
   *					 to be filled by the function
   * @param minMaxLines - vector of points that collects the end points for
   *					  the min and max lines in x and y for the image
   */  
  void findArnlLines(std::string& input_path,
  					 std::vector<std::vector<int> >& linePoints, 
  					 std::vector<cv::Point>& minMaxLines);

  /*
   * Generally cleans up the lines in the map. Specifically, it helps 
   *    findArnlLines to filter the input image such that smaller disjointed
   *	lines are not counted and longer lines with small breaks are joined 
   * 	together. 
   *
   * @param input_path - input path for the image
   */    
  void filterMapLines(std::string& input_path);

  /*
   * Uses the filtered image to find the non-zero pixels in the map and
   * 	converts them into map points in ARNL format
   *
   * @param minMaxPoints - output vector of min/max row/col points to be 
   *					   filled by the function
   * @param rowPoints - final output vector of row (y) points in ARNL format 
   *					to be filled by the function
   * @param colPoints - final output vector of col (x) points in ARNL format 
   *					to be filled by the function
   */ 
  void findArnlPoints(std::vector<int>& minMaxPoints, 
  					  std::vector<int>& rowPoints, 
  					  std::vector<int>& colPoints);

  /*
   * Writes the ARNL formatted lines and points into the output .map file
   * 	converts them into map points in ARNL format
   *
   * @param input_path - input path for the image .png
   * @param output_path - output path for the ARNL .map
   * @param linePoints - line points in ARNL format
   * @param minMaxLines - end points for the min and max lines in x and y
   * @param minMaxPoints - vector of min/max row/col points 
   * @param rowPoints - vector of row (y) points in ARNL format
   * @param colPoints - vector of col (x) points in ARNL format
   * @param newHeader - boolean input to specify whether a new header should
   *					be written
   */ 
  void writeToMapFile(std::string& input_path,
  					  std::string& output_path,
  					  std::vector<std::vector<int> >& linePoints,
  					  std::vector<cv::Point>& minMaxLines,
  					  std::vector<int>& minMaxPoints,
  					  std::vector<int>& rowPoints,
  					  std::vector<int>& colPoints,
  					  bool newHeader);

  /*
   * Writes the header part of the .map file. This only happens on a first 
   *	time conversion. All times after, the persistent header is used.
   *
   * @param outfile - output file for the .map
   * @param numLines - total number of ARNL lines
   * @param numPoints - total number of ARNL points
   * @param minMaxPoints - vector of min/max row/col points 
   * @param minMaxLines -  end points for the min and max lines in x and y
   *					 
   */ 
  void writeNewMapFileHeader(std::ofstream& outfile,
  							int numLines, 
  							int numPoints, 
  							std::vector<int>& minMaxPoints, 
  							std::vector<cv::Point>& minMaxLines);

  /*
   * Copies the header that is already in the file and writes it again.
   *	Done in the case of an updating map, not a new conversion.
   *
   * @param outfile - output file for the .map
   * @param infile - input file of .map that has the copyable header
   */      
  void writePersistantHeader(std::ofstream& outfile, std::ifstream& infile);

  /*
   * Draws a .png image based on the read in .map values. This file is 
   *	readable by ROS.
   *
   * @param points - vector of xy points from ARNL that can be pixelized
   * @param linePoints - vector of endpoints of lines in ARNL
   * @param minPos - min x and y of map
   * @param maxPos - max x and y of map 
   * @param resolution -  resolution of pixels to points
   * @param output_path - desired filename of output .png for ROS
   *					 
   */ 
  void drawMap(std::vector<cv::Point>& points, 
  			   std::vector<std::vector<int> >& linePoints, 
  			   std::vector<int>& minPos, 
  			   std::vector<int>& maxPos, 
  			   int& resolution, 
  			   std::string& output_path);

  /*
   * Creates the .yaml file that ROS accesses in order to find its map
   *
   * @param output_path - desired filename of output .yaml for ROS
   * @param resolution -  resolution of pixels to points
   *					 
   */ 
  void createYAML(std::string& output_path, int& resolution);

  void shapeDetector(void);

};
}

#endif