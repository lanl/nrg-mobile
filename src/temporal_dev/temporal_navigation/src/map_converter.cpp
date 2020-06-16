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


#include "temporal_navigation/map_converter.h"

namespace map_converter
{
MapConverter::MapConverter(ros::NodeHandle& _n) :
    node_(_n)
{
	ROS_INFO_NAMED("map_converter","map_converter: Starting Map Converter");
	ros_convert_srv_ = node_.advertiseService("map_converter/ros2arnl", 
                                      &MapConverter::parseROSMap, this);
	arnl_convert_srv_ = node_.advertiseService("map_converter/arnl2ros",  
                                      &MapConverter::parseARNLMap, this);
	update_srv_ = node_.advertiseService("map_converter/update_map",  
                                &MapConverter::updateMap, this);

  // Get directory path by project name
	char dir_name[150];
  sprintf(dir_name, "%s",ros::package::getPath("temporal_navigation").c_str());
	dir_path_ = dir_name;

  // setting the resolution
  // in the future, this can be read in from a .yaml, but the default
  // is generally 20 anyway
  resolution_ = 20;

  // Just waiting for things to happen
	while(ros::ok()) {
    ROS_INFO_THROTTLE_NAMED(100,"map_converter",
      "map_converter: Process running normally. Waiting for input.");
  }
  ROS_INFO_THROTTLE_NAMED(10,"map_converter",
    "map_converter: Process Complete. Waiting on shutdown.");
}

MapConverter::~MapConverter() {}; // Default destructor

bool MapConverter::parseROSMap(temporal_navigation::MapConvert::Request& req,
                               temporal_navigation::MapConvert::Response& res) {
  
  // Parse file names
  char in_path[150];
  char out_path[150];
  // sprintf(in_path,"%s/scripts/maps/%s", 
  sprintf(in_path,"%s/data/pointscans/test_days_100/drawings/%s", 
          dir_path_.c_str(),
          req.input_path.data.c_str());
  // sprintf(out_path,"%s/scripts/maps/%s", 
  sprintf(out_path,"%s/data/pointscans/test_days_100/color_maps/%s", 
          dir_path_.c_str(),
          req.output_name.data.c_str());
  std::string input_path = in_path;
	std::string output_path = out_path;
	ROS_INFO_NAMED("map_converter","map_converter: Input: %s", input_path.c_str());
	ROS_INFO_NAMED("map_converter","map_converter: Output: %s",output_path.c_str());

  // Open image at filename
  cv::Mat image;
  cv::Mat grey_image;
  cv::Mat image_for_arnl;
  cv::Mat blank_im;
  image = cv::imread(input_path, cv::IMREAD_COLOR);

  // Create a blank image of same size
  int m = image.rows;
  int n = image.cols;
  cv::cvtColor(image,grey_image, cv::COLOR_BGR2GRAY);
  image_for_arnl = cv::Mat(m,n, CV_8UC1, cv::Scalar(0));
  cv::threshold(image_for_arnl,blank_im,200,255,cv::THRESH_BINARY_INV);
  cv::imwrite(dir_path_+"/scripts/maps/blank_image.png",blank_im);

  // Create fillable fields for the processing functions
  std::vector<std::vector<int> > linePoints;  // ARNL formatted line end points
  std::vector<cv::Point> minMaxLines;         // Min/Max line endpoints
  std::vector<int> minMaxPoints;
  std::vector<int> rowPoints;
  std::vector<int> colPoints;

  // Processing input image and converting from ROS .png to ARNL .map
  straightenImage(image,grey_image);
  findArnlLines(req.input_path.data, linePoints,minMaxLines);
  findArnlPoints(minMaxPoints,rowPoints,colPoints);
  writeToMapFile(input_path,output_path,linePoints,
                 minMaxLines,minMaxPoints,rowPoints,colPoints,true);
  ROS_INFO_NAMED("map_converter","map_converter: Converted ros map to arnl");
  return true;
}

bool MapConverter::updateMap(temporal_navigation::MapConvert::Request& req,
                             temporal_navigation::MapConvert::Response& res) {

  // Parse filenames
  char in_path[150]; 
  char out_path[150];
  char head_path[180];
  sprintf(in_path,"%s/scripts/maps/%s", 
                   dir_path_.c_str(),req.input_path.data.c_str());
  sprintf(out_path,"%s/scripts/maps/%s", 
                    dir_path_.c_str(),req.output_name.data.c_str());
  
  std::string input_path = in_path;
  std::string output_path = out_path;

  std::size_t last = output_path.find_last_of("_");
  std::string day = output_path.substr(last+1);
  ROS_INFO("Day: %s", day.c_str());
  std::string remaining = output_path.substr(0,last);
  std::size_t sec_to_last = remaining.find_last_of("_");
  std::string type = remaining.substr(sec_to_last+1);
  ROS_INFO("Type: %s", type.c_str());
  sprintf(head_path,"%s/scripts/maps/static_maps/update_output_%s_%s.map", 
                       dir_path_.c_str(),type.c_str(),day.c_str());
  ROS_INFO("testing");
  
  std::string header_path = head_path;
  
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Input: %s", input_path.c_str());
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Output: %s",output_path.c_str());
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Header: %s",header_path.c_str());

  // Open image at filename
  cv::Mat image;
  cv::Mat grey_image;
  cv::Mat image_for_arnl;
  cv::Mat blank_im;
  image = cv::imread(input_path, cv::IMREAD_COLOR);

  // Create a blank image of same size
  int m = image.rows;
  int n = image.cols;
  cv::cvtColor(image,grey_image, cv::COLOR_BGR2GRAY);

  // Create fillable fields for the processing functions
  std::vector<std::vector<int> > linePoints;  // ARNL formatted line end points
  std::vector<cv::Point> minMaxLines;         // Min/Max line endpoints
  std::vector<int> minMaxPoints;
  std::vector<int> rowPoints;
  std::vector<int> colPoints;

  // Processing input image and converting from ROS .png to ARNL .map
  straightenImage(image,grey_image);
  findArnlLines(input_path, linePoints,minMaxLines);
  findArnlPoints(minMaxPoints,rowPoints,colPoints);
  // Notably, newHeader is set to false
  // Use persistent header when updating
  writeToMapFile(header_path,output_path,linePoints,
                 minMaxLines,minMaxPoints,rowPoints,colPoints,false);
  ROS_INFO_NAMED("map_converter","map_converter: Updated map.");
  return true;
}

void MapConverter::straightenImage(cv::Mat& image, cv::Mat& grey_im) {
  
  // Tries to clean up the gaps in the image
  cv::Mat grey_clone = grey_im.clone();
  cv::Mat edges;
  cv::Mat blur;
  std::vector<cv::Vec4i> lines;
  cv::GaussianBlur(grey_clone,blur,cv::Size(3,3),4);

  // Finds the edges
  // Canny edge detector (src,dst,thresh1,thresh2,aperture size)
  cv::Canny(blur,edges,50,150,3);  
  cv::imwrite(dir_path_+"/scripts/maps/canny_edges.png",edges);
  ROS_INFO_NAMED("map_converter","map_converter: Edges: %lu",edges.cols);

  // Tries to make lines out of edges
  // Hough lines (src, lines vector, rho,theta,thresh,srn,stn)
  cv::HoughLinesP(edges,lines,1,M_PI/180.0,10,80,40); 
  ROS_INFO_NAMED("map_converter","map_converter: Lines: %d", lines.size());
  
  double rhoPersist = 0.0;
  double distPersist = 0.0;
  double rho,theta,dist;
  cv::Point p1,p2,p1Persist,p2Persist;
  cv::Mat hough_im = image.clone();
  
  // Finds and draws all the lines on the original image
  for(auto& line:lines) {
    p1.x = line[0];
    p1.y = line[1];
    p2.x = line[2];
    p2.y = line[3];
    // distance between two points formula
    dist = std::sqrt(std::pow(p1.x-p2.x,2)+std::pow(p1.y-p2.y,2));
    
    // Tries to find the longest line
    if (dist > distPersist) {
      distPersist = dist;
      p1Persist.x = p1.x;
      p1Persist.y = p1.y;
      p2Persist.x = p2.x;
      p2Persist.y = p2.y;
    }
  }

  // Draws the longest line
  // This is the major axis of the image about which we can rotate
  cv::line(hough_im,p1Persist,p2Persist,cv::Scalar(0,0,255),5, cv::LINE_AA);
  cv::imwrite(dir_path_+"/scripts/maps/cv_hough.png",hough_im);
  double lineAngle = std::atan2(p1Persist.y-p2Persist.y,
                                p1Persist.x-p2Persist.x);
  ROS_INFO_NAMED("map_converter","map_converter: dist: %5.4f, y1: %d, y2: %d, arctan: %5.4f",distPersist,
                                                        p1Persist.y,
                                                        p2Persist.y,
                                                        lineAngle*180/M_PI);

  // Gets the angle of the greatest, longest line
  // This is probably a major wall
  // we want to align the image against this major axis
  double rot_angle = lineAngle;
  
  cv::Size im_size = image.size();
  cv::Point center(image.cols/2, image.rows/2);
  cv::Mat rot_mat;
  cv::Mat aff_mat; 
  cv::Mat rot_im;

  // Determines the mod angle to the nearest quadrant right angle
  int mod_angle = std::abs((90 - (int)std::round(rot_angle*180/M_PI)) % 90);
  if(mod_angle > 2) {
    ROS_INFO_NAMED("map_converter","map_converter: Offset angle: %d", mod_angle);
    // Creates a rotation matrix according to the offset from the axis
    rot_mat = cv::getRotationMatrix2D(center,rot_angle*180/M_PI,1);
    // Rotates the image by the rotation matrix
    cv::warpAffine(image,aff_mat,rot_mat,im_size,cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,cv::Scalar(255,255,255));
    cv::imwrite(dir_path_+"/scripts/maps/rotated_map.png",aff_mat);
    rot_im = aff_mat.clone();
  } else {
    cv::imwrite(dir_path_+"/scripts/maps/cv_hough.png",image);
    rot_im = image.clone();
  }

  // Gets rid of useless black border
  cv::Mat grey_rot,thresh_rot;
  cv::cvtColor(rot_im,grey_rot, cv::COLOR_BGR2GRAY);
  cv::threshold(grey_rot,thresh_rot,200,255,cv::THRESH_BINARY_INV);
  cv::imwrite(dir_path_+"/scripts/maps/rotated_thresh.png",thresh_rot);
  cv::Mat non0;
  cv::Point center_bb((thresh_rot.cols-1)/2.0, (thresh_rot.rows -1)/2.0);
  cv::findNonZero(thresh_rot,non0);
  cv::Rect bbox = cv::boundingRect(non0);
  cv::Mat cropped_im = rot_im(bbox);
  cv::imwrite(dir_path_+"/scripts/maps/cropped_rotated.png",cropped_im);

  // Rotates a final time to put the image into a landscape 
  // as opposed to a portrait if necessary
  cv::Mat cond_im;
  cv::Mat final_im;
  cv::Mat rot_final;
  cv::Mat warp_final;
  cv::Mat wide_im;
  cond_im = cropped_im.clone();
  int m = cond_im.rows;
  int n = cond_im.cols;
  ROS_INFO_NAMED("map_converter","map_converter: Num Rows: %d Num Cols: %d",m,n);
  if (n < m) {
    //image in portrait, need to rotate
    wide_im = cv::Mat(n,m, CV_8UC1, cv::Scalar(0));
    rot_final = cv::getRotationMatrix2D(cv::Point(n/2,m/2),90,1);
    rot_final.at<double>(0,2) += m/2 - n / 2;
    rot_final.at<double>(1,2) += n/2 - m / 2;
    cv::warpAffine(cond_im,wide_im,rot_final,wide_im.size(),cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
    final_im = wide_im;
  } else {
    //image in landscape, no need to rotate
    final_im = cond_im;
  }
  // Writing the final, appropriately rotated image
  cv::imwrite(dir_path_+"/scripts/maps/conditioned_image.png",final_im);

}

void MapConverter::findArnlLines(std::string& input_path,
                                 std::vector<std::vector<int> >& linePoints, 
                                 std::vector<cv::Point>& minMaxLines) {
  

  // New copy of original image for processing
  // cv::Mat image = cv::imread(input_path, CV_LOAD_IMAGE_COLOR);
  // int m = image.rows;
  // int n = image.cols;

  // Filtering image to make lines clearer
  filterMapLines(input_path);
  cv::Mat contourImage,imCntr;
  contourImage = cv::imread(dir_path_ +"/scripts/maps/full_ctr_map.png");
  // cv::cvtColor(imCntr,contourImage,cv::COLOR_BGR2GRAY);
  imCntr = contourImage.clone();
  int minLineLength = 20;
  int maxLineGap = 10;
  std::vector<cv::Vec4i> lines;
  
  // Finding the simple lines on the filtered image
  cv::Mat grey_ctr;
  cv::cvtColor(contourImage,grey_ctr, cv::COLOR_BGR2GRAY);
  cv::HoughLinesP(grey_ctr,lines,1,M_PI/180,60,minLineLength,maxLineGap);
  // ROS_INFO_NAMED("map_converter","map_converter: Lines: %d",lines.size());
  cv::Mat blank_im;
  blank_im = cv::imread(dir_path_ + "/scripts/maps/rotated_blank.png");
  
  // Get end points of lines
  std::vector<int> x0Points,y0Points,x1Points,y1Points;
  int xMin,xMax,yMin,yMax,xMinIdx,xMaxIdx,yMinIdx,yMaxIdx;
  for(int i = 0; i < lines.size(); i++) {
    int x0 = lines[i][0];
    int y0 = lines[i][1];
    int x1 = lines[i][2];
    int y1 = lines[i][3];
    // ROS_INFO_NAMED("map_converter","map_converter: P0: [%d, %d] P1: [%d, %d]", x0,y0,x1,y1);
    cv::line(blank_im,cv::Point(x0,y0),cv::Point(x1,y1),(0,0,0),1);
    x0Points.push_back(x0);
    y0Points.push_back(y0);
    x1Points.push_back(x1);
    y1Points.push_back(y1);

    // Getting the min/max of x/y that encompass all the line points
    if (x0 < xMin || x1 < xMin) {
      xMin = (x0 < x1) ? x0 : x1;
      xMinIdx = i;
    }
    if (x0 > xMax || x1 > xMax) {
      xMax = (x0 > x1) ? x0 : x1;
      xMaxIdx = i;
    }
    if (y0 < xMin || y1 < xMin) {
      xMin = (y0 < y1) ? y0 : y1;
      yMinIdx = i;
    }
    if (y0 > xMax || y1 > xMax) {
      xMax = (y0 > y1) ? y0 : y1;
      yMaxIdx = i;
    }

  }
  minMaxLines.push_back(cv::Point(xMin,yMin));
  minMaxLines.push_back(cv::Point(xMax,yMax));
  linePoints.push_back(x0Points);
  linePoints.push_back(y0Points);
  linePoints.push_back(x1Points);
  linePoints.push_back(y1Points);
  cv::imwrite(dir_path_+"/scripts/maps/filtered_hough.png",blank_im);  
}

void MapConverter::filterMapLines(std::string& input_path) {
        
        // Get a copy of the rotated image and invert it
        cv::Mat image, greyIm;
        image = cv::imread(dir_path_+"/scripts/maps/conditioned_image.png");
        cv::cvtColor(image,greyIm,cv::COLOR_BGR2GRAY);
        cv::Mat image_threshold,edges;
        cv::threshold(greyIm,image_threshold,150,255,cv::THRESH_BINARY);
 
        // Find the edges
        cv::Canny(image_threshold,edges,50,150,3);
        ROS_INFO_NAMED("map_converter","map_converter: Edges: %lu", edges.size());
        cv::imwrite(dir_path_+"/scripts/maps/edges_map.png",edges);

        // New blank image
        int m = image.rows;
        int n = image.cols;
        cv::Mat blank_rotated(m,n, CV_8UC1, cv::Scalar(255));
        cv::imwrite(dir_path_+"/scripts/maps/rotated_blank.png",blank_rotated);

        // Get filled contours from edges
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edges.clone(),contours,hierarchy,
                         cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
        ROS_INFO_NAMED("map_converter","map_converter: Contours: %lu",contours.size());
        cv::drawContours(image_threshold, contours, -1, (0,255,0), 1);
        cv::imwrite(dir_path_+"/scripts/maps/lines_map.png",image_threshold);
        


        // Dilate image to close small gaps then erode to get back to size
        // cv::Mat inverted;
        // cv::threshold(image_threshold,inverted,150,255,cv::THRESH_BINARY_INV);
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
        //                                             cv::Size(45,45));
        // cv::Mat image_dilate;
        // cv::dilate(inverted,image_dilate,element);
        // cv::imwrite(dir_path_ + "/scripts/maps/dilated_image.png",image_dilate);
        // cv::Mat image_erode;
        // cv::erode(image_dilate,image_erode,element);
        // cv::imwrite(dir_path_ + "/scripts/maps/eroded_image.png",image_erode);

        // Find edges again after dilation/erosion
        cv::Mat morph_edges,blank_morph;
        blank_morph = blank_rotated.clone();
        cv::Canny(image_threshold,morph_edges,50,150,3);
        std::vector<std::vector<cv::Point> > morph_contours;
        std::vector<cv::Vec4i> morph_hierarchy;
        cv::findContours(morph_edges.clone(),morph_contours,morph_hierarchy,
                         cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        ROS_INFO_NAMED("map_converter","map_converter: Morph Contours: %lu",morph_contours.size());
        cv::drawContours(blank_morph, morph_contours, -1, (0,255,0), 1);
        
        // Grabs the convex hull of the map (exterior trace)
        // Just a quick test
        std::vector<std::vector<cv::Point> > hull(morph_contours.size());
        for (int j = 0; j< morph_contours.size();j++) {     
          cv::convexHull(cv::Mat(morph_contours[j]), hull[j], false);
          cv::drawContours(blank_morph, hull, j, (0, 200, 50), 1, 8, 
                           std::vector<cv::Vec4i>(), 0, cv::Point());
        }
        cv::imwrite(dir_path_ + "/scripts/maps/morph_contours.png",blank_morph);

        shapeDetector();
        cv::Mat color_contours(m,n, CV_8UC3, cv::Scalar(255,255,255));
        // std::random_device dev;
        int dev = 200;
        cv::Mat hsv;
        cv::cvtColor(color_contours, hsv, cv::COLOR_BGR2HSV);
        std::mt19937 rng(dev);
        std::uniform_int_distribution<std::mt19937::result_type> dist1(0,179); 
        std::uniform_int_distribution<std::mt19937::result_type> dist2(0,255); 
        for (int c = 0; c < contours.size(); c++) {
          cv::Scalar hsv_rand(dist1(rng),dist2(rng),dist2(rng));
          drawContours(color_contours,contours,c, hsv_rand,3,cv::LINE_8,hierarchy);
        }
        char col_path[250];
        sprintf(col_path,"%s/data/pointscans/test_days_100/color_maps/%s", 
        dir_path_.c_str(),
        input_path.c_str());
        ROS_INFO("%s", col_path);
        cv::imwrite(col_path,color_contours);

        // Set up for line filtering
        cv::Mat imTest = cv::imread(dir_path_+"/scripts/maps/lines_map.png",1);
        int rows,cols;
        rows = imTest.rows;
        cols = imTest.cols;
        int col_count = 0;
        int count = 0;
        std::vector<std::vector<cv::Point> > filteredContours;
        std::vector<std::vector<cv::Point> > currentContour;
        std::vector<int> contourX;
        std::vector<int> contourY;
        std::vector<cv::Vec4i> currentHierarchy;
        int i = 0;

        // For each contour, check its size and exclude lines smaller than 10px
        for (int i = 0; i < contours.size(); i++) {

          // In order to be used in drawContour, each contour has to be
          // in a vector of contours. We hack this by adding the contour to 
          // a vector of 1 each time and clearing it out again.
          currentContour.clear();
          currentContour.push_back(contours[i]);
          contourX.clear();
          contourY.clear();

          //Separate the x and y points in each contour at the current level
          for (auto& ctr:contours[i] ) {
            contourX.push_back(ctr.x);
            contourY.push_back(ctr.y);
          }
          // std::cout << currentContour[0] << std::endl;

          // Draw the contours and get a mask
          cv::Scalar color( rand()&255, rand()&255, rand()&255 );
          cv::drawContours(blank_rotated, contours,i,(100, 200, 0),
                           cv::FILLED,8);
          cv::Mat grey_mask(greyIm.size(), CV_8UC1, cv::Scalar(0));
          cv::drawContours(grey_mask,currentContour,-1,255,-1);

          // Find all points in the mask that are nonzero (part of the map) 
          cv::Mat pixelpoints;
          cv::findNonZero(grey_mask,pixelpoints); 

          // Find the min/max x/y in each of the x/y contour point vectors
          // The goal is to find the extreme points in each contour
          // From the extremes, we can tell if a contour is of a size
          // worthy to be included in the line set. Otherwise, filter it out.
          int minXIdx,minYIdx,maxXIdx,maxYIdx;
          minXIdx = std::min_element(contourX.begin(),
                                      contourX.end())-contourX.begin();
          minYIdx = std::min_element(contourY.begin(),
                                      contourY.end())-contourY.begin();
          maxXIdx = std::max_element(contourX.begin(),
                                      contourX.end())-contourX.begin();
          maxYIdx = std::max_element(contourY.begin(),
                                      contourY.end())-contourY.begin();

          // Extreme points
          cv::Point leftmost,rightmost,topmost,bottommost;
          leftmost = currentContour[0][minXIdx];
          rightmost = currentContour[0][maxXIdx];
          topmost = currentContour[0][minYIdx];
          bottommost = currentContour[0][maxYIdx];

          // Distance between extreme points gives size of contour
          double test1 = std::sqrt(std::pow((leftmost.x-rightmost.x),2)
                                  +std::pow((leftmost.y-rightmost.y),2));
          double test2 = std::sqrt(std::pow((topmost.x-bottommost.x),2)
                                  +std::pow((topmost.y-bottommost.y),2));

          // Arbitrarily set to 10 pixels. This could be tunable
          if (test1 > 10  || test2  > 10) {
              // #ROS_INFO_NAMED("map_converter","map_converter: Test1: %s Test2: %s\n' % (str(test1),str(test2))
              filteredContours.push_back(currentContour[0]);
          }
        }

        ROS_INFO_NAMED("map_converter","map_converter: Final contour number: %d", filteredContours.size());
        cv::Mat blank_filtered(blank_rotated.size(), CV_8UC3, cv::Scalar(0,0,0));
        std::vector<std::vector<cv::Point> > test;
        for (auto& ctr:filteredContours) {
          // std::cout << ctr << std::endl;
          test.clear();
          test.push_back(ctr);
          cv::drawContours(blank_filtered, test, -1,cv::Scalar(0,255,0),1);
        }
        cv::imwrite(dir_path_+ "/scripts/maps/full_ctr_map.png",blank_filtered);
}

void MapConverter::findArnlPoints(std::vector<int>& minMaxPoints, 
                                  std::vector<int>& rowPoints, 
                                  std::vector<int>& colPoints) {

  // Load in the image from the line filter
  cv::Mat imageTest,greyIm,imageThresh;
  std::vector<cv::Point> pixelpoints;
  imageTest = cv::imread(dir_path_ + "/scripts/maps/lines_map.png");

  // Find the non-zero points in the inverted image to use as ARNL data points
  cv::cvtColor(imageTest,greyIm,cv::COLOR_BGR2GRAY);
  cv::threshold(greyIm,imageThresh,150,255,cv::THRESH_BINARY_INV);
  cv::imwrite(dir_path_ + "/scripts/maps/non_zero_test.png",imageThresh);
  findNonZero(imageThresh,pixelpoints);
  cv::Point  minRowPoint,minColPoint,maxRowPoint,maxColPoint;
  int minRow,minCol,maxRow,maxCol;


  /**********************************************
   *
   *
   *        !!!! IMPORTANT NOTE !!!! 
   * 
   *            ROW = Y COL = X
   *
   *        !!!! IMPORTANT NOTE !!!!
   *
   *
   *********************************************/

  // Get the min/max row/col points
  // Find the minimum y in pixel points with custom lambda
  minRowPoint = *(std::min_element(pixelpoints.begin(),pixelpoints.end(),
                  [](cv::Point const& lhs, cv::Point const& rhs) {
        return lhs.y < rhs.y;    
  }));
  // Find the minimum x in pixel points with custom lambda
  minColPoint = *(std::min_element(pixelpoints.begin(),pixelpoints.end(),
                  [](cv::Point const& lhs, cv::Point const& rhs) {
        return lhs.x < rhs.x;    
  }));
  // Find the maximum y in pixel points with custom lambda
  maxRowPoint = *(std::max_element(pixelpoints.begin(),pixelpoints.end(),
                  [](cv::Point const& lhs, cv::Point const& rhs) {
        return lhs.y < rhs.y;    
  }));
  // Find the maximum x in pixel points with custom lambda
  maxColPoint = *(std::max_element(pixelpoints.begin(),pixelpoints.end(),
                  [](cv::Point const& lhs, cv::Point const& rhs) {
        return lhs.x < rhs.x;    
  }));

  minRow = minRowPoint.y;
  minCol = minColPoint.x;
  maxRow = maxRowPoint.y;
  maxCol = maxColPoint.x;
  minMaxPoints.push_back(minRow); // MinY
  minMaxPoints.push_back(minCol); // MinX
  minMaxPoints.push_back(maxRow); // MaxY
  minMaxPoints.push_back(maxCol); // MaxX


  ROS_INFO_NAMED("map_converter","map_converter: Min Row Point: [%d, %d]", minRowPoint.x, minRowPoint.y);
  ROS_INFO_NAMED("map_converter","map_converter: Min Col Point: [%d, %d]", minColPoint.x, minColPoint.y);
  ROS_INFO_NAMED("map_converter","map_converter: Max Row Point: [%d, %d]", maxRowPoint.x, maxRowPoint.y);
  ROS_INFO_NAMED("map_converter","map_converter: Max Col Point: [%d, %d]", maxColPoint.x, maxColPoint.y);

  ROS_INFO_NAMED("map_converter","map_converter: Pixel Points Min Row: %d Min Col: %d ",minRow,minCol);
  ROS_INFO_NAMED("map_converter","map_converter: Pixel Points Max Row: %d Max Col: %d ",maxRow,maxCol);
        
  // Get the points into rows and cols
  for (auto& p:pixelpoints) {
      rowPoints.push_back(p.y);
      colPoints.push_back(p.x);
  }
}

void MapConverter::writeToMapFile(std::string& header_path, 
                                  std::string& output_path, 
                                  std::vector<std::vector<int> >& linePoints, 
                                  std::vector<cv::Point>& minMaxLines, 
                                  std::vector<int>& minMaxPoints, 
                                  std::vector<int>& rowPoints, 
                                  std::vector<int>& colPoints, 
                                  bool newHeader) {

  // Open the .map ARNL file
  std::ofstream outfile;
  outfile.open(output_path + ".map");
  if (newHeader) {
    // If it is a new map being created/converted
    int numLines = linePoints[0].size();
    int numPoints = rowPoints.size();
    writeNewMapFileHeader(outfile,numLines,numPoints,minMaxPoints,minMaxLines);
  }
  else {
    // If we are only updating the map
    std::ifstream infile;
    infile.open(header_path);
    writePersistantHeader(outfile,infile);
  }

  char output_line_string[150];

  // Print the lines info
  outfile << "LINES\n";
  for (int i = 0; i << linePoints[0].size(); i++) {
    // (x0(i) - minX)*res
    int x0Adjusted = (linePoints[0][i]-minMaxPoints[1])*resolution_;
    // (maxY - y0(i))*res
    int y0Adjusted = (minMaxPoints[2] - linePoints[1][i])*resolution_;
    // (x1(i) - minX)*res
    int x1Adjusted = (linePoints[2][i]-minMaxPoints[1])*resolution_;
    // (maxY - y1(i))*res
    int y1Adjusted = (minMaxPoints[2] - linePoints[3][i])*resolution_;

    sprintf(output_line_string,"%d %d %d %d\n",x0Adjusted,y0Adjusted,
                                               x1Adjusted,y1Adjusted);
    outfile << output_line_string;
  }

  // Print the points info
  outfile << "DATA\n";

  for (int j = 0; j < rowPoints.size(); j++) {
    // (MaxRow - y(j)) * res ((because pixels count from zero at top of image))
    int rowPointAdjusted = (minMaxPoints[2] - rowPoints[j]) *resolution_;
    // (x(j) - minCol) * res
    int colPointAdjusted = (colPoints[j]-minMaxPoints[1])*resolution_;
    sprintf(output_line_string,"%d %d \n",colPointAdjusted,rowPointAdjusted);
    outfile << output_line_string;
  }
  outfile.close();

  // Make an image out of the info we just put into the .map file 
  // in the same name as a check
  cv::Mat protoImage,bnwIm1,erosion,finalIm;
  protoImage = cv::imread(dir_path_ + "/scripts/maps/rotated_blank.png");

  int shift = 2;
  // Draw the points
  for (int k = 0; k < rowPoints.size(); k++){
      //ROS_INFO_NAMED("map_converter","map_converter: Data - X Point: %d Y Point: %d", rowPoints[k], colPoints[k]);
      cv::circle(protoImage, cv::Point(int(colPoints[k]*std::pow(2,shift)),int(rowPoints[k]*std::pow(2,shift))), 1, 
                 cv::Scalar(0,0,0), cv::FILLED,cv::FILLED, shift);
  } 

  // Draw the lines
  for (int l = 0; l < linePoints[0].size(); l++) {
     cv::line(protoImage,cv::Point(linePoints[0][l],linePoints[1][l]),
              cv::Point(linePoints[2][l],linePoints[3][l]),cv::Scalar(0,0,0),1);
  }
  // cv::threshold(protoImage,bnwIm1,150,255,cv::THRESH_BINARY_INV);
  // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
  // cv::erode(bnwIm1,erosion, element);
  // cv::threshold(erosion,finalIm,150,255,cv::THRESH_BINARY_INV);
  int bSize = 10;
  ROS_INFO_NAMED("map_converter","map_converter: Adding Border of size %d to give buffer", bSize);  
  cv::Mat withBorder;
  cv::copyMakeBorder(protoImage,withBorder, bSize,bSize,bSize,bSize,
                     cv::BORDER_CONSTANT,cv::Scalar(255,255,255));  
  cv::imwrite(output_path + ".png",withBorder);

}

void MapConverter::writeNewMapFileHeader(std::ofstream& outfile, 
                                         int numLines, 
                                         int numPoints, 
                                         std::vector<int>& minMaxPoints, 
                                         std::vector<cv::Point>& minMaxLines) {
  outfile << "2D-Map\n";
  outfile << "MinPos: 0 0\n";
  outfile << "MaxPos: " << (minMaxPoints[3] - minMaxPoints[1])*resolution_ << " " << (minMaxPoints[2] - minMaxPoints[0])*resolution_ << "\n";
  outfile << "NumPoints: " << numPoints << "\n";
  outfile << "PointsAreSorted: false\n";
  outfile << "Resolution: " << resolution_ << "\n";
  outfile << "LineMinPos: " << (minMaxLines[0].y - minMaxPoints[0])*resolution_ << " " << (minMaxLines[0].x - minMaxPoints[1])*resolution_ << "\n";
  outfile << "LineMaxPos: " << (minMaxLines[1].y - minMaxPoints[0])*resolution_ << " " << (minMaxLines[1].x - minMaxPoints[1])*resolution_ << "\n";
  outfile << "NumLines: " << numLines << "\n";
  outfile << "LinesAreSorted: false\n";
  outfile << "RouteInfo: EveryBefore ""_everyBefore""\n";
  outfile << "RouteInfo: EveryAfter ""_everyAfter""\n";
  outfile << "SchedInfo: Current ""_Friday""\n";
  outfile << "SchedInfo: Current ""_Monday""\n";
  outfile << "SchedInfo: Current ""_Saturday""\n";
  outfile << "SchedInfo: Current ""_Sunday""\n";
  outfile << "SchedInfo: Current ""_Thursday""\n";
  outfile << "SchedInfo: Current ""_Tuesday""\n";
  outfile << "SchedInfo: Current ""_Wednesday""\n";
}

void MapConverter::writePersistantHeader(std::ofstream& outfile, 
                                         std::ifstream& infile) {
  
  std::string line;
  bool break_loop = false;
  ROS_INFO("Testing persistent header");
  if (infile.is_open()) {
      while (std::getline(infile,line)) {
        std::stringstream ss(line);
        std::string value;
        // ROS_INFO("Line: %s",line.c_str() );
        while (ss >> value) {
          // We only want to change the data
          // All data comes after the line "LINES"
          if(!strcmp(value.c_str(),"LINES")) { //strcmp yields 0 for val1 = val2
            break_loop = true;
            break;
          }
          outfile << value;
          outfile << " ";
        }
        if (break_loop) {
          break;
        }
        outfile << "\n";
      }
      infile.close(); 
  }
}

bool MapConverter::parseARNLMap(temporal_navigation::MapConvert::Request& req,
                                temporal_navigation::MapConvert::Response& res) {
  
  // Get the appropriet input/output file names
  char in_path[150];
  char out_path[150];
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Test in: %s", req.input_path.data.c_str());
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Test out: %s", req.output_name.data.c_str());
  sprintf(in_path,"%s/scripts/maps/%s", dir_path_.c_str(),
                                        req.input_path.data.c_str());
  sprintf(out_path,"%s/scripts/maps/%s", dir_path_.c_str(),
                                         req.output_name.data.c_str());
  std::string input_path = in_path;
  std::string output_path = out_path;
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Input: %s", input_path.c_str());
  ROS_INFO_NAMED("map_converter",
                 "map_converter: Output: %s",output_path.c_str());

  // Open the .map file
  std::ifstream infile;
  infile.open(input_path);
  // ROS_INFO("1");
  // Setup
  std::string line;
  bool dataPointRead = false;
  bool dataLinesRead = false;
  int pointCount = 0;
  int lineCount = 0;
  int resolution;
  std::vector<cv::Point> dataPoints;
  std::vector<std::vector<int> > linePoints;
  std::vector<int> minPos;
  std::vector<int> maxPos;

  // ROS_INFO("2");
  // Line parsing
  if (infile.is_open()) {
    // ROS_INFO("3");
    while (std::getline(infile,line)) {

      // Every new line, convert it into a string stream
      // This makes the string searchable
      std::stringstream ss(line);
      std::string returnedLine = ss.str();


      if (returnedLine.find("DATA") != std::string::npos) {
        // Flag to start reading the data points
        dataPointRead = true;
        dataLinesRead = false;
        // ROS_INFO("Data line read");

      } else if (returnedLine.find("LINES") != std::string::npos) {
        // Flag to start reading the line end points
        dataPointRead = false;
        dataLinesRead = true;
        // ROS_INFO("Began reading lines");

      } else if (returnedLine.find("MinPos:") != std::string::npos 
              && returnedLine.find("Line") == std::string::npos) {
        // If it's MinPose for the points, read the point mins
        // Get everything after the ":" character as a space-delimited value
        int index = returnedLine.find(":");
        std::string remainder = returnedLine.substr(index + 1);
        int pointVal;
        std::stringstream rss(remainder);
        while (rss >> pointVal) {minPos.push_back(pointVal);}

      } else if (returnedLine.find("MaxPos:") != std::string::npos 
              && returnedLine.find("Line") == std::string::npos) {
        // If it's MinPose for the points, read the point maxes
        int index = returnedLine.find(":");
        std::string remainder = returnedLine.substr(index + 1);
        int pointVal;
        std::stringstream rss(remainder);
        while (rss >> pointVal) {maxPos.push_back(pointVal);}
       
      } else if (returnedLine.find("Resolution") != std::string::npos) {
        int index = returnedLine.find(":");
        std::string remainder = returnedLine.substr(index + 1);
        std::stringstream rss(remainder);
        while (rss >> resolution);
      
      } else if (dataPointRead == true) {
        // Everything is space-delimited
        // ROS_INFO("5");
        std::vector<int> xyPoint;
        std::stringstream rss(returnedLine);
        int pointVal;
        // ROS_INFO("5.1");
        while (rss >> pointVal) {
          // ROS_INFO("Point Val: %d",pointVal);
          xyPoint.push_back(pointVal);
        }
        // ROS_INFO("5.2");
        // Have to convert back into pixels
        cv::Point tempPoint;
        // ROS_INFO("5.20");
        // (x/res) + minX/res 
        // ROS_INFO("xyPoint x: %d", xyPoint[0]);
        // ROS_INFO("resolution: %d", resolution);
        // ROS_INFO("minPos x: %d", minPos[0]);
        tempPoint.x = (xyPoint[0]/resolution)-minPos[0]/resolution; 
        // MaxY/res - y/res
        // ROS_INFO("5.25");
        tempPoint.y = maxPos[1]/resolution-(xyPoint[1]/resolution);
        // ROS_INFO("Data point old: [%d, %d] new: [%d,%d]",xyPoint[0],
                                                               // xyPoint[1],
                                                               // tempPoint.x,
                                                               // tempPoint.y);
        // ROS_INFO("5.3");
        dataPoints.push_back(tempPoint);
        pointCount++;
        // ROS_INFO("6");

      } else if (dataLinesRead == true) {
        std::vector<int> xyPoints;
        std::stringstream rss(returnedLine);
        int pointVal;
        while (rss >> pointVal) {
          xyPoints.push_back(pointVal);
        }

        // Convert pack to pixels
        xyPoints[0] = (xyPoints[0]/resolution)-minPos[1]/resolution;  //x1
        xyPoints[1] = maxPos[1]/resolution-(xyPoints[1]/resolution);  //y1
        xyPoints[2] = (xyPoints[2]/resolution)-minPos[1]/resolution;  //x2
        xyPoints[3] = maxPos[1]/resolution-(xyPoints[3]/resolution);  //y2
        linePoints.push_back(xyPoints);
        lineCount++;
      }
      // ROS_INFO("Here perhaps?");
    }
    // ROS_INFO("No more lines");
  }
  // ROS_INFO("4");
  infile.close();
  // Draw the map based on what was just read in
  drawMap(dataPoints,linePoints,minPos,maxPos,resolution,output_path);
  // Make a matching .yaml file
  createYAML(output_path,resolution);
  ROS_INFO_NAMED("map_converter","map_converter: Arnl map converted");
  return true;
}

void MapConverter::drawMap(std::vector<cv::Point>& points, 
                           std::vector<std::vector<int> >& linePoints,
                           std::vector<int>& minPos, std::vector<int>& maxPos, 
                           int& resolution, std::string& output_path) {
  ROS_INFO_NAMED("map_converter","map_converter: MaxX: %d MinX %d MaxY: %d MinY: %d Resolution: %d", maxPos[0], 
                                                                minPos[0], 
                                                                maxPos[1], 
                                                                minPos[1], 
                                                                resolution);
  int n = std::round((maxPos[0] - minPos[0])/resolution);
  int m = std::round((maxPos[1] - minPos[1])/resolution);
  ROS_INFO_NAMED("map_converter","map_converter: Image will be %dx%d", m,n);

  // Make a blank canvas the size of the data
  cv::Mat blankIm(m,n, CV_8UC3, cv::Scalar(255,255,255));
  cv::imwrite(dir_path_ + "/scripts/maps/anotherBlank.png",blankIm);
  cv::Mat drawnMap = blankIm.clone();

  // Draw the points on the map
  int shift = 5;
  double factor = 1 << shift;
  for (int i = 0; i < points.size(); i ++) {
    // ROS_INFO_NAMED("map_converter","map_converter: Point: [%d, %d]", points[i].x, points[i].y);
    // int x = points[i].x;
    // int y = points[i].y;
    // double x_shift = x*std::pow(2,-shift);
    // double y_shift = y*std::pow(2,-shift);
    // int x_factor = int(x_shift*std::pow(2,shift));
    // int y_factor = int(y_shift*std::pow(2,shift));
    // ROS_INFO_NAMED("map_converter",
    // "map_converter: \n\tPoint: [%d, %d]\n\tPoint Unshifted: [%d, %d]\n", 
    //                                                    x,y,x_factor,y_factor);

    cv::circle(drawnMap, cv::Point(int(points[i].x*std::pow(2,shift)),int(points[i].y*std::pow(2,shift))),
               1, cv::Scalar(0,0,0), -1, -1, shift);
  }

  // Draw the lines on the map
  // for (int j = 0; j < linePoints.size(); j++) {
  //   int x0 = linePoints[j][0];
  //   int y0 = linePoints[j][1];
  //   int x1 = linePoints[j][2];
  //   int y1 = linePoints[j][3];
  //   cv::line(drawnMap, cv::Point(x0,y0),cv::Point(x1,y1),(0,0,0),1);
  // }

  // Add a small white buffer to prevent loss of info at the boundaries
  int bSize = 10;
  ROS_INFO_NAMED("map_converter","map_converter: Adding Border of size %d to give buffer", bSize);  
  cv::Mat withBorder;
  cv::copyMakeBorder(drawnMap,withBorder, bSize,bSize,bSize,bSize,
                     cv::BORDER_CONSTANT,cv::Scalar(255,255,255));      
  cv::imwrite(output_path + ".png",withBorder);
  ROS_INFO_NAMED("map_converter","map_converter: Path: %s", (output_path + ".png").c_str() );
}

void MapConverter::createYAML(std::string& output_path, int& resolution) {
  std::ofstream yamlFile;
  yamlFile.open(output_path + ".yaml");
  char output_line_string[150];
  
  yamlFile << "image: " + output_path + ".png\n";
  sprintf(output_line_string,"resolution: %f\n",std::round(resolution/1000.0));
  yamlFile << output_line_string;
  yamlFile << "origin: [0.000000,0.000000, 0.000000]\n";
  yamlFile << "negate: 0\n";
  yamlFile << "occupied_thresh: 0.65\n";
  yamlFile << "free_thresh: 0.196\n";
  yamlFile << "home_point: [1.000,1.0000,0.0000]\n";
  yamlFile.close();
}


void MapConverter::shapeDetector() {
//   cv::Mat lines, thresh,sure_bg,sure_fg,dist_transform,opening,unknown,markers,ws_img;
  cv::Mat lines, thresh,unknown;
  lines = cv::imread(dir_path_+"/scripts/maps/lines_map.png",1);
  int bSize = 10;
  ROS_INFO_NAMED("map_converter","map_converter: Adding Border of size %d to give buffer", bSize);  
  cv::Mat withBorder;
  cv::copyMakeBorder(lines,withBorder, bSize,bSize,bSize,bSize,
                     cv::BORDER_CONSTANT,cv::Scalar(255,255,255));      
  cv::imwrite(dir_path_ + "/scripts/maps/shape_border.png",withBorder);

  cv::threshold(withBorder,thresh,200,255,cv::THRESH_BINARY_INV);

  cv::imwrite(dir_path_ + "/scripts/maps/shape_thresh.png",thresh);
  cv::Mat grey_im,blank;
  ROS_INFO("Type thresh: %d", thresh.type());
  // cv::cvtColor(thresh,grey_im, cv::COLOR_BGR2GRAY);
  blank = cv::imread(dir_path_ + "/scripts/maps/rotated_blank.png");
  ROS_INFO_NAMED("map_converter","map_converter: Adding Border of size %d to give buffer", bSize);  
  cv::Mat blank_border;
  cv::copyMakeBorder(blank,blank_border, bSize,bSize,bSize,bSize,
                     cv::BORDER_CONSTANT,cv::Scalar(255,255,255));    
  cv::cvtColor(blank_border,unknown, cv::COLOR_BGR2GRAY);
  ROS_INFO("1");



  cv::Mat dilated;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
  ROS_INFO("2");
  cv::dilate(thresh,dilated,element);
  cv::imwrite(dir_path_ + "/scripts/maps/dilated.png",dilated);

  cv::Mat shape_edges;
  cv::Canny(dilated,shape_edges,50,150,5);
  cv::imwrite(dir_path_ + "/scripts/maps/shape_edges.png",shape_edges);
  std::vector<std::vector<cv::Point> > shape_contours;
  std::vector<cv::Vec4i> shape_hierarchy;
  cv::findContours(shape_edges.clone(),shape_contours,shape_hierarchy,
                   cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  for (auto vec : shape_contours) {
    for (auto v : vec)
        std::cout << v;
    std::cout << " \n" << std::endl;
  }
  for (auto vec : shape_hierarchy)
    std::cout << vec << std::endl;
  ROS_INFO_NAMED("map_converter","map_converter: Shape Contours: %lu",shape_contours.size());
  cv::Mat color_im(blank_border.rows,blank_border.cols, CV_8UC3, cv::Scalar(255,255,255));
  // std::random_device dev;
  int dev = 200;
  cv::Mat hsv;
  cv::cvtColor(color_im, hsv, cv::COLOR_BGR2HSV);
  std::mt19937 rng(dev);
  std::uniform_int_distribution<std::mt19937::result_type> dist1(0,179); 
  std::uniform_int_distribution<std::mt19937::result_type> dist2(0,255); 



  for (int c = 0; c < shape_contours.size(); c++) {
    cv::Scalar hsv_rand(dist1(rng),dist2(rng),dist2(rng));
    drawContours(color_im,shape_contours,c, hsv_rand,3,cv::LINE_8,shape_hierarchy,0);
    cv::imwrite(dir_path_ + "/scripts/maps/color_im"+std::to_string(c).c_str()+".png",color_im);
  }

//   cv::morphologyEx(grey_im,opening,cv::MORPH_OPEN,element);
//   cv::Mat erode_test;
//   cv::erode(grey_im,erode_test,element);
//   cv::imwrite(dir_path_ + "/scripts/maps/erosion.png",erode_test);
//   cv::imwrite(dir_path_ + "/scripts/maps/opening.png",opening);
//   cv::imwrite(dir_path_ + "/scripts/maps/background.png",sure_bg);
//   cv::distanceTransform(grey_im,dist_transform,cv::DIST_L2,5);
//   cv::normalize(dist_transform, dist_transform, 0, 1.0, cv::NORM_MINMAX);
//   double minVal, maxVal; 
//   cv::minMaxLoc(dist_transform, &minVal, &maxVal);
//   cv::imwrite(dir_path_ + "/scripts/maps/dist_transform.png",dist_transform);
  ROS_INFO("3");
//   cv::threshold(dist_transform,sure_fg,0.7*maxVal,255,cv::THRESH_BINARY);
//   cv::imwrite(dir_path_ + "/scripts/maps/foreground.png",sure_fg);
//   ROS_INFO("4");
//   sure_fg.convertTo(sure_fg,CV_8U);
//   ROS_INFO("Type bg: %d Type fg: %d Type unknown: %d", sure_bg.type(),sure_fg.type(), unknown.type());
//   cv::subtract(sure_bg,sure_fg,unknown);
//   ROS_INFO("4.1");
//   cv::imwrite(dir_path_ + "/scripts/maps/unknown_region.png",unknown);
//   cv::connectedComponents(sure_fg,markers);
//   ROS_INFO("4.2");
//   markers = markers + 1;
//   ROS_INFO("5");
//   markers.setTo(0,unknown==255);
//   cv::imwrite(dir_path_ + "/scripts/maps/markers.png",markers);
//   cv::watershed(lines,markers);
//   ROS_INFO("6");
//   lines.setTo(cv::Scalar(255,0,0),markers ==-1);
//   cv::imwrite(dir_path_ + "/scripts/maps/watershed.png",lines);

}
}
int main( int argc, char** argv ) {
  ros::init(argc,argv, "map_converter");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  map_converter::MapConverter* mc = new map_converter::MapConverter(nh);


   
  // Process holds here until ROS is shutdown

  ros::shutdown();
  ros::waitForShutdown();  
  delete mc;

  return 0;
}