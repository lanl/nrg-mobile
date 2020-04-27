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
* Authors: Alex von Sternberg and Meredith Symmank
*
* Description: Converts an occupancy grid from the ROS navigation 
*  stack into a coarser grid that can be used for cellular 
*  decomposition and navigation purposes
*********************************************************************/

#include "coarse_grid_converter.h"

namespace full_coverage
{

CoarseGridConverter::CoarseGridConverter() :
  n("~"),
  spinner(3),
  mapAvailable(false),
  costmapAvailable(false),
  transformCount(0),
  broadcastTransform(false),
  broadcastPreviousTransform(false)
{
  std::string mt = "/map";
  std::string ct = "/move_base/global_costmap/costmap";
  if(!n.getParam("/GridConverter/map_topic",mt))
  {
    ROS_ERROR("CoarseGridConverter could not read map topic from param server. Setting to default /map");
  }
  if(!n.getParam("/GridConverter/costmap_topic",ct))
  {
    ROS_ERROR("CoarseGridConverter could not read costmap topic from param server. Setting to default /move_base/global_costmap/costmap");
  }

  navGrid.grid_cells.clear();
  navGrid.grid_cols = 0;
  mapSub = n.subscribe<nav_msgs::OccupancyGrid>(mt,1,&CoarseGridConverter::mapCallback,this);
  costmapSub = n.subscribe<nav_msgs::OccupancyGrid>(ct,1,&CoarseGridConverter::costmapCallback,this);
  costmapUpdatesSub = n.subscribe<map_msgs::OccupancyGridUpdate>(ct + "_updates",1,&CoarseGridConverter::costmapUpdatesCallback,this);
  getGridServer = n.advertiseService("get_coarse_grid", &CoarseGridConverter::getGridCallback, this);
  rotPub = n.advertise<nav_msgs::OccupancyGrid>("/rotated_map",1);
 
  spinner.start();

  previousTransform.child_frame_id_ = "";
}

CoarseGridConverter::~CoarseGridConverter()
{
  // do nothing
}

bool CoarseGridConverter::getGridCallback(full_coverage::GetCoarseGrid::Request &req, full_coverage::GetCoarseGrid::Response &res)
{
  if (mapAvailable && costmapAvailable)
  {
    parseMap(publishedMap,publishedCostmap);
    res.cell_grid = navGrid;
    //if (req.print_map)
      printNewGrid();
    return true;
  }
  else
    return false;
}

void CoarseGridConverter::mapCallback(nav_msgs::OccupancyGrid grid)
{
  publishedMap = grid;
  mapAvailable = true;
}

void CoarseGridConverter::costmapCallback(nav_msgs::OccupancyGrid grid)
{
  publishedCostmap = grid;
  costmapAvailable = true;
}

void CoarseGridConverter::costmapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& gridUpdate)
{
  // If we don't have a full map yet, do nothing
  if(!costmapAvailable)
  {
    return;
  }

  // Reject updates which have any out-of-bounds data.
  if(gridUpdate->x < 0 ||
     gridUpdate->y < 0 ||
     publishedCostmap.info.width < gridUpdate->x + gridUpdate->width ||
     publishedCostmap.info.height < gridUpdate->y + gridUpdate->height )
  {
    return;
  }

  // Copy the incoming data into publishedCostmap's data.
  for( size_t y = 0; y < gridUpdate->height; y++ )
  {
    memcpy( &publishedCostmap.data[ (gridUpdate->y + y) * publishedCostmap.info.width + gridUpdate->x ],
            &gridUpdate->data[ y * gridUpdate->width ],
            gridUpdate->width );
  }
}

void CoarseGridConverter::parseMap(nav_msgs::OccupancyGrid globalGrid, nav_msgs::OccupancyGrid localGrid)
{
  double mapRotation = 0.0;
  findRotation(globalGrid, mapRotation);

  if(globalGrid.header.frame_id.compare(localGrid.header.frame_id) != 0)
  {
    ROS_ERROR_THROTTLE(1.0, "CoarseGridConverter: Global and local maps should have same frame_id. Cannot overlay local grid on global grid.");
  }
  else
  {
    overlayLocal(globalGrid,localGrid);
  }

  int oldWidth = globalGrid.info.width;
  int oldHeight = globalGrid.info.height;
  ROS_INFO("Coarse Grid Converter: Map width Coarse Grid Converter: (pixels): %4d Map height (pixesl): %4d", oldWidth, oldHeight);
  if(fabs(mapRotation) > 0.00001)
  {
    rotateMap(globalGrid, mapRotation);
    ROS_INFO_STREAM("Coarse Grid Converter: Rotated map by " << mapRotation << " radians.");
  }
  else
    ROS_INFO("Coarse Grid Converter: No map rotation necessary");

  // Pixelated map dimensions
  int gwidth = globalGrid.info.width;
  int gheight = globalGrid.info.height;

  // Real map dimensions
  double realWidth = gwidth * globalGrid.info.resolution;
  double realHeight = gheight * globalGrid.info.resolution;
  ROS_INFO("Coarse Grid Converter: Map resolution (m/cell): %4.2f Real Width (m): %4.2f Real Height(m): %4.2f", globalGrid.info.resolution, realWidth, realHeight);

  // Placeholders for the ROS params
  int numCells;
  double robotDim;     // meters

  if(!n.getParam("/Grid/cell_dim", numCells) ||
     !n.getParam("/Grid/sweep_width", robotDim))
  {
    ROS_ERROR("Coarse Grid Converter: Could not get parameters from ROS parameter server. Cannot parse map.");
    return;
  }

  ROS_INFO_STREAM("Coarse Grid Converter: Robot sweep cell width is " << robotDim);
  ROS_INFO_STREAM("Coarse Grid Converter: Robot cell will be " << numCells << " by " << numCells << " wavefront cells.");

  // Make robot cells
  double cellDim = robotDim/numCells;

  // Breaks down the grid into rows and columns
  double cols_guess = realWidth/cellDim;
  double rows_guess = realHeight/cellDim;
  // Divides the pixels into cells
  int pixPerCellWidth = gwidth/cols_guess;
  int pixPerCellLength = gheight/rows_guess;
  ROS_INFO("Coarse Grid Converter: Pixels/Cell Width: %d Pixels/Cell Height: %d", pixPerCellWidth, pixPerCellLength);
  // Adjust cols for whole number of pixels
  int numCols = gwidth/pixPerCellWidth;
  int numRows = gheight/pixPerCellLength;
  ROS_INFO("Coarse Grid Converter: The grid will be %d by %d (rxc)", numRows, numCols);
  // Store the info in the output grid
  navGrid.grid_cells.clear();
  navGrid.grid_cols = numCols;
  navGrid.frame_id = globalGrid.header.frame_id;
  navGrid.stamp = ros::Time::now();

  // Calulate new robot cell dimensions
  double robotWidth = pixPerCellWidth * globalGrid.info.resolution * numCells;
  double robotLength = pixPerCellLength * globalGrid.info.resolution * numCells;
  ROS_INFO_STREAM("Robot cells will be approximated as " << robotWidth << " meters wide and " << robotLength << " meters long.");

  std::vector<int> occSums; // Tracks the occupancy sums in each coarse cell in a coarse row
  occSums.resize(numCols,0); // Need one sum for each cell in the current row
  std::vector<bool> unknown;
  unknown.resize(numCols, false);
  std::vector<bool> occupied;
  occupied.resize(numCols, false);
  int occSum = 0;           // Sum of one row of fine cells within a coarse cell

  // Cycles through pixels starting in bottom left corner [(max vertical pixels - 1), 0]
  // and going to top right [0, (max horizontal pixels - 1)]
  for (int i = (gheight-1); i >= 0; i--)
  {
    for (int j = 0; j < gwidth; j++)
    {
      int index = i*gwidth+j;
      //  Add fine cell's value to row total
      occSum += globalGrid.data[index];
      int tempI = 0;
      if(0 == (j+1) % pixPerCellWidth)
        tempI = (j+1)/pixPerCellWidth - 1;
      else
        tempI = floor((j+1)/pixPerCellWidth);
      if(globalGrid.data[index]<0)
        unknown[tempI] = true;
      if(100==globalGrid.data[index])
        occupied[tempI] = true;
      //  If this is the last fine cell within a coarse cell, record the total
      if ((j+1) % pixPerCellWidth == 0)
      {
        occSums[((j+1)/pixPerCellWidth)-1] += occSum;
        occSum = 0;
      }

      // When the pixel that corresponds to the top right corner of a coarse cell is
      // reached, create the cell.
      if((j+1) % pixPerCellWidth == 0 && i % pixPerCellLength == 0)
      {
        // Get the average occupancy of a cell
        double occAvg = occSums[((j+1)/pixPerCellWidth)-1]/(pixPerCellWidth*pixPerCellLength);
        if(unknown[((j+1)/pixPerCellWidth)-1] && occAvg < 10)
          occAvg = -1;
        if(occupied[((j+1)/pixPerCellWidth)-1])
          occAvg = 100;
        // Create the new cell
        full_coverage::SquareCell cell = fillCell(i, j, pixPerCellWidth, pixPerCellLength, globalGrid.info.resolution, occAvg, globalGrid.info.origin);
        // Add the new cell to the output grid
        navGrid.grid_cells.push_back(cell);
      }

      // If this is the last coarse cell in a row, reset the sums
      if(j==(gwidth-1) && i % pixPerCellLength == 0)
      {
        occSums.clear();
        occSums.resize(numCols,0);
        unknown.clear();
        unknown.resize(numCols,false);
        occupied.clear();
        occupied.resize(numCols,false);
      }
    }
  }

}  //parseMap

void CoarseGridConverter::overlayLocal(nav_msgs::OccupancyGrid &globalGrid, const nav_msgs::OccupancyGrid &localGrid)
{
  // check scale
  int scale = localGrid.info.resolution/globalGrid.info.resolution;
  if (scale < 1)
    scale = 1;

  // copy data
  for(int i = 0; i < localGrid.info.height; i++) 
  {
    for(int j = 0; j < localGrid.info.width; j++)
    {
      // calculate equivalent local grid index
      int localIndex = i*localGrid.info.width + j;
      double x = localGrid.info.origin.position.x + j*localGrid.info.resolution + localGrid.info.resolution/2.0;
      double y = localGrid.info.origin.position.y + i*localGrid.info.resolution + localGrid.info.resolution/2.0;
      int globalj = floor((x-globalGrid.info.origin.position.x)/globalGrid.info.resolution);
      int globali = floor((y-globalGrid.info.origin.position.y)/globalGrid.info.resolution);

      for(int row = globali - (scale/2); row <= globali + (scale/2); row++)
      {
        for(int col = globalj - (scale/2); col <= globalj + (scale/2); col++)
        {
          int globalIndex = row*globalGrid.info.width + col;
          // take the max value of the global and local grid at this point
          if(globalIndex >= 0 && globalIndex < globalGrid.data.size())
          {
            if(localGrid.data[localIndex] == 100)
              globalGrid.data[globalIndex] = 100;
          }
        }
      }
    }
  }
}

void CoarseGridConverter::findRotation(const nav_msgs::OccupancyGrid &grid, double &mapRotation)
{
  // Convert occupancy grid to an image. We use color channels for known/unknown areas.
  nav_msgs::MapMetaData mInfo = grid.info;
  cv::Mat colorMap = cv::Mat(mInfo.height, mInfo.width, CV_8UC3);
  cv::Mat binaryMap = cv::Mat(mInfo.height, mInfo.width, CV_8UC1);

  // iterate over map, store in image
  // (0,0) is lower left corner of OccupancyGrid
  for(int i = 0; i < mInfo.height; i++) 
  {
    for(int j = 0; j < mInfo.width; j++) 
    {
      double value = grid.data[i*mInfo.width+j]*255/100;
      if(value < 0)
      {
        binaryMap.at<uchar>(i,j) = 0;
        colorMap.at<cv::Vec3b>(i,j) = cv::Vec3b(0,255,0);
      }
      else if(value < 150)
      {
        binaryMap.at<uchar>(i,j) = 0;
        colorMap.at<cv::Vec3b>(i,j) = cv::Vec3b(255-floor(value),0,0);
      }
      else
      {
        binaryMap.at<uchar>(i,j) = 255;
        colorMap.at<cv::Vec3b>(i,j) = cv::Vec3b(255-floor(value),0,0);
      }
    }
  }

  cv::imwrite("/home/demo/color.png", colorMap);
  cv::imwrite("/home/demo/binary.png", binaryMap);

  // Use Hough transform to find longest line
  cv::Mat skel(binaryMap.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat tempI(binaryMap.size(), CV_8UC1);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
  bool done = false;
  while(!done)
  {
    cv::morphologyEx(binaryMap,tempI,cv::MORPH_OPEN,element);
    cv::bitwise_not(tempI,tempI);
    cv::bitwise_and(binaryMap,tempI,tempI);
    cv::bitwise_or(skel, tempI, skel);
    cv::erode(binaryMap,binaryMap,element);

    double max;
    cv::minMaxLoc(binaryMap, 0, &max);
    done = (0 == max);
  }
  //cv::Canny(binaryMap, dst, 50, 200, 3);
  cv::imwrite("/home/demo/skel.png", skel);

  std::vector<cv::Vec2f> lines;
  cv::HoughLines(skel, lines, 1, M_PI/(8*180), 40, 0, 0 );
  if(lines.size() > 0)
  {
    // first line in array should be "best" line
    mapRotation = lines[0][1];

    // Make sure rotation value is in first quadrant so sin and cos are positive
    while(mapRotation < 0.0)
      mapRotation += M_PI/2.0;
    while(mapRotation > M_PI/2.0)
      mapRotation -= M_PI/2.0;
  }
  else // we didn't find any lines and can't rotate map
  {
    ROS_ERROR("No lines found in map. Cannot rotate.");
    mapRotation = 0.0;
  }

  // write the lines to the dst image
  cv::Mat cdst;
  cv::cvtColor(skel,cdst,cv::COLOR_GRAY2BGR);
  for(int i = 0; i < lines.size() && i<2; i++)
  {
    float rho = lines[i][0];
    float theta = lines[i][1];
    cv::Point pt1,pt2;
    double a = cos(theta);
    double b = sin(theta);
    double x0 = a*rho;
    double y0 = b*rho;
    pt1.x = cvRound(x0 - 1000*b);
    pt1.y = cvRound(y0 + 1000*a);
    pt2.x = cvRound(x0 + 1000*b);
    pt2.y = cvRound(y0 - 1000*a);
    cv::line(cdst,pt1,pt2,cv::Scalar(0,0,255));
  }
  cv::imwrite("/home/demo/cdst.png", cdst);

  // if we aren't rotating, we don't need to calculate transforms
  if(fabs(mapRotation) < 0.00001)
  {
    bool tempT = broadcastTransform;
    broadcastTransform = false;
    broadcastPreviousTransform = false;
    previousTransform = transform;
    broadcastPreviousTransform = tempT;
    return;
  }
}

void CoarseGridConverter::rotateMap(nav_msgs::OccupancyGrid &grid, const double mapRotation)
{
  // Convert occupancy grid to an image. We use color channels for known/unknown areas.
  nav_msgs::MapMetaData mInfo = grid.info;
  cv::Mat colorMap = cv::Mat(mInfo.height, mInfo.width, CV_8UC3);

  // iterate over map, store in image
  // (0,0) is lower left corner of OccupancyGrid
  for(int i = 0; i < mInfo.height; i++) 
  {
    for(int j = 0; j < mInfo.width; j++) 
    {
      double value = grid.data[i*mInfo.width+j]*255/100;
      if(value < 0)
      {
        colorMap.at<cv::Vec3b>(i,j) = cv::Vec3b(0,255,0);
      }
      else
      {
        colorMap.at<cv::Vec3b>(i,j) = cv::Vec3b(255-floor(value),0,0);
      }
    }
  }
  cv::imwrite("/home/demo/color.png", colorMap);

  // Rotate image to be in direction of longest line
  cv::Point2f center = cv::Point( colorMap.cols/2.0, colorMap.rows/2.0 );
  cv::Mat rotTrans;
  rotTrans = cv::getRotationMatrix2D( center, mapRotation*180/M_PI, 1.0 );
  double cos = fabs(rotTrans.at<double>(0,0));
  double sin = fabs(rotTrans.at<double>(0,1));
  int nw = ((colorMap.rows*sin) + (colorMap.cols*cos));
  int nh = ((colorMap.rows*cos) + (colorMap.cols*sin));
  rotTrans.at<double>(0,2) += (nw/2) - center.x;
  rotTrans.at<double>(1,2) += (nh/2) - center.y;

  /// Rotate the warped image
  cv::Mat rotMap;
  //warpAffine( colorMap, rotMap, rotTrans, ibound.size() );
  warpAffine( colorMap, rotMap, rotTrans, cv::Size(nw,nh) );

  cv::imwrite("/home/demo/trans.png", rotMap);

  // Save old transform
  bool temp = broadcastTransform;
  broadcastPreviousTransform = false;
  previousTransform = transform;
  broadcastPreviousTransform = temp;

  std::stringstream ss;
  ss << "rotated_map" << transformCount;
  transform.child_frame_id_ = ss.str();
  transform.frame_id_ = grid.header.frame_id;

  // Convert image back to occupancy grid
  grid.header.frame_id = transform.child_frame_id_;
  grid.header.stamp = ros::Time::now();
  grid.info = mInfo;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.info.width = rotMap.cols;
  grid.info.height = rotMap.rows;
  grid.data.resize(grid.info.height*grid.info.width);

  // iterate over map, store in data
  // (0,0) is lower left corner of OccupancyGrid
  for(int i = 0; i < grid.info.height; i++){
    for(int j = 0; j < grid.info.width; j++){
      cv::Vec3b value = rotMap.at<cv::Vec3b>(i,j);
      if(value.val[1]>150)
        grid.data[i*grid.info.width+j] = -1;
      else
        grid.data[i*grid.info.width+j] = 100-floor(value.val[0]*100/255);
    }
  }

  // Calculate transform from map frame to new rotated frame
  tf::Transform dispTransform;
  tf::Transform rotateTransform;
  tf::Transform dispBack;
  tf::Quaternion q;
  q.setRPY(0,0,0);
  dispTransform.setRotation(q);
  dispTransform.setOrigin(tf::Vector3(mInfo.origin.position.x + center.x*mInfo.resolution, mInfo.origin.position.y + center.y*mInfo.resolution,0));

  rotateTransform.setOrigin(tf::Vector3(0,0,0));
  q.setRPY(0, 0, mapRotation);
  rotateTransform.setRotation(q);
  q.setRPY(0, 0, 0);
  //dispBack.setOrigin(tf::Vector3(-(grid.info.origin.position.x + fbound.width/2.0*grid.info.resolution), -(grid.info.origin.position.y + fbound.height/2.0*grid.info.resolution),0));
  dispBack.setOrigin(tf::Vector3(-(grid.info.origin.position.x + nw/2.0*grid.info.resolution), -(grid.info.origin.position.y + nh/2.0*grid.info.resolution),0));
  dispBack.setRotation(q);
  transform.setData(dispTransform * rotateTransform * dispBack);
  rotPub.publish(grid);
  broadcastTransform = true;
  transformCount++;
  if(transformCount > 10) // no need for the count to get really high
    transformCount = 1;
}

void CoarseGridConverter::updateTransform()
{
  if(broadcastTransform)
  {
    transform.stamp_ = ros::Time::now();
    broadcaster.sendTransform(transform);
  }
  if(previousTransform.child_frame_id_.compare(transform.child_frame_id_) != 0 && broadcastPreviousTransform)
  {
    previousTransform.stamp_ = ros::Time::now();
    broadcaster.sendTransform(previousTransform);
  }
}

full_coverage::SquareCell CoarseGridConverter::fillCell(int row, int col, int cw, int ch, double res, int op, geometry_msgs::Pose origin) {
  full_coverage::SquareCell temp_cell;
  double xl = origin.position.x + (col-cw)*res;  // This is really confusing. Added debug grid publishing from wall_planner to debug this (if its a problem again)
  double xr = origin.position.x + col*res;
  double yl = origin.position.y + row*res;
  double yh = origin.position.y + (row + ch)*res;
  // Top left corner
  temp_cell.tl_vertex.pose.position.y = yh;
  temp_cell.tl_vertex.pose.position.x = xl;
  // Top right corner
  temp_cell.tr_vertex.pose.position.y = yh;
  temp_cell.tr_vertex.pose.position.x = xr;
  // Bottom left corner
  temp_cell.bl_vertex.pose.position.y = yl;
  temp_cell.bl_vertex.pose.position.x = xl;
  // bottom right corner
  temp_cell.br_vertex.pose.position.y = yl;
  temp_cell.br_vertex.pose.position.x = xr;
  // Occupancy of the averaged pixels in the cell
  temp_cell.occupancy_prob = op;
  return temp_cell;
}  // fillCell


void CoarseGridConverter::printNewGrid() {
  int gridCols = navGrid.grid_cols;
  int gridRows = navGrid.grid_cells.size()/gridCols;

  cv::Mat coarseMap = cv::Mat(gridRows, gridCols, CV_8UC3);

  // iterate over map, store in image
  // (0,0) is lower left corner of OccupancyGrid
  for(int i = 0; i < gridRows; i++) {
    for(int j = 0; j < gridCols; j++) {
      int index = i*gridCols+j;
      full_coverage::SquareCell cell = navGrid.grid_cells[index];
      double value = cell.occupancy_prob*255.0/100.0;
      if(value < 0)
      {
        coarseMap.at<cv::Vec3b>(i,j) = cv::Vec3b(0,255,0);
      }
      else
      {
        coarseMap.at<cv::Vec3b>(i,j) = cv::Vec3b(255-floor(value),0,0);
      }
    }
  }

  cv::imwrite("/home/demo/coarse.png", coarseMap);
}  //printNewGrid


} // namespace full_coverage

int main(int argc, char **argv) {
  ros::init(argc, argv, "coarse_grid_converter");

  // Creates an instance of CoarseGridConverter
  full_coverage::CoarseGridConverter* sg = new full_coverage::CoarseGridConverter();

  ros::Rate rate(10.0);
  while(ros::ok())
  {
    sg->updateTransform();
    rate.sleep();
  }

  return 0; // success
} // main
