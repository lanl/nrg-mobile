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
* Author: Alex von Sternberg
*********************************************************************/

#ifndef _ROLLING_MAP_H_
#define _ROLLING_MAP_H_

#include <unordered_set>
#include <string>
#include <vector>
#include <atomic>
#include "boost/thread/shared_mutex.hpp"
#include "pcl_ros/point_cloud.h"
#include "coord.h"

#ifdef TIMEIT
#include "cpp_timer/Timer.h"
#endif

#define cuda_ptr

#ifdef USE_CUDA
#include <cuda_runtime.h>
#define CUDA_ONLY(x) x
#define CUDA_BOTH __host__ __device__
#else
#define CUDA_ONLY(x)
#define CUDA_BOTH
#endif

namespace rolling_map
{

// Forward declaration
class cudaVoxelGrid;

typedef std::unordered_set<Coord, HashCoord, CompareCoord> CellMap;

/**
* Rolling Array construct
*/
class RollingMap 
{
private:
  //BoolArray boolArray;    // Occupancy array whose size if fixed in constructor
  CellMap map;            // Unordered map of tracked occupied cells
  const int width;              // Number of grid cells in x and y directions
  const int height;             // Number of grid cells in z direction
  const float resolution;      // Grid resoltuion (m/cell)
  float xPosition;       // Current x position of the robot in (m)
  float yPosition;       // Current y position of the robot in (m)
  float x0;
  float y0;
  float z0;              // Height of z[0] cells in array (m)
  float minXP;            // x position of lower left hand corner of grid
  float minYP;            // y position of lower left hand corner of grid
  int minXI;
  int minYI;
  boost::shared_mutex mapMutex;        // Mutex for thread safety when we translate the map
  boost::shared_mutex translateMutex;  // Mutex for thread safety when we translate the map
  cuda_ptr cudaVoxelGrid* d_voxel_grid_;
  uint8_t* d_voxel_data_;
  bool cuda_ok = true;

  void clearX(int shift);
  void clearY(int shift);
  void position(int ix, int iy, int iz, float &px, float &py, float &pz);
  void index(float px, float py, float pz, int &ix, int &iy, int &iz);
#ifndef USE_CUDA
  void castRay(const pcl::PointXYZ &occPoint, const pcl::PointXYZ &sensorOrigin, CellMap &free);
#else
  bool castRays(const std::vector<pcl::PointXYZ>& points, const pcl::PointXYZ& sensor_origin, const pcl::PointXYZ& start_voxel_loc);
  bool cudaInit();
#endif
  Coord newCoord(int x, int y, int z);
  float getMinXP();
  float getMaxXP();
  float getMinYP();
  float getMaxYP();
  int getMinXI();
  int getMaxXI();
  int getMinYI();
  int getMaxYI();
  float getXPosition();
  float getYPosition();
#ifndef USE_CUDA
  void setFree(CellMap &free);
#else
  void setFree(int cloudSize, int maxRay, int* rayPoints, int* raySizes);
#endif
  void setOccupied(CellMap &free);

  /*
    Get occupancy value at (x,y,z) coord (meters)
  */
  bool atPosition(float x, float y, float z, bool &value);

  /*
    Get occupancy value at (x,y,z) index
  */
  bool atIndex(int x, int y, int z, bool &value);

  /*
    Set occupancy value at (x,y,z) coord (meters)
  */
  bool setPosition(float x, float y, float z, bool value);
  
  /*
    Set occupancy value at (x,y,z) index
  */
  bool setIndex(int x, int y, int z, bool value);

  /* 
    Check if (x,y,z) indices are in the bounds of the array
  */
  bool checkIndex(int x, int y, int z);

  /*
    Check if (x,y,z) position is in bounds of array
  */
  bool checkPosition(float x, float y, float z);

  /*
    Clear the area contained within the given x-y index polygon and z1-z2 coordinate heights
  */
  bool clearIndexBox(std::vector<std::vector<int>> polygon, int z1, int z2);
  bool pointInPoly(std::vector<int> point, std::vector<std::vector<int>> poly);

  /*
    Get the position of the cell at the supplied index
  */
  bool toPosition(int ix, int iy, int iz, float &px, float &py, float &pz);

  /*
    Get the index of the cell that contains the supplied position
  */
  bool toIndex(float px, float py, float pz, int &ix, int &iy, int &iz);

public:
  RollingMap(int w, int h, float res, float x, float y, float zmin);
  ~RollingMap();
  
  /////////////////////////////
  // Update functions
  /////////////////////////////
  /*
    Set the current position of the robot
  */
  void updatePosition(float x, float y);

  /////////////////////////////
  // Array access functions
  /////////////////////////////
  /*
    Clear the grid
  */
  void clearAll();

  /*
    Clear the area contained within the given x-y polygon and z1-z2 height
  */
  bool clearPositionBox(std::vector<std::vector<float>> polygon, float z1, float z2);

  /////////////////////////////
  //  Ray casting and point cloud insertion
  /////////////////////////////
  void insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin);
  void getMap(std::vector<pcl::PointXYZ> &mapcloud, int &minxi, int &minyi, float &minxp, float &minyp);

  /////////////////////////////
  // Getter functions
  /////////////////////////////
  int getWidth();
  int getHeight();
  float getResolution();
  float getMinZP();
  float getMaxZP();
  int getMinZI();
  int getMaxZI();

  #ifdef TIMEIT
  std::unique_ptr<cpp_timer::Timer> timer;
  #else
  private: std::unique_ptr<char> decoy_;
  #endif

}; // RollingMap

} // namespace rolling_map

#endif // _ROLLING_MAP_H_
