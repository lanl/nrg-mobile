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

#include <ros/ros.h>
#include <unordered_set>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include "boost/thread/shared_mutex.hpp"
#include "pcl/point_types.h"
#include "coord.h"

#ifdef TIMEIT
#include "cpp_timer/Timer.h"
#endif

#define cuda_ptr

#ifdef USE_CUDA
#include <cuda_runtime.h>
#endif
#include "cuda_safe.cuh"

#include "cuda_voxel_grid.cuh"

namespace rolling_map
{

struct ProbabilityModel{
  float maximum;
  float threshold;
  float hit_miss_ratio;
};

// Forward declaration
class cudaVoxelGrid;

// Typedef for the set of occupied points in the map
typedef std::unordered_set<Coord> CellMap;

/**
* Rolling Array construct
*/
class RollingMap 
{
private:
  CellMap map_;                   // Unordered map of tracked occupied cells
  const int width_;               // Number of grid cells in x and y directions
  const int height_;              // Number of grid cells in z direction
  const float resolution_;        // Grid resoltuion (m/cell)
  float xPosition_;               // Current x position of the robot in (m)
  float yPosition_;               // Current y position of the robot in (m)
  float x0_;                      // x position of lower left hand corner of grid
  float y0_;                      // y position of lower left hand corner of grid
  float z0_;                      // Height of z[0] cells in array (m)

  std::shared_timed_mutex map_mutex_;        // Mutex for thread safety when we translate the map

  // Probability modelling
  ProbabilityModel probability_model;

  // CUDA variables
  cuda_ptr cudaVoxelGrid* d_voxel_grid_ = nullptr;  // Address of the cudaVoxelGrid object on the GPU
  cuda_ptr voxel_block_t* d_voxel_data_ = nullptr;  // Address of the voxel grid data structure on the GPU
  size_t voxel_grid_size_bytes_ = 0;
  bool cuda_ok = true;

  /*
    Get the position of the index coordinate (ix,iy,iz)
    Not thread safe
  */
  void position(int ix, int iy, int iz, float &px, float &py, float &pz);

  /*
    Get the index of the position coordinate (px,py,pz)
    Not thread safe
  */
  void index(float px, float py, float pz, int &ix, int &iy, int &iz);

  /////////////////////////////
  // CUDA-Only functions
  /////////////////////////////
#ifdef USE_CUDA

  /*
    Initialize the voxel grid on the device and store its address
    Returns false if there are any errors in CUDA API calls
    Not thread safe
  */
  bool cudaInit(ProbabilityModel model);

  /*
    Launch a kernel to cast rays for each point in the points vector
    Returns false if there are any errors in CUDA API calls
    Not thread safe
  */
  bool castRays(const std::vector<pcl::PointXYZ>& points, const pcl::PointXYZ& sensor_origin, const pcl::PointXYZ& start_voxel_loc);

  /////////////////////////////
  // Non CUDA-Only functions
  /////////////////////////////
#else

  /*
    Remove any points which are within shift indices of the map x boundary
    Not thread safe
  */
  void clearX(int shift);

  /*
    Remove any points which are within shift indices of the map y boundary
    Not thread safe
  */
  void clearY(int shift);

  /*
    Cast a ray from the sensor origin to an occupied point, and store all unoccupied points in "free"
    Not thread safe
  */
  void castRay(const pcl::PointXYZ &occPoint, const pcl::PointXYZ &sensorOrigin, CellMap &free);

  /*
    For all points in "free", mark as unoccupied in the map
    Not thread safe
  */
  void setFree(CellMap &free);

  /*
    For all points in "occ", mark as occupied in the map
    Not thread safe
  */
  void setOccupied(CellMap &occ);

#endif

  /*
    Clear the area contained within the given x-y index polygon and z1-z2 coordinate heights
    Not thread safe
  */
  bool clearIndexBox(std::vector<std::vector<int>> polygon, int z1, int z2);

  /*
    Determine if a point is inside the polygon descriped by the vertices in "poly"
  */
  bool pointInPoly(std::vector<int> point, std::vector<std::vector<int>> poly);

public:
  RollingMap(int w, int h, float res, float x, float y, float zmin, ProbabilityModel model);
  ~RollingMap();
  
  /////////////////////////////
  // Update functions
  /////////////////////////////
  /*
    Set the current position of the robot
    Thread safe
  */
  void updatePosition(float x, float y);

  /*
    Set occupancy value at (x,y,z) coord (meters)
    Thread safe
  */
  bool setPosition(float x, float y, float z, bool value);

  /*
    Set occupancy value at (x,y,z) index
    Thread safe
  */
  bool setIndex(int x, int y, int z, bool value);

  /*
    Clear the grid
    Thread safe
  */
  void clearAll();

  /*
    Clear the area contained within the given x-y polygon and z1-z2 height
    Thread safe
  */
  bool clearPositionBox(std::vector<std::vector<float>> polygon, float z1, float z2);

  /////////////////////////////
  // Array access functions
  /////////////////////////////

  /*
    Get occupancy value at (x,y,z) coord (meters)
    Thead safe
  */
  bool atPosition(float x, float y, float z, bool &value);

  /*
    Get occupancy value at (x,y,z) index
    Thread safe
  */
  bool atIndex(int x, int y, int z, bool &value);

  /* 
    Check if (x,y,z) indices are in the bounds of the array
    Not thead safe
  */
  bool checkIndex(int x, int y, int z);

  /*
    Check if (x,y,z) position is in bounds of array
    Not thead safe
  */
  bool checkPosition(float x, float y, float z);

  /*
    Get the position of the cell at the supplied index
    Returns false is the index is outside of the map bounds
    Not thead safe
  */
  bool toPosition(int ix, int iy, int iz, float &px, float &py, float &pz);

  /*
    Get the index of the cell that contains the supplied position
    Returns false is the point is outside of the map bounds
    Not thread safe
  */
  bool toIndex(float px, float py, float pz, int &ix, int &iy, int &iz);

  /////////////////////////////
  //  Ray casting and point cloud insertion
  /////////////////////////////

  /*
    For each point in scancloud, cast a ray from sensorOrigin to the point and update occupancy values
    Thread safe
  */ 
  void insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin);

  /*
    Get a vector of all occupied points within the map bounds
    Thread safe
  */
  std::vector<Coord> getMap();

  /////////////////////////////
  // Getter functions
  /////////////////////////////

  int getWidth();
  int getHeight();
  float getResolution();
  float getMinZP();
  float getMaxZP();
  float getMinXP();
  float getMaxXP();
  float getMinYP();
  float getMaxYP();
  int getMaxXI();
  int getMaxYI();
  int getMaxZI();
  float getXPosition();
  float getYPosition();

  /////////////////////////////
  // Benchmark Timer
  /////////////////////////////

  #ifdef TIMEIT
  std::unique_ptr<cpp_timer::Timer> timer;
  #else
  // Even if not using the timer, we need to keep the class size the same
  private: std::unique_ptr<char> decoy_;
  #endif

}; // RollingMap

} // namespace rolling_map

#endif // _ROLLING_MAP_H_
