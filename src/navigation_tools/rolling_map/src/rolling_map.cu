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

#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <rolling_map.h>

#define THREADS_PER_BLOCK 256

// Branchless sign function
#define SIGN(x) ( ((x) > 0) - ((x) < 0) )

// Easy square function
#define SQ(x) ((x)*(x))

// Index a pcl::PointXYZ like a float[4]
#define INDEX(point, idx) ((float*)(&point))[idx]

// Warpper for CUDA commands
#define CUDA_SAFE(command)                                                                                                                 \
if (cuda_ok){                                                                                                                              \
    cudaError_t err = command;                                                                                                             \
    cuda_ok = cuda_ok && (err == cudaSuccess);                                                                                             \
    if (not cuda_ok) {                                                                                                                     \
        const char* err_s = cudaGetErrorString(err);                                                                                       \
        ROS_WARN("Cuda Error [%d]\nFailed Command: \"%s\"\nError: %s\nAt line %d of %s", (int)(err), #command, err_s, __LINE__, __FILE__); \
    }                                                                                                                                      \
}

// Device constants 
__constant__ int c_width;
__constant__ int c_height;
__constant__ float c_resolution;
__constant__ float c_min_x;
__constant__ float c_min_y;
__constant__ float c_min_z;

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ rolling_map::Coord toIndex(const pcl::PointXYZ& p){
  rolling_map::Coord idx;
  const float one_over_res = 1/c_resolution;
  idx.x = (p.x - c_min_x)*one_over_res;
  idx.y = (p.y - c_min_y)*one_over_res;
  idx.z = (p.z - c_min_z)*one_over_res;
  return idx;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ void markVoxel(char* voxel_grid, const rolling_map::Coord& c){
  size_t flat_idx = c_width*c_width*c.z + c_width*c.y + c.x;
  char* byte      = voxel_grid + flat_idx/8;
  char  bit_mask  = flat_idx % 8;
  *byte |= bit_mask;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ void freeVoxel(char* voxel_grid, const rolling_map::Coord& c){
  size_t flat_idx = c_width*c_width*c.z + c_width*c.y + c.x;
  char* byte      = voxel_grid + flat_idx/8;
  char  bit_mask  = flat_idx % 8;
  *byte &= ~bit_mask;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ bool offGrid(const rolling_map::Coord& coord){
  return (coord.x < 0 || coord.x >= c_width ||
          coord.y < 0 || coord.y >= c_width ||
          coord.z < 0 || coord.z >= c_height );
}


// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__global__ void crs(pcl::PointXYZ* pointcloud, int cloudSize, int maxRay, const pcl::PointXYZ sensor_origin, const pcl::PointXYZ start_voxel_loc, int* outPoints, int* outSizes, char* voxel_grid)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;

  // if there are extra threads, don't run them
  if(index >= cloudSize)
    return;

  // Get the information about the current and starting point
  const pcl::PointXYZ& point = pointcloud[index];

  rolling_map::Coord point_idx     = toIndex(point);
  rolling_map::Coord current_index = toIndex(sensor_origin); 

  // calculate normal vector in direction of sensor->point
  float direction[3] = {point.x-sensor_origin.x, point.y-sensor_origin.y, point.z-sensor_origin.z};
  float directionMagnitude = sqrt(SQ(direction[0]) + SQ(direction[1]) + SQ(direction[2]));

  // variables used for ray casting algorithm
  int   stepDirection[3];      // +/- step in each cardinal direction
  float accumulatedError[3];   // error accumulated in each direction
  float deltaError[3];         // change in error accumulated for a step in a direction

  // If the occupied point is in the map, we use it as a stopping point
  bool usePI = not offGrid(point_idx);

  // check direction magnitude for divide by zero or same cell
  if(fabs(directionMagnitude) < c_resolution) return;

  // set up initial values in each direction
  for(int dir = 0; dir < 3; dir++)
  {
    direction[dir]    /= directionMagnitude;
    stepDirection[dir] = SIGN(direction[dir]);

    float voxelBorder     = INDEX(start_voxel_loc, dir) + stepDirection[dir]*c_resolution*0.5;
    accumulatedError[dir] = (voxelBorder - INDEX(sensor_origin, dir)) / direction[dir];
    deltaError[dir]       = c_resolution/fabs(direction[dir]);
  }
  
  int count = 0;
  bool done = false;
  while(!done && count < maxRay)
  {
    // Find direction of min error
    int dim = 2;
    if(fabs(accumulatedError[0]) < fabs(accumulatedError[1]) && fabs(accumulatedError[0]) < fabs(accumulatedError[2])){
      dim = 0;
    }
    else if(fabs(accumulatedError[1]) < fabs(accumulatedError[0]) && fabs(accumulatedError[1]) < fabs(accumulatedError[2])){
      dim = 1;
    }

    // Advance in direction of min error
    ((int*)&current_index)[dim] += stepDirection[dim];
    accumulatedError[dim]       += deltaError[dim]; 

    // Determine if we are done
    done = (usePI && (current_index == point_idx)) || offGrid(current_index);

    // Otherwise we mark the current index as unoccupied
    if(!done)
    {
      outPoints[index*maxRay+count]                      = current_index.x;
      outPoints[index*maxRay+(cloudSize*maxRay)+count]   = current_index.y;
      outPoints[index*maxRay+(2*cloudSize*maxRay)+count] = current_index.z;
      // freeVoxel(voxel_grid, current_index);
    }
    count = count + 1;
  }

  // if (!offGrid(point_idx)){
  //   markVoxel(voxel_grid, point_idx);
  // }

  outSizes[index] = count;
  return;
}

// ================================================================================================
// ================================================================================================
// ================================================================================================

bool rolling_map::RollingMap::castRays(const std::vector<pcl::PointXYZ>& points, int maxRay, const pcl::PointXYZ& sensor_origin, const pcl::PointXYZ& start_voxel_loc, int* outPoints, int* outSizes) 
{
  // Device copies of three inputs and output, size of allocated memory, num of threads and blocks
  pcl::PointXYZ *d_pointcloud;
  int *d_outPoints, *d_outSizes;

  // To determine if the kernel ran successfully
  bool h_error = false;

  // Copy constant values over to device
  CUDA_SAFE(cudaMemcpyToSymbol(c_resolution, &this->resolution, sizeof(float)));
  CUDA_SAFE(cudaMemcpyToSymbol(c_min_x,      &this->x0,         sizeof(float)));
  CUDA_SAFE(cudaMemcpyToSymbol(c_min_y,      &this->y0,         sizeof(float)));
  CUDA_SAFE(cudaMemcpyToSymbol(c_min_z,      &this->z0,         sizeof(float)));
  CUDA_SAFE(cudaMemcpyToSymbol(c_width,      &this->width,      sizeof(int)));
  CUDA_SAFE(cudaMemcpyToSymbol(c_height,     &this->height,     sizeof(int)));

  // Alloc memory for device copies of inputs and outputs
  size_t cloudSize       = points.size();
  size_t pointcloud_size = cloudSize * sizeof(pcl::PointXYZ);
  size_t outsizes_size   = cloudSize * sizeof(int);
  size_t outpoints_size  = cloudSize * sizeof(int) * maxRay * 3;
  CUDA_SAFE(cudaMalloc(&d_pointcloud, pointcloud_size));
  CUDA_SAFE(cudaMalloc(&d_outSizes,   outsizes_size));
  CUDA_SAFE(cudaMalloc(&d_outPoints,  outpoints_size));

  // Copy inputs to device
  CUDA_SAFE(cudaMemcpy(d_pointcloud, points.data(), pointcloud_size, cudaMemcpyHostToDevice));

  // Calculates blocks and threads and launch average3 kernel on GPU
  if (cuda_ok){
    int thr=THREADS_PER_BLOCK;
    int blk=cloudSize/THREADS_PER_BLOCK+1;
    crs<<<blk,thr>>>(d_pointcloud, cloudSize, maxRay, sensor_origin, start_voxel_loc, d_outPoints, d_outSizes, d_voxel_grid_);

    // Wait for the GPU to finish
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError());
  }

  // Copy result back to host and cleanup
  CUDA_SAFE(cudaMemcpy(outPoints, d_outPoints, outpoints_size, cudaMemcpyDeviceToHost));
  CUDA_SAFE(cudaMemcpy(outSizes,  d_outSizes,  outsizes_size,  cudaMemcpyDeviceToHost));

  cudaFree(d_outSizes); 
  cudaFree(d_outPoints);   
  cudaFree(d_pointcloud);

  bool error = cuda_ok;
  cuda_ok = true;
  return error;
}
