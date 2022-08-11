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
#include "cuda_safe.cuh"
#include "cuda_voxel_grid.cuh"

#define THREADS_PER_BLOCK 256

// Branchless sign function
#define SIGN(x) ( ((x) >= 0) - ((x) < 0) )

// Easy square function
#define SQ(x) ((x)*(x))

// Index a pcl::PointXYZ like a float[4]
#define INDEX(point, idx) ((float*)(&point))[idx]

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ int minErrorDirection(const float* error){
  if (fabs(error[0]) < fabs(error[1]) && fabs(error[0]) < fabs(error[2]))
    return 0;
  else if (fabs(error[0]) > fabs(error[1]) && fabs(error[1]) < fabs(error[2]))
    return 1;

  return 2;
}


// DEVICE FUNCTIONS
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// KERNEL

__global__ void crs(pcl::PointXYZ* pointcloud, int cloudSize, const pcl::PointXYZ sensor_origin, const pcl::PointXYZ start_voxel_loc, rolling_map::cudaVoxelGrid* voxel_grid)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;

  // if there are extra threads, don't run them
  if(index >= cloudSize)
    return;

  // Get the information about the current and starting point
  const pcl::PointXYZ& point = pointcloud[index];

  rolling_map::Coord point_idx     = voxel_grid->toIndex(point);
  rolling_map::Coord current_index = voxel_grid->toIndex(sensor_origin); 

  // calculate normal vector in direction of sensor->point
  float direction[3] = {point.x-sensor_origin.x, point.y-sensor_origin.y, point.z-sensor_origin.z};
  float directionMagnitude = sqrt(SQ(direction[0]) + SQ(direction[1]) + SQ(direction[2]));

  // variables used for ray casting algorithm
  int   stepDirection[3];      // +/- step in each cardinal direction
  float accumulatedError[3];   // error accumulated in each direction
  float deltaError[3];         // change in error accumulated for a step in a direction

  // check direction magnitude for divide by zero or same cell
  if(fabs(directionMagnitude) < voxel_grid->resolution) return;

  // set up initial values in each direction
  for(int dir = 0; dir < 3; dir++)
  {
    direction[dir]    /= directionMagnitude;
    stepDirection[dir] = SIGN(direction[dir]);

    float voxelBorder     = INDEX(start_voxel_loc, dir) + stepDirection[dir]*voxel_grid->resolution*0.5;
    accumulatedError[dir] = (voxelBorder - INDEX(sensor_origin, dir)) / direction[dir];
    deltaError[dir]       = voxel_grid->resolution/fabs(direction[dir]);
  }
  
  bool done = false;
  while(!done)
  {
    // Find direction of min error
    int dim = minErrorDirection(accumulatedError);

    // Advance in direction of min error
    ((int*)&current_index)[dim] += stepDirection[dim];
    accumulatedError[dim]       += deltaError[dim]; 

    // Determine if we are done
    done = (current_index == point_idx) || voxel_grid->offGrid(current_index);

    // Otherwise we mark the current index as unoccupied
    if(!done)
      voxel_grid->freeVoxel(current_index);
  }

  if (!voxel_grid->offGrid(point_idx)){
    voxel_grid->markVoxel(point_idx);
  }

}

// DEVICE CODE
// ================================================================================================
// ================================================================================================
// ================================================================================================
// HOST CODE

bool rolling_map::RollingMap::castRays(const std::vector<pcl::PointXYZ>& points, const pcl::PointXYZ& sensor_origin, const pcl::PointXYZ& start_voxel_loc) 
{
  // Device copies of three inputs and output, size of allocated memory, num of threads and blocks
  pcl::PointXYZ *d_pointcloud;

  // Alloc memory for the pointcloud on the device
  size_t cloudSize       = points.size();
  size_t pointcloud_size = cloudSize * sizeof(pcl::PointXYZ);
  CUDA_SAFE(cudaMalloc(&d_pointcloud, pointcloud_size));

  // Copy inputs to device
  CUDA_SAFE(cudaMemcpy(d_pointcloud, points.data(), pointcloud_size, cudaMemcpyHostToDevice));

  // Calculates blocks and threads and launch average3 kernel on GPU
  if (cuda_ok){
    int thr=THREADS_PER_BLOCK;
    int blk=cloudSize/THREADS_PER_BLOCK+1;
    crs<<<blk,thr>>>(d_pointcloud, cloudSize, sensor_origin, start_voxel_loc, d_voxel_grid_);

    // Wait for the GPU to finish
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError() /*cast rays*/);
  }

  cudaFree(d_pointcloud);

  // Determine if the operation was successful, then reset the flag
  bool all_good = cuda_ok;
  cuda_ok = true;
  return all_good;
}
