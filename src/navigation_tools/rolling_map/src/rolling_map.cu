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
  idx.x = (p.x - c_min_x)/c_resolution;
  idx.y = (p.y - c_min_y)/c_resolution;
  idx.z = (p.z - c_min_z)/c_resolution;
  return idx;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ void markVoxel(char* voxel_grid, int x, int y, int z){
  size_t flat_idx = c_width*c_width*z + c_width*y + x;
  char* byte = voxel_grid + flat_idx/8;
  char bit_mask = flat_idx % 8;
  *byte |= bit_mask;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ void freeVoxel(char* voxel_grid, int x, int y, int z){
  size_t flat_idx = c_width*c_width*z + c_width*y + x;
  char* byte = voxel_grid + flat_idx/8;
  char bit_mask = flat_idx % 8;
  *byte &= ~bit_mask;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ bool d_error;

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__global__ void crs(pcl::PointXYZ* fPoints, int cloudSize, int maxRay, const pcl::PointXYZ* sensor_origin, float* fStartVoxel, int* outPoints, int* outSizes, char* voxel_grid)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;

  // if there are extra threads, don't run them
  if(index >= cloudSize)
    return;

  // init vars
  bool done = false;

  // Get the information about the current point
  pcl::PointXYZ& point = fPoints[index];
  rolling_map::Coord point_idx = toIndex(point);
  rolling_map::Coord start_idx = toIndex(*sensor_origin); 

  // calculate normal vector in direction of sensor->point
  float direction[3] = {point.x-sensor_origin->x, point.y-sensor_origin->y, point.z-sensor_origin->z};
  float directionMagnitude = powf(powf(direction[0],2) + powf(direction[1],2) + powf(direction[2],2),0.5);

  // variables used for ray casting algorithm
  int stepDirection[3];        // +/- step in each cardinal direction
  float accumulatedError[3];   // error accumulated in each direction
  float deltaError[3];         // change in error accumulated for a step in a direction
  int currentIndex[3];         // for tracking the index as we trace
  bool usePI = true;          // we only check for the final point if it is on the map, 
                               // otherwise we are done when we leave the map

  // Set the starting position to the sensor position, and the final index
  for(int i = 0; i < 3; i++)
  {
    currentIndex[i] = ((int*)(&start_idx))[i];
  }

  // If the occupied point is in the map, we use it as a stopping point
  if(point_idx.x < 0 || point_idx.x > c_width ||
     point_idx.y < 0 || point_idx.y > c_width ||
     point_idx.z < 0 || point_idx.z > c_height)
    usePI = false;

  // check direction magnitude for divide by zero or same cell
  if(fabs(directionMagnitude) < c_resolution)
  {
    d_error = true;
    return;
  }

  // set up initial values in each direction
  for(int dir = 0; dir < 3; dir++)
  {
    direction[dir] = fdividef(direction[dir],directionMagnitude);
    if(direction[dir] > 0.0)
      stepDirection[dir] = 1;
    else if(direction[dir] < 0.0)
      stepDirection[dir] = -1;

    float voxelBorder = fStartVoxel[dir] + stepDirection[dir]*c_resolution*0.5;
    accumulatedError[dir] = fdividef((voxelBorder - ((float*)sensor_origin)[dir]),direction[dir]);
    deltaError[dir] = fdividef(c_resolution,fabs(direction[dir]));
  }
  
  int count = 0;
  // loop until we are out of map bounds
  while(!done)
  {
    // find direction of min error
    int dim = 2;
    if(fabs(accumulatedError[0]) < fabs(accumulatedError[1]) && fabs(accumulatedError[0]) < fabs(accumulatedError[2]))
      dim = 0;
    else if(fabs(accumulatedError[1]) < fabs(accumulatedError[0]) && fabs(accumulatedError[1]) < fabs(accumulatedError[2]))
      dim = 1;

    // advance in direction of min error
    currentIndex[dim] = currentIndex[dim] + stepDirection[dim];
    accumulatedError[dim] = accumulatedError[dim] + deltaError[dim]; 

    // done if we are at occ point
    if(usePI)
    {
      if(currentIndex[0] == point_idx.x &&
         currentIndex[1] == point_idx.y &&
         currentIndex[2] == point_idx.z)
      {
        done = true;
      }
    }

    // if we are off the map, we are done. 
    if(currentIndex[0] < 0 || currentIndex[0] > c_width ||
       currentIndex[1] < 0 || currentIndex[1] > c_width ||
       currentIndex[2] < 0 || currentIndex[2] > c_height)
    {
      done = true;
    }
      
    //otherwise we mark the current index as unoccupied
    if(!done)
    {
      outPoints[index*maxRay+count] = currentIndex[0];
      outPoints[index*maxRay+(cloudSize*maxRay)+count] = currentIndex[1];
      outPoints[index*maxRay+(2*cloudSize*maxRay)+count] = currentIndex[2];
    }
    count = count + 1;
  }
  outSizes[index] = count;
  return;
}

// ================================================================================================
// ================================================================================================
// ================================================================================================

bool rolling_map::RollingMap::castRays(const std::vector<pcl::PointXYZ>& points, int cloudSize, int maxRay, const pcl::PointXYZ& sensor_origin, float* fStartVoxel, int* outPoints, int* outSizes) 
{
  // Device copies of three inputs and output, size of allocated memory, num of threads and blocks
  pcl::PointXYZ *d_fPoints, *d_fStart;
  float *d_fStartVoxel;
  int *d_outPoints, *d_iStart, *d_outSizes, *d_min, *d_max;
  int thr, blk;
  bool h_error = false;
  int temp;
  cudaMemset(&d_error,0,sizeof(bool));

  // Copy constant values over to device
  cudaMemcpyToSymbol(c_resolution, &this->resolution, sizeof(resolution));
  cudaMemcpyToSymbol(c_min_x, &this->minXP, sizeof(float));
  cudaMemcpyToSymbol(c_min_y, &this->minYP, sizeof(float));
  cudaMemcpyToSymbol(c_min_z, &this->z0,    sizeof(float));

  // Alloc memory for device copies of inputs and outputs
  cudaMalloc((void**)&d_fPoints, ((cloudSize) * sizeof(pcl::PointXYZ)));
  cudaMalloc((void**)&d_fStart, (sizeof(pcl::PointXYZ)));
  cudaMalloc((void**)&d_fStartVoxel, (3 * sizeof(float)));
  cudaMalloc((void**)&d_outPoints, ((cloudSize*maxRay*3) * sizeof(int)));
  cudaMalloc((void**)&d_outSizes, (cloudSize * sizeof(int)));

  // Copy inputs to device
  cudaMemcpy(d_fPoints, points.data(), ((cloudSize) * sizeof(pcl::PointXYZ)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_fStart, &sensor_origin, (sizeof(pcl::PointXYZ)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_fStartVoxel, fStartVoxel, (3 * sizeof(float)), cudaMemcpyHostToDevice);

  // Calculates blocks and threads and launch average3 kernel on GPU
  thr=THREADS_PER_BLOCK;
  blk=cloudSize/THREADS_PER_BLOCK+1;
  crs<<<blk,thr>>>(d_fPoints, cloudSize, maxRay, d_fStart, d_fStartVoxel, d_outPoints, 
                   d_outSizes, d_voxel_grid_);

  // Wait for the GPU to finish
  cudaDeviceSynchronize();

  //// Copy result back to host and cleanup
  cudaMemcpy(outPoints, d_outPoints, (cloudSize*maxRay*3) * sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpy(outSizes, d_outSizes, cloudSize * sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpyFromSymbol(&h_error, "d_error", sizeof(bool), 0, cudaMemcpyDeviceToHost);
  cudaFree(d_outSizes); 
  cudaFree(d_outPoints);   
  cudaFree(d_fStartVoxel); 
  cudaFree(d_fStart); 
  cudaFree(d_fPoints);
  return !h_error;
}
