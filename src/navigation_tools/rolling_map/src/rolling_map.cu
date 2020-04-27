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

#define THREADS_PER_BLOCK 256

__device__ bool d_error;

__global__ void crs(float* fPoints, int* iPoints, int cloudSize, int maxRay, float* fStart, int* iStart, float* fStartVoxel, int* outPoints, int* outSizes, int* min, int* max, float resolution)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;

  // if there are extra threads, don't run them
  if(index >= cloudSize)
    return;

  // init vars
  bool done = false;

  // calculate normal vector in direction of sensor->point
  float direction[3] = {fPoints[index]-fStart[0], fPoints[cloudSize+index]-fStart[1], fPoints[2*cloudSize+index]-fStart[2]};
  float directionMagnitude = powf(powf(direction[0],2) + powf(direction[1],2) + powf(direction[2],2),0.5);

  // variables used for ray casting algorithm
  int stepDirection[3];        // +/- step in each cardinal direction
  float accumulatedError[3];   // error accumulated in each direction
  float deltaError[3];         // change in error accumulated for a step in a direction
  int currentIndex[3];         // for tracking the index as we trace
  int pointIndex[3];           // index of final occupied point
  bool usePI = true;          // we only check for the final point if it is on the map, 
                               // otherwise we are done when we leave the map

  // Set the starting position to the sensor position, and the final index
  for(int i = 0; i < 3; i++)
  {
    currentIndex[i] = iStart[i];
  }
  pointIndex[0] = iPoints[index];
  pointIndex[1] = iPoints[cloudSize+index];
  pointIndex[2] = iPoints[2*cloudSize+index];

  // If the occupied point is in the map, we use it as a stopping point
  if(pointIndex[0] < min[0] || pointIndex[0] > max[0] ||
     pointIndex[1] < min[1] || pointIndex[1] > max[1] ||
     pointIndex[2] < min[2] || pointIndex[2] > max[2])
    usePI = false;

  // check direction magnitude for divide by zero or same cell
  if(fabs(directionMagnitude) < resolution)
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

    float voxelBorder = fStartVoxel[dir] + stepDirection[dir]*resolution*0.5;
    accumulatedError[dir] = fdividef((voxelBorder - fStart[dir]),direction[dir]);
    deltaError[dir] = fdividef(resolution,fabs(direction[dir]));
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
      if(currentIndex[0] == pointIndex[0] &&
         currentIndex[1] == pointIndex[1] &&
         currentIndex[2] == pointIndex[2])
      {
        done = true;
      }
    }

    // if we are off the map, we are done. 
    if(currentIndex[0] < min[0] || currentIndex[0] > max[0] ||
       currentIndex[1] < min[1] || currentIndex[1] > max[1] ||
       currentIndex[2] < min[2] || currentIndex[2] > max[2])
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

bool castRays(float* fPoints, int* iPoints, int cloudSize, int maxRay, float* fStart, int* iStart, float* fStartVoxel, int* outPoints, int* outSizes, int minX, int maxX, int minY, int maxY, int minZ, int maxZ, float resolution) 
{
  // Device copies of three inputs and output, size of allocated memory, num of threads and blocks
  float *d_fPoints, *d_fStart, *d_fStartVoxel;
  int *d_iPoints, *d_outPoints, *d_iStart, *d_outSizes, *d_min, *d_max;
  int min[3] = {minX, minY, minZ};
  int max[3] = {maxX, maxY, maxZ};
  int thr, blk;
  bool h_error = false;
  int temp;
  for(int i = 0; i < 3; i ++)
  {
    if(min[i] > max[i])
    {
      temp = min[i];
      min[i] = max[i];
      max[i] = temp;
    }
  }
  //cudaMemset(&d_error,0,sizeof(bool));

  // Alloc memory for device copies of inputs and outputs
  cudaMalloc((void**)&d_fPoints, ((cloudSize*3) * sizeof(float)));
  cudaMalloc((void**)&d_iPoints, ((cloudSize*3) * sizeof(int)));
  cudaMalloc((void**)&d_fStart, (3 * sizeof(float)));
  cudaMalloc((void**)&d_iStart, (3 * sizeof(int)));
  cudaMalloc((void**)&d_fStartVoxel, (3 * sizeof(float)));
  cudaMalloc((void**)&d_min, (3 * sizeof(int)));
  cudaMalloc((void**)&d_max, (3 * sizeof(int)));
  cudaMalloc((void**)&d_outPoints, ((cloudSize*maxRay*3) * sizeof(int)));
  cudaMalloc((void**)&d_outSizes, (cloudSize * sizeof(int)));

  // Copy inputs to device
  cudaMemcpy(d_fPoints, fPoints, ((cloudSize*3) * sizeof(float)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_iPoints, iPoints, ((cloudSize*3) * sizeof(int)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_fStart, fStart, (3 * sizeof(float)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_iStart, iStart, (3 * sizeof(int)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_fStartVoxel, fStartVoxel, (3 * sizeof(float)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_min, min, (3 * sizeof(int)), cudaMemcpyHostToDevice);
  cudaMemcpy(d_max, max, (3 * sizeof(int)), cudaMemcpyHostToDevice);

  // Calculates blocks and threads and launch average3 kernel on GPU
  thr=THREADS_PER_BLOCK;
  blk=cloudSize/THREADS_PER_BLOCK+1;
  crs<<<blk,thr>>>(d_fPoints, d_iPoints, cloudSize, maxRay, d_fStart, d_iStart, d_fStartVoxel, d_outPoints, 
                   d_outSizes, d_min, d_max, resolution);

  // Wait for the GPU to finish
  cudaDeviceSynchronize();

  //// Copy result back to host and cleanup
  cudaMemcpy(outPoints, d_outPoints, (cloudSize*maxRay*3) * sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpy(outSizes, d_outSizes, cloudSize * sizeof(int), cudaMemcpyDeviceToHost);
  //cudaMemcpyFromSymbol(&h_error, "d_error", sizeof(bool), 0, cudaMemcpyDeviceToHost);
  cudaFree(d_outSizes); 
  cudaFree(d_outPoints);  
  cudaFree(d_max);  
  cudaFree(d_min);  
  cudaFree(d_fStartVoxel); 
  cudaFree(d_iStart); 
  cudaFree(d_fStart); 
  cudaFree(d_iPoints); 
  cudaFree(d_fPoints);
  return !h_error;
}
