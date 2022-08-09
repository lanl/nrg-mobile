/*********************************************************************
*
*  © (or copyright) 2022. Triad National Security, LLC.
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
* Author: Alex Navarro
*********************************************************************/

#include "coord.h"
#include "rolling_map.h"

#include "cuda_safe.cuh"
#include "cuda_voxel_grid.cuh"

#include <cuda_runtime.h>

#define THREADS_PER_BLOCK 1024
#define X_CHANGE 1
#define Y_CHANGE 2

#define SIGN(x) ( ((x)>0) - ((x)<0) )

__global__ void updateMetaData(rolling_map::cudaVoxelGrid* voxel_grid, float min_x, float min_y){
  voxel_grid->min_x = min_x;
  voxel_grid->min_y = min_y;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__global__ void translateMap(rolling_map::cudaVoxelGrid* voxel_grid, int change, int dir){

  // Index of thread in either the x or y directions
  size_t long_index = threadIdx.x + blockIdx.x * blockDim.x;

  // Index of thread in the z direction
  size_t z_index = threadIdx.z + blockIdx.z * blockDim.z;
  
  // Do not go outside grid bounds
  if (z_index >= voxel_grid->height || long_index >= voxel_grid->width)
    return;

  // Inintialize our translation variables
  rolling_map::Coord voxel_index{long_index, long_index, z_index};
  rolling_map::Coord reference_index = voxel_index;

  // Determine the iteration start and end points
  int start = (change > 0) ?         0         : (voxel_grid->width - 1);
  int end   = (change > 0) ? voxel_grid->width :           -1           ;
  int diff  = SIGN(end - start);

  // Determine what axis we are iterating across
  int* changing_index;
  int* changing_ref_index;
  switch(dir){
    case X_CHANGE:
      changing_index     = &voxel_index.x;
      changing_ref_index = &reference_index.x;
      break;
    
    case Y_CHANGE:
      changing_index     = &voxel_index.y;
      changing_ref_index = &reference_index.y;
      break;

    default:
      assert(false); // CUDA version of throwing an error
  }

  // References for more readable code
  int& curr = *changing_index;
  int& ref  = *changing_ref_index;

  for (curr = start; curr != end; curr += diff){
    // Update the reference index
    ref = curr + change;

    // If the translated voxel is off the grid, clear the current voxel
    if (voxel_grid->offGrid(reference_index)){
      voxel_grid->freeVoxel(voxel_index);
    }

    // Otherwise match the current voxel with the translated voxel
    else if (voxel_grid->getVoxel(reference_index)){
      voxel_grid->markVoxel(voxel_index);
    }

    else{
      voxel_grid->freeVoxel(voxel_index);
    }
  }      

}

// KERNELS
// ================================================================================================
// ================================================================================================
// ================================================================================================
// HOST CODE

void rolling_map::RollingMap::updatePosition(float x, float y){

  // find indices of the new and old positions
  int xIndex, yIndex, zIndex, xOld, yOld;
  index(x,y,0.0,xIndex,yIndex,zIndex);
  index(getXPosition(),getYPosition(),0.0,xOld,yOld,zIndex);

  // calculate the shift from the old position to the new
  int xChange = xIndex-xOld;
  int yChange = yIndex-yOld;

  dim3 block_size;
  block_size.x = THREADS_PER_BLOCK/8;
  block_size.y = 1;
  block_size.z = 8;

  dim3 grid_size;
  grid_size.x = this->width / block_size.x + 1;
  grid_size.y = 1;
  grid_size.z = this->height / block_size.z + 1;

  if (xChange){
    translateMap<<<grid_size,block_size>>>(d_voxel_grid_, xChange, X_CHANGE);
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError(); /* translateMapX */);
  }

  if (yChange){
    translateMap<<<grid_size,block_size>>>(d_voxel_grid_, yChange, Y_CHANGE);
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError(); /* translateMapY */);
  }
  
  xPosition = x;
  yPosition = y;
  minXP += xChange*resolution;
  minYP += yChange*resolution;
  x0 = minXP;
  y0 = minYP;

  updateMetaData<<<1,1>>>(d_voxel_grid_, minXP, minYP);
  cudaDeviceSynchronize();
  CUDA_SAFE(cudaGetLastError(); /* updateMetaData */);
}