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
* Author: Alex Navarro
*********************************************************************/

#include <pcl/point_types.h>
#include "cuda_safe.cuh"
#include "rolling_map.h"
#include "cuda_voxel_grid.cuh"

namespace rolling_map{

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ cudaVoxelGrid::cudaVoxelGrid(int width, int height, float res) :
width(width), 
height(height),
resolution(res)
{}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ cudaVoxelGrid::~cudaVoxelGrid() {

}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// 
__device__ rolling_map::Coord cudaVoxelGrid::toIndex(const pcl::PointXYZ& p){
  rolling_map::Coord idx;
  const float one_over_res = 1/resolution;
  idx.x = (p.x - min_x)*one_over_res;
  idx.y = (p.y - min_y)*one_over_res;
  idx.z = (p.z - min_z)*one_over_res;
  return idx;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ void cudaVoxelGrid::markVoxel(const Coord& c){
  size_t         flat_idx  = width*width*c.z + width*c.y + c.x;
  voxel_block_t* mem_block = voxels + flat_idx/voxel_block_size;
  voxel_block_t  bit_mask  = static_cast<voxel_block_t>(1) << (flat_idx % voxel_block_size);
  atomicOr(mem_block, bit_mask);
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ void cudaVoxelGrid::freeVoxel(const Coord& c){
  size_t         flat_idx  = width*width*c.z + width*c.y + c.x;
  voxel_block_t* mem_block = voxels + flat_idx/voxel_block_size;
  voxel_block_t  bit_mask  = static_cast<voxel_block_t>(1) << (flat_idx % voxel_block_size);
  atomicAnd(mem_block, ~bit_mask);
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ bool cudaVoxelGrid::getVoxel(const Coord& c) const {
  const size_t          flat_idx = width*width*c.z + width*c.y + c.x;
  const voxel_block_t* mem_block = voxels + flat_idx/voxel_block_size;
  const voxel_block_t   bit_mask = static_cast<voxel_block_t>(1) << (flat_idx % voxel_block_size);
  return (*mem_block & bit_mask);
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__device__ bool cudaVoxelGrid::offGrid(const Coord& coord) const {
  return (coord.x < 0 || coord.x >= width ||
          coord.y < 0 || coord.y >= width ||
          coord.z < 0 || coord.z >= height );
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

__global__ void initVoxelGrid(rolling_map::cudaVoxelGrid** map_ptr, int width, int height, float resolution, float minx, float miny, float minz, uint32_t* voxel_device_ptr){
	*map_ptr = new cudaVoxelGrid(width, height, resolution);
	(*map_ptr)->voxels = voxel_device_ptr;
	(*map_ptr)->min_x  = minx;
	(*map_ptr)->min_y  = miny;
	(*map_ptr)->min_z  = minz;
}


// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

bool rolling_map::RollingMap::cudaInit(){

    // Initialize the voxel grid
    cudaVoxelGrid** new_grid;
    CUDA_SAFE(cudaMalloc(&new_grid, sizeof(cudaVoxelGrid**)));
    // Make sure that voxel grid data space is divisible by 32 for CUDA atomic operations
    CUDA_SAFE(cudaMalloc(&d_voxel_data_, (width*width*height/voxel_block_size + 1)*sizeof(voxel_block_t)));	
    initVoxelGrid<<<1,1>>>(new_grid, width, height, resolution, minXP, minYP, z0, d_voxel_data_);
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError(); /* initVoxelGrid */);

    // Copy the grid pointer to the host
    CUDA_SAFE(cudaMemcpy(&d_voxel_grid_, new_grid, sizeof(cudaVoxelGrid*), cudaMemcpyDeviceToHost));

    bool output = cuda_ok;
    cuda_ok = true;
    return output;
}

} // end namespace rolling_map
