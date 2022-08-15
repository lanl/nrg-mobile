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

#include <cuda_runtime.h>
#include "rolling_map.h"
#include "cuda_safe.cuh"
#include "cuda_voxel_grid.cuh"

// ================================================================================================
// ================================================================================================

namespace rolling_map{

__global__ void setIndexCuda(rolling_map::cudaVoxelGrid* voxel_grid, const rolling_map::Coord idx, const bool value){
    // Note: Boundary checking is done on the host
    if (value){
        voxel_grid->markVoxel(idx);
    }else{
        voxel_grid->freeVoxel(idx);
    }
}

bool RollingMap::setIndex(int x, int y, int z, bool value){
    std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);

    if (not checkIndex(x, y, z)) return false;

    setIndexCuda<<<1,1>>>(d_voxel_grid_, {x,y,z}, value);
    return true;
}

bool RollingMap::setPosition(float xp, float yp, float zp, bool value){
    std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);

    int x, y, z;
    if (not toIndex(xp, yp, zp, x, y, z)) return false;

    setIndexCuda<<<1,1>>>(d_voxel_grid_, {x,y,z}, value);
    return true;
}

// ================================================================================================
// ================================================================================================

__global__ void getIndexCuda(rolling_map::cudaVoxelGrid* voxel_grid, const rolling_map::Coord idx, bool* value){
    *value = voxel_grid->getVoxel(idx);
}

bool RollingMap::atIndex(int x, int y, int z, bool& value){
    std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

    if (not checkIndex(x, y, z)) return false;

    bool* val;
    CUDA_SAFE(cudaMalloc(&val, sizeof(bool)));
    getIndexCuda<<<1,1>>>(this->d_voxel_grid_, {x,y,z}, val);
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError(); /* getIndexCuda */);

    CUDA_SAFE(cudaMemcpy(&value, val, sizeof(bool), cudaMemcpyDeviceToHost));
    CUDA_SAFE(cudaFree(val));

    bool ret_val = cuda_ok;
    cuda_ok = true;
    return ret_val;
}

bool RollingMap::atPosition(float xp, float yp, float zp, bool& value){
    std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

    int x, y, z;
    index(xp, yp, zp, x, y, z);

    return atIndex(x, y, z, value);
}

// ================================================================================================
// ================================================================================================

void RollingMap::clearAll(){
    std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);
    CUDA_SAFE(cudaMemset(d_voxel_grid_, 0x00, voxel_grid_size_bytes_));
}

// ================================================================================================
// ================================================================================================

void RollingMap::insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin){
    std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_); 

    Coord sensor_voxel_idx;
    pcl::PointXYZ start_voxel_loc;

    // Get index of sensor posistion
    if(!toIndex(sensorOrigin.x, sensorOrigin.y, sensorOrigin.z, sensor_voxel_idx.x, sensor_voxel_idx.y, sensor_voxel_idx.z))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because it is not contained in the map bounds. Map bounds: (" 
        << getMinXP() << ", " << getMinYP() << ", " << getMinZP() << ") to (" << getMaxXP() << ", " << getMaxYP() << ", " << getMaxZP() 
        << "). sensor position: (" << sensorOrigin.x << ", " << sensorOrigin.y << ", " << sensorOrigin.z << ").");
    return; 
    }

    // Get position of start voxel
    if(!toPosition(sensor_voxel_idx.x, sensor_voxel_idx.y, sensor_voxel_idx.z, start_voxel_loc.x, start_voxel_loc.y, start_voxel_loc.z))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because sensor voxel is not contained in the map bounds. Map bounds: (0, 0, 0) to " 
        << getMaxXI() << ", " << getMaxYI() << ", " << getMaxZI() 
        << "). sensor position: (" << sensor_voxel_idx.x << ", " << sensor_voxel_idx.y << ", " << sensor_voxel_idx.z << ").");
    return; 
    }

    // Cast all rays on gpu for multithreading
    if(!castRays(scancloud, sensorOrigin, start_voxel_loc))
        ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Error in cuda castRays function.");
}

// ================================================================================================
// ================================================================================================

std::vector<pcl::PointXYZ> RollingMap::getMap(){
    std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_); 

    // Initialize output vector
    std::vector<pcl::PointXYZ> mapcloud;

    const size_t num_bits = width_*width_*height_;
    const size_t num_ints = num_bits/voxel_block_size + 1;
    std::vector<voxel_block_t> voxel_grid(num_ints);
    CUDA_SAFE(cudaMemcpy(voxel_grid.data(), d_voxel_data_, num_ints*sizeof(voxel_block_t), cudaMemcpyDeviceToHost));

    for (size_t bit_idx = 0; bit_idx < num_bits; bit_idx++){
        const size_t  byte_idx = bit_idx/voxel_block_size;
        const voxel_block_t bit_mask = static_cast<voxel_block_t>(1) << (bit_idx % voxel_block_size);

        if (voxel_grid[byte_idx] & bit_mask){
            mapcloud.emplace_back(
                (bit_idx % width_),
                (bit_idx / width_) % width_,
                (bit_idx / width_ / width_) % height_
            );
        }
    }

    return mapcloud;
}


} // end namespace rolling_map
