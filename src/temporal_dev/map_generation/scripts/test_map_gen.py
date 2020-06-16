# /*********************************************************************
# *
# *  © (or copyright) 2020. Triad National Security, LLC.
# *  All rights reserved.
# *  This program was produced under U.S. Government contract 
# *  89233218CNA000001 for Los AlamosNational Laboratory (LANL), 
# *  which is operated by Triad National Security, LLC for the U.S.
# *  Department of Energy/National Nuclear Security Administration. 
# *  All rights in the program are reserved by Triad National 
# *  Security, LLC, and the U.S. Department of Energy/National Nuclear
# *  Security Administration. The Government is granted for itself 
# *  and others acting on its behalf a nonexclusive, paid-up, 
# *  irrevocable worldwide license in this material to reproduce, 
# *  prepare derivative works, distribute copies to the public, 
# *  perform publicly and display publicly, and to permit others 
# *  to do so.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions
# *  are met:
# *
# *   * Redistributions of source code must retain the above copyright
# *     notice, this list of conditions and the following disclaimer.
# *   * Redistributions in binary form must reproduce the above
# *     copyright notice, this list of conditions and the following
# *     disclaimer in the documentation and/or other materials provided
# *     with the distribution.
# *   * Neither the name of the copyright holder nor the names of its
# *     contributors may be used to endorse or promote products derived
# *     from this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# *  POSSIBILITY OF SUCH DAMAGE.
# *
# * Author: Meredith Symmank
# *
# * Description: ROS service for calling the tmap generator for testing 
# *   purposes
# *********************************************************************/
#!/usr/bin/env python
import os 
import rospy
from std_srvs.srv import Empty
from map_generation.srv import InitMapGen, InitMapGenRequest
from map_generation.msg import TemporalObject

# SOLE purpose of this node is to run the map generator a bunch of times in a row 
# so I can get a lot of simulated data. -- Meredith 

def service_client():
    rospy.loginfo('waiting service')
    rospy.wait_for_service('/tmap_generator/initialize_map')
    rospy.wait_for_service('/tmap_generator/get_next_map')
    # rospy.wait_for_service('map_converter/ros2arnl')
    init_service = rospy.ServiceProxy('/tmap_generator/initialize_map', InitMapGen)
    request = InitMapGenRequest()
    request.map_file = ''
    request.type_array = [6,12,9,10]
    try:
        response = init_service(request)
        print("Initialization complete")
    except rospy.ServiceException as e:
        print("Service call to initialize tmap_generator failed: %s" % e)

    for i in range(0,999):
        try:
            service = rospy.ServiceProxy('/tmap_generator/get_next_map', Empty)
            service()
            print("Finished map step %d" % (i + 1))
        except rospy.ServiceException as e:
            print("Service call to tmap_generator failed: %s" % e)
    counter = 0
    while not rospy.is_shutdown():
        counter = counter+1
        if counter % 100000 == 0:
            print("Waiting for ROS to shutdown")




def testMapGeneration():
    rospy.init_node('test_tmap_generator')
    while not rospy.is_shutdown():
       service_client()
    print('Test complete')

    
    
if __name__ == '__main__':
    testMapGeneration()
