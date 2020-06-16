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
* Author: Meredith Symmank
*
* Description: Implementation of a 3D point struct.
*********************************************************************/
#ifndef POINT_H
#define POINT_H


struct Point3D{
	public:
		/* The x, y, and z variables defining the point
		 * Not going to bother with private members and accessor functions
		 */
		float x, y, z;
    
		/* The constructor will accept float and int data types or none at all.
		 * Two parameters sets to x,y respectively; one parameter sets both
		 * x and y to the same value.  An empty constructor sets x,y = 0.0
		 *
		 * There is no destructor since the point is essential just two floats
		 */
		Point3D() : x(0.0), y(0.0), z(0.0) {};
		Point3D(float x, float y, float z) : x(x), y(y), z(z) {};
		Point3D(float val) : x(val), y(val), z(val) {};
		Point3D(int x, int y, int z) : x(static_cast<float>(x)), 
		                               y(static_cast<float>(y)), 
		                               z(static_cast<float>(z)) {};
		Point3D(int val) : x(static_cast<float>(val)), 
		                   y(static_cast<float>(val)),
		                   z(static_cast<float>(val)) {};

		//Operators:

		//Addition
		Point3D operator+=(Point3D pnt){ (*this).x += pnt.x; (*this).y += pnt.y; (*this).z += pnt.z; return (*this); }
		Point3D operator+=(float num){ (*this).x += num; (*this).y += num; (*this).z += num; return (*this); }
		Point3D operator+(Point3D pnt) { return Point3D((*this).x + pnt.x, (*this).y + pnt.y, (*this).z + pnt.z);	}
		Point3D operator+(float num) { return Point3D((*this).x + num, (*this).y + num, (*this).z + num); }

		//Subtraction
		Point3D operator-=(Point3D pnt){ (*this).x -= pnt.x; (*this).y -= pnt.y; (*this).z -= pnt.z; return (*this); }
		Point3D operator-=(float num){ (*this).x -= num; (*this).y -= num; (*this).z -= num; return (*this); }
		Point3D operator-(Point3D pnt) { return Point3D((*this).x - pnt.x, (*this).y - pnt.y, (*this).z - pnt.z); }
		Point3D operator-(float num) { return Point3D((*this).x - num, (*this).y - num, (*this).z - num); }

		//Multiplication
		Point3D operator*=(Point3D pnt){ (*this).x *= pnt.x; (*this).y *= pnt.y; (*this).z *= pnt.z; return (*this); }
		Point3D operator*=(float num){ (*this).x *= num; (*this).y *= num; (*this).z *= num; return (*this); }
		Point3D operator*(Point3D pnt) { return Point3D((*this).x * pnt.x, (*this).y * pnt.y, (*this).z * pnt.z); }
		Point3D operator*(float num) { return Point3D((*this).x * num, (*this).y * num, (*this).z * num); }

		//Division
		Point3D operator/=(Point3D pnt){ (*this).x /= pnt.x; (*this).y *= pnt.y; (*this).z *= pnt.z; return (*this); }
		Point3D operator/=(float num){ (*this).x /= num; (*this).y *= num; (*this).z *= num; return (*this); }
		Point3D operator/(Point3D pnt) { return Point3D((*this).x / pnt.x, (*this).y * pnt.y, (*this).z * pnt.z); }
		Point3D operator/(float num) { return Point3D((*this).x / num, (*this).y * num, (*this).z * num); }

		//Equal (Assignment)
		Point3D operator=(Point3D pnt) { (*this).x = pnt.x; (*this).y = pnt.y; (*this).z = pnt.z; return (*this); }
		Point3D operator=(float num) { (*this).x = num; (*this).y = num; (*this).z = num; return (*this); }


};

//Comparative operators:
bool operator==(Point3D a, Point3D b) { return a.x==b.x && a.y==b.y && a.z==b.z; }
bool operator==(Point3D a, float num) { return a.x==num && a.y==num && a.z==num; }
bool operator!=(Point3D a, Point3D b) { return !(a==b); }
bool operator!=(Point3D a, float num) { return !(a.x==num && a.y==num && a.z==num); }

bool operator<(Point3D a, Point3D b) {
    if (a.x < b.x) return true;
    if (a.x > b.x) return false;
    if (a.y < b.y) return true;
    if (a.y > b.y) return false;
    if (a.z < b.z) return true;
    return false;
}
bool operator<(Point3D pnt, float num) { return pnt.x < num && pnt.y < num && pnt.z < num;}
bool operator>(Point3D a, Point3D b) {
    if (a.x > b.x) return true;
    if (a.x < b.x) return false;
    if (a.y > b.y) return true;
    if (a.y < b.y) return false;
    if (a.z > b.z) return true;
    return false;
}
bool operator>(Point3D pnt, float num) { return pnt.x > num && pnt.y > num && pnt.z > num;}

//Other operators:
Point3D operator+(float num, Point3D pnt) { return (pnt + num); }
Point3D operator*(float num, Point3D pnt) { return (pnt * num); }


//Allows the display of the points in a 'coordinate' format (x,y) to the output stream
std::ostream& operator<<(std::ostream& os, const Point3D a) {
	os << "(" << a.x << "," << a.y << "," << a.z << ")";
	return os;
}


#endif 