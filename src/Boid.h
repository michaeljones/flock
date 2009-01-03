#ifndef __BOID_H__
#define __BOID_H__

#include <ImathVec.h>

#include <vector>

/*!
\file Boid.h
\brief boid class for a flocking system. 
\author Michael Jones
\version 1
\date 17/03/06
*/

class Boid
{
public:

	/*! Integer ID for the boid within the flock */
	int boidID;
	
	/*! Integer ID for the flock the boid is a part of */
	int flockID;
	
	/*! 3-dimensional position of the boid in space */
	Imath::V3f Pos;
	
	/*! 3-dimensional vector specifying the boid's velocity at the latest time step */
	Imath::V3f Vel;
	
	/*! 3-dimensional vector specifying the boid's acceleration at the latest time step */
	Imath::V3f Acc;
	
	/*! 3-dimensional vector in which each component is the boid's rotation
	 * around a particular access. For example, Dir.x is the pitch of the boid
	 * in radians, ie. the rotation around the x axis */
	Imath::V3f Dir;
	
	/*! An STL vector of the old rotation values associated with the roll of the boid */
	std::vector<float> oldRoll;
	
	
	/*! \brief this constructor method creates a boid with ID bID, setting its flock ID to fID. 
		\param bID - the ID of the boid being created
		\param flockID - the flock ID for the boid being created
		\param x - the x co-ordinate of the position around which the flock is created
		\param y - the y co-ordinate of the position around which the flock is created
		\param z - the z co-ordinate of the position around which the flock is created
		\param spread - the spread of the flock around the position at which it is created */
	Boid(int bID, int flockID, double x, double y, double z, double spread);
	
	/*! \brief this method draws the boid at location Pos and with orientation
		defined by the current velocity
		\param floorHeight - the floor height of the world which is used for
		drawing simple shadows on the ground */
	void Draw(float floorHeight) const;
};

#endif
