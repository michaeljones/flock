#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include <ImathColor.h>


namespace Flock {

/*!
\file Particle.h
\brief contains particle information 
\author Michael Jones
\version 1
\date 17/03/06
*/

class Particle
{
public:

	/*! 3-dimensional position of the particle in space */
	Imath::V3f Pos;
	
	/*! static 3-dimensional vector specifying the particle's velocity */
	Imath::V3f Dir;
	
	/*! Colour object for colour of the particle */
	Imath::Color4<float> colour;
	
	/*! 3d vector specifying the gravitational down direction */
	Imath::V3f gravity;
	
	/*! Floating point value of the y height of the ground in the world */
	float floorHeight;
	
	/*! Default empty constructor for the class */
	Particle();
	
	/*! Default empty deconstructor for the class */
	~Particle();
	
	/*! \brief this constructor method creates a particle at a certain point with specified colour
		\param boidPos - position to create the particle at
		\param boidColor - colour of the particle
		\param fHeight - floor height of the world
		*/
	Particle( Imath::V3f boidPos, Imath::Color4<float> boidColor, float fHeight);
	
	/*! \brief this method updates the particle's motion */
	void Update();
	
	/*! \brief this method draws the particle at location Pos */
	void Draw();
};

}; // Flock

#endif //end particle_h

