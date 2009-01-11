#include "Particle.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

/*!
\file Particle.cpp
\brief contains methods for the particle class
\author Michael Jones
\version 1
\date 06/02/06
*/

namespace Flock {

// Empty Constructor
Particle::Particle()
{

}

// Empty Deconstructor
Particle::~Particle()
{

}

/* Constructor:
*  ------------
*	Sets default values for the object
*/
Particle::Particle(Imath::V3f boidPos, Imath::Color4<float> boidColour, float fHeight)
{
	Pos = boidPos;
	colour = boidColour;
	
	floorHeight = fHeight;
	
	// Dir.set(RandomNum(0.3), RandomPosNum(0.5), RandomNum(0.3)); 
	Dir.setValue( 0.0, 1.0, 0.0 );

	gravity.setValue(0.0, -0.8, 0.0);
}

/* Update:
*  -------
*	Simple update of the particles position taking into account
*	the gravitational down direction. Actual value is not 9.8
*/
void Particle::Update()
{
	Dir = Dir + gravity/25.0;

	Pos = Pos + Dir;
}

/* Draw:
*  -----
*	Draws a rough sphere at each particle position 
*	and a shadow beneath it.
*/
void Particle::Draw()
{
	// Draw particle
	glPushMatrix();
	
		glTranslatef(Pos.x, Pos.y, Pos.z);
		// colour.Use();
		
		glutSolidSphere(0.1, 3, 3);
		
	glPopMatrix();
	
	// Set shadow colour
	glColor3f(0.1, 0.3, 0.1);
	
	// Draw shadow
	glPushMatrix();

		glTranslatef(Pos.x, floorHeight + 0.1, Pos.z);
		
		glBegin(GL_QUADS);
			
			glVertex3f(0.1, 0.0, 0.1);
			glVertex3f(0.1, 0.0, -0.1);
			glVertex3f(-0.1, 0.0, 0.1);
			glVertex3f(-0.1, 0.0, -0.1);

		glEnd();

		
	glPopMatrix();
}

} // Flock
