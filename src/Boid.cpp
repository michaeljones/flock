#include "Boid.h"
#include "Flock.h"

#include <iostream>
#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <ImathRandom.h>

/*!
\file Boid.cpp
\brief contains methods for the boid class
\author Michael Jones
\version 1
\date 06/02/06
*/

/* Constructor:
*  ---------------------
*	Sets default values for the boids
*/
Boid::Boid(int bID, int fID,  double x, double y, double z, double spread)
{
	boidID = bID;
	flockID = fID;

	Imath::Rand48 rand( bID );

	// Create a spread of positions around an average
	Pos.x = x + rand.nextf(-1.0, 1.0) * spread/2;
	Pos.y = y + rand.nextf(-1.0, 1.0) * spread/2;
	Pos.z = z + rand.nextf(-1.0, 1.0) * spread/2;

	// Create a spread of velocities
	// Vel.setValue(RandomPosNum(5) - 2.5, RandomPosNum(5) - 2.5, RandomPosNum(5) - 2.5, 0);
	Vel.setValue( 1, 1, 1 );
	
	// Pitch
	Dir.x = -atan(Vel.y/sqrt(Vel.x*Vel.x + Vel.z*Vel.z));

	// Yaw 
	Dir.y = atan2(Vel.x, Vel.z);
	
	// Zero the inital acceleration
	Acc.setValue(0.0, 0.0, 0.0 );
}

/* Draw:
*  ---------------------
*	Runs the OpenGL commands required to display the boid.
*/
void Boid::Draw(float floorHeight) const
{
	float Pitch, Yaw, Roll;
	
	// Draw boid
	
	glPushMatrix();
	
		// translate to the particle position
		// Pos.Translate();
		glTranslatef( Pos.x, Pos.y, Pos.z );
		
		// Convert from radians to degrees
		Pitch = Dir.x * 57.295779524;
		Yaw = Dir.y * 57.295779524;
		Roll = Dir.z * 57.295779524;
	
		// Rotate the boid appropriately 
		glRotatef(Pitch, 1.0, 0.0, 0.0);
		glRotatef(Yaw, 0.0, cos(Dir.x), -sin(Dir.x));
		glRotatef(Roll, 0.0, 0.0, 1.0);
		
		glScalef(1.0, 0.3, 0.3);
		glutSolidCone(0.5, 1, 4, 1);
		
	glPopMatrix();
	
	// Shadow Colour
	glColor3f(0.1, 0.3, 0.1);
	
	// Draw Shadow
	
	glPushMatrix();
	
		// translate to the particle position
		glTranslatef(Pos.x, floorHeight+0.1, Pos.z);
		
		// Rotate shadow
		glRotatef(Yaw, 0.0, 1.0, 0.0);
		
		Pitch = fabs(0.5 + (Dir.x * 57.295779524/90.0)/2);
		Yaw = Dir.y * 57.295779524/180.0;
		
		// Scale it a little with the pitch
		glScalef(1, 1, Pitch);
		
		// Draw a basic triangle
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(0.0, 0.0, 1.0);
			glVertex3f(0.5, 0.0, -0.3);
			glVertex3f(-0.5, 0.0, -0.3);
		glEnd();
		
	glPopMatrix();
}
