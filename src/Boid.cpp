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

namespace Flock {

/* Constructor:
*  ---------------------
*	Sets default values for the boids
*/
Boid::Boid( unsigned bID, int fID,  double x, double y, double z, double spread)
 :	m_id( bID ),
	m_flock_id( fID )
{
	Imath::Rand48 rand( bID );

	// Create a spread of positions around an average
	m_pos.x = x + rand.nextf(-1.0, 1.0) * spread/2;
	m_pos.y = y + rand.nextf(-1.0, 1.0) * spread/2;
	m_pos.z = z + rand.nextf(-1.0, 1.0) * spread/2;

	// Create a spread of velocities
	// Vel.setValue(RandomPosNum(5) - 2.5, RandomPosNum(5) - 2.5, RandomPosNum(5) - 2.5, 0);
	m_vel.setValue( rand.nextf( -2.5, 2.5 ), rand.nextf( -2.5, 2.5 ), rand.nextf( -2.5, 2.5 ) );
	
	// Pitch
	m_dir.x = -atan(m_vel.y/sqrt(m_vel.x*m_vel.x + m_vel.z*m_vel.z));

	// Yaw 
	m_dir.y = atan2(m_vel.x, m_vel.z);

	// Zero the inital acceleration
	m_acc.setValue(0.0, 0.0, 0.0 );
}

void Clamp( Imath::V3f &value, float maxValue) 
{
	if( value.length() > maxValue )	
	{
		value.normalize();
		value = value * maxValue;
	}
}

void Boid::update( const Flock::Behaviour& behaviour )
{
	float preClampAcc = m_acc.length();

	Clamp(m_acc, behaviour.maxAcc);

	// Increment velocity by acceleration
	m_vel = m_vel + m_acc*0.04;

	Clamp(m_vel, behaviour.maxVel);

	// lower bound clamp on velocity.
	if(m_vel.length() < behaviour.minVel)
	{
		m_vel.normalize();
		m_vel *= behaviour.minVel;
	}

	// increment position by velocity
	m_pos += m_vel*0.04;
	
	float vx = m_vel.x;
	float vy = m_vel.y;
	float vz = m_vel.z;
	
	// Pitch
	m_dir.x = -atan(vy/sqrt(vx*vx + vz*vz));

	// Yaw - Working
	m_dir.y = atan2(vx, vz);
	
	// Roll
	Imath::V3f up(0.0, 1.0, 0.0);
	
	// calculate local xAxis for boid
	Imath::V3f xAxis = m_vel.cross(up);  
	xAxis.normalize();
	
	float totalAcc = behaviour.localFC.max + behaviour.globalFC.max + behaviour.goalFC.max
		+ behaviour.collisionAvoidance.max + behaviour.objectAvoidance.max
		+ behaviour.velocityMatching.max + behaviour.hunt.max + behaviour.flee.max;
	
	float AccWeight = preClampAcc/totalAcc;

	Imath::V3f AccNorm = m_acc;
	float tilt = xAxis.dot(AccNorm);
	tilt = behaviour.bankingScale * tilt * AccWeight * behaviour.maxAcc;
	
	// Save roll in roll vector
	m_old_roll.insert(m_old_roll.begin(), tilt);
	
	// remove last roll if necessary
	if( m_old_roll.size() > behaviour.bankingDepth )
		m_old_roll.pop_back();
	

	std::vector<float>::iterator currentRoll = m_old_roll.begin();
	std::vector<float>::iterator endRoll = m_old_roll.end();

	int count = 0;
	float roll = 0;
	
	// Average over the last 'n' rolls
	for( ; currentRoll != endRoll; ++currentRoll )
	{
		roll = roll + (*currentRoll);
		++count;
	}

	roll = roll / count;
	
	m_dir.z = -atan2(roll, -9.8);

	m_acc.setValue( 0.0f, 0.0f, 0.0f );
}

/* Draw:
*  ---------------------
*	Runs the OpenGL commands required to display the boid.
*/
void Boid::Draw(float floorHeight) const
{
	float pitch, yaw, roll;
	
	// Draw boid
	
	glPushMatrix();
	
		// translate to the particle position
		// Pos.Translate();
		glTranslatef( m_pos.x, m_pos.y, m_pos.z );
		
		// Convert from radians to degrees
		pitch = m_dir.x * 57.295779524;
		yaw = m_dir.y * 57.295779524;
		roll = m_dir.z * 57.295779524;
	
		// Rotate the boid appropriately 
		glRotatef(pitch, 1.0, 0.0, 0.0);
		glRotatef(yaw, 0.0, cos(m_dir.x), - sin(m_dir.x));
		glRotatef(roll, 0.0, 0.0, 1.0);
		
		glScalef(1.0, 0.3, 0.3);
		glutSolidCone(0.5, 1, 4, 1);
		
	glPopMatrix();
	
	// Shadow Colour
	glColor3f(0.1, 0.3, 0.1);
	
	// Draw Shadow
	
	glPushMatrix();
	
		// translate to the particle position
		glTranslatef(m_pos.x, floorHeight+0.1, m_pos.z);
		
		// Rotate shadow
		glRotatef(yaw, 0.0, 1.0, 0.0);
		
		pitch = fabs(0.5 + (m_dir.x * 57.295779524/90.0)/2);
		yaw = m_dir.y * 57.295779524/180.0;
		
		// Scale it a little with the pitch
		glScalef(1, 1, pitch);
		
		// Draw a basic triangle
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(0.0, 0.0, 1.0);
			glVertex3f(0.5, 0.0, -0.3);
			glVertex3f(-0.5, 0.0, -0.3);
		glEnd();
		
	glPopMatrix();
}

} // Flock

