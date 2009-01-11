#include "Object.h"

#include <GL/gl.h>
#include <GL/glut.h>

/*!
\file Object.cpp
\brief contains methods for the object class
\author Michael Jones
\version 1
\date 06/02/06
*/

namespace Flock {

/* Constructor:
*  ------------
*	Sets default values for the object
*/
Object::Object(World& theContainer, float x, float y, float z)
 :	m_pos( x, y, z ),
	m_vel( 0.0f, 0.0f, 0.0f )
{
	m_container = &theContainer;
}

/* Contain:
*  --------
*	If the object is moving this loops it back into the world
*	if it leaves one side.
*/
void Object::Contain()
{
		if(m_pos.x > m_container->maxX)
			m_pos.x = m_container->minX;
		else if(m_pos.x < m_container->minX)
			m_pos.x = m_container->maxX;
		else if(m_pos.y > m_container->maxY)
			m_pos.y = m_container->minY;
		else if(m_pos.y < m_container->minY)
			m_pos.y = m_container->maxY;
		else if(m_pos.z > m_container->maxZ)
			m_pos.z = m_container->minZ;
		else if(m_pos.z < m_container->minZ)
			m_pos.z = m_container->maxY;
}

/* Update:
*  -------
*	A very simple position update that is not currently used.
*/
void Object::Update()
{
	m_pos += m_vel;

	Contain();
}


/* Draw:
*  -----
*	Draws a white sphere at the objects position and a shadow below it.
*/
void Object::Draw()
{
	// Draw sphere
	glPushMatrix();
		glTranslatef(m_pos.x, m_pos.y, m_pos.z);
		glColor4f(1.0, 1.0, 1.0, 1.0);
		glutSolidSphere(2, 9, 9);
	glPopMatrix();
	
	// Draw Shadow
	glPushMatrix();
		glTranslatef(m_pos.x, m_container->minY + 0.01, m_pos.z);
		glColor4f(0.1, 0.3, 0.1, 1.0);
		glBegin(GL_TRIANGLE_FAN);
			
			glVertex3f(0.0, 0.0, 0.0);
			
			for(float ang=0; ang<=6.3; ang=ang+0.1)
			{
				glVertex3f(2*sin(ang), 0.0, 2*cos(ang));
			}
		glEnd();
	glPopMatrix();
}

} // Flock
