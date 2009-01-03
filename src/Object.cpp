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

/* Constructor:
*  ------------
*	Sets default values for the object
*/
Object::Object(World& theContainer, float x, float y, float z)
{
	Pos.x = x;
	Pos.y = y;
	Pos.z = z;

	Vel.x = 0.0;
	Vel.y = 0.0;
	Vel.z = 0.0;

	container = &theContainer;
}

/* Contain:
*  --------
*	If the object is moving this loops it back into the world
*	if it leaves one side.
*/
void Object::Contain()
{
		if(Pos.x > container->maxX)
			Pos.x = container->minX;
		else if(Pos.x < container->minX)
			Pos.x = container->maxX;
		else if(Pos.y > container->maxY)
			Pos.y = container->minY;
		else if(Pos.y < container->minY)
			Pos.y = container->maxY;
		else if(Pos.z > container->maxZ)
			Pos.z = container->minZ;
		else if(Pos.z < container->minZ)
			Pos.z = container->maxY;
}

/* Update:
*  -------
*	A very simple position update that is not currently used.
*/
void Object::Update()
{
	Pos += Vel;

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
		glTranslatef(Pos.x, Pos.y, Pos.z);
		glColor4f(1.0, 1.0, 1.0, 1.0);
		glutSolidSphere(2, 9, 9);
	glPopMatrix();
	
	// Draw Shadow
	glPushMatrix();
		glTranslatef(Pos.x, container->minY + 0.01, Pos.z);
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
