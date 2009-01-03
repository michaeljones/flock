#include "World.h"
#include "Flock.h"
#include "Object.h"

#include <GL/gl.h>
#include <GL/glu.h>

/*!
\file World.cpp
\brief contains methods for the world class
\author Michael Jones
\version 1
\date 06/02/06
*/

/* Constructor:
*  ------------
*	Empty
*/
World::World()
{

}


/* AddFlock:
*  ---------
*	Adds a flock to the classes STL vector of flocks.
*/
void World::AddFlock(Flock* addFlock)
{
	flocks.push_back(addFlock);
}


/* AddObject:
*  ---------
*	Adds an object to the classes STL vector of flocks.
*/
void World::AddObject(Object* addObject)
{
	objects.push_back(addObject);
}


/* DrawGround:
*  ---------
*	Draws a green plane at ground level in the world.
*/
void World::DrawGround()
{
	glPushMatrix();
		
		glTranslatef(0.0, minY, 0.0);
			
		glColor4f(0.0, 1.0, 0.0, 1.0);
	
		glBegin(GL_POLYGON);
			glVertex3f(minX, 0.0, minZ);
			glVertex3f(minX, 0.0, maxZ);
			glVertex3f(maxX, 0.0, maxZ);
			glVertex3f(maxX, 0.0, minZ);
		glEnd();

	glPopMatrix();
}
