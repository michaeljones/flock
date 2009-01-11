#include "Goal.h"

#include <iostream>

#include <GL/gl.h>
#include <GL/glut.h>

/*!
\file Goal.cpp
\brief contains methods for the goal class
\author Michael Jones
\version 1
\date 06/02/06
*/

namespace Flock {

/* Constructor:
*  ------------
*	Sets default values for the goal
*/
Goal::Goal(World& theContainer)
{
	Pos.x = 10;
	Pos.y = 0;
	Pos.z = 0;

	Vel.x = 0.16;
	Vel.y = 0;
	Vel.z = 0;

	curveDistance = 0.002;
	direction = 1;
	
	container = &theContainer;
}


/* setCurve:
*  ---------
*	Adds the curve created to the goal curve follow object
*/
// void Goal::setCurve()
// {
// 	// curve = &theCurve;
// }


/* Update:
*  -------
*	Updates the goal's position along the curve
*	flipping its direction when it gets to an end point
*/
void Goal::Update()
{
	// if(direction > 0)
	// {
	// 	Pos = curve->GetPointOnCurve(curveDistance);
	// 	curveDistance += 0.002;
	// 	if(curveDistance >= 1.0)
	// 		direction = -1;
	// } 
	// else 
	// {
	// 	Pos = curve->GetPointOnCurve(curveDistance);
	// 	curveDistance -= 0.002;
	// 	if(curveDistance <= 0.0)
	// 		direction = 1;
	// }
}


/* Draw:
*  -------
*	Draws the goal as chunky white sphere.
*/
void Goal::Draw()
{
	glPushMatrix();
		// translate to the particle position
		// Pos.Translate();
		glColor4f(1.0, 1.0, 1.0, 1.0);
		glutSolidSphere(0.5, 3, 3);
	glPopMatrix();
}

} // Flock
