#ifndef __GOAL_H__
#define __GOAL_H__

#include "World.h"

/*!
\file Goal.h
\brief contains target for flock placed on a curve
\author Michael Jones
\version 1
\date 17/03/06
*/

namespace Flock {

class Goal
{
public:

	/*! 3-dimensional position of the goal in space */
	Imath::V3f Pos;
	
	/*! static 3-dimensional vector specifying the goal's velocity */
	Imath::V3f Vel;

/*! Curve object that the goal follows through space */
// Imath::V3follow *curve;

/*! Distance along the curve at the current time */
float curveDistance;

/*! Direction that the goal is moving along the curve */
int direction;

/*! World pointer to the world containing the goal */
World *container;

/*! \brief this constructor method creates a goal with a pointer to the world 
	\param theContainer - the reference of the world object */
Goal(World& theContainer);

/*! \fn setCurve(GraphicsLib::CurveFollow& theCurve)
	this method sets the curve for goal object to follow
	\param theCurve - the CurveFollow object to set the curve to*/
// void setCurve(GraphicsLib::CurveFollow& theCurve);

/*! \fn void Update()
	this method updates the position of the goal along the curve, reversing its direction when it reaches either end*/
void Update();

/*! \fn void Draw
	this method draws a white sphere at the current goal position*/
void Draw();

};

}; // Flock

#endif
