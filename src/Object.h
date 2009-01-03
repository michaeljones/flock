#ifndef __OBJECT_H__
#define __OBJECT_H__

#include "World.h"

/*!
\file Object.h
\brief contains obstacle information for a flock 
\author Michael Jones
\version 1
\date 17/03/06
*/

class World;

class Object
{
public:
	
	/*! 3-dimensional position of the object in space */
	Imath::V3f Pos;
	
	/*! static 3-dimensional vector specifying the object's velocity */
	Imath::V3f Vel;
	
	/*! World pointer to the world containing the object */
	World *container;
	
	/*! \brief this constructor method creates a object at position (x, y, z) with a pointer to the world 
		\param theContainer - the reference of the world object 
		\param x - the x co-ordinate of the object within the world
		\param y - the y co-ordinate of the object within the world
		\param z - the z co-ordinate of the object within the world */
	Object(World& theContainer, float x, float y, float z);
	
	/*! \fn void Contain()
		this method restarts the object at the opposite side of the container if it moves out of bounds*/
	void Contain();
	
	/*! \fn void Update()
		this method updates the objects position
		\todo Add greater flexibility, currently only supports fixed x direction movement */
	void Update();
	
	/*! \fn void Draw()
		this method draws a white sphere at the current object position */
	void Draw();

};

#endif
