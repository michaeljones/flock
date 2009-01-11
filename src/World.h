#ifndef __WORLD_H__
#define __WORLD_H__

#include "Flock.h"
#include "Object.h"

/*!
\file World.h
\brief contains world (bounding box) information for a flock 
\author Michael Jones
\version 1
\date 17/03/06
*/

namespace Flock {

class Flock;
class Object;

class World 
{
public:

/*! Min and max x values for the extent of the world */
float maxX, minX;

/*! Min and max y values for the extent of the world */
float maxY, minY;

/*! Min and max z values for the extent of the world */
float maxZ, minZ;

/*! STL vector with pointers to all the flocks in the current configuration */
std::vector<Flock*> flocks;

/*! STL vector with pointers to all the objects in the current configuration */
std::vector<Object*> objects;

/*! Default empty constructor for the class */
World();

/*! \brief this method draws a green ground plane at the base of the box.*/
void DrawGround();

/*! \brief method used to add an instance of a flock to the world 
	\param addFlock - a pointer to the flock that is to be added to the world */
void AddFlock(Flock* addFlock);

/*! \brief method used to add an instance of an object to the world 
	\param addObject - a pointer to the object that is to be added to the world */
void AddObject(Object* addObject);
};

}; // Flock

#endif
