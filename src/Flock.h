#ifndef __FLOCK_H__
#define __FLOCK_H__

#include <vector>

#include "Boid.h"
#include "World.h"
#include "Object.h"
#include "Particle.h"

#include <ImathVec.h>
#include <ImathColor.h>

/*!
\file Flock.h
\brief contains controlling function for boids in a flock 
\author Michael Jones
\version 1
\date 17/03/06
*/

class World;

class Flock 
{
public:
	/*! Integer ID for the flock */
	int flockID;
	
	/*! Colour object for colour of the boids in the flock */
	Imath::Color4< float > colour;
	
	/*! Integer value for the number of boids in the flock */
	int numMembers;
	
	/*! Integer value relating to the position of the flock within the local foodchain */
	int rank;
	
	/*! Integer value for the number of time steps that the boids' banking is smoothed over */
	unsigned int bDepth;
	
	/*! Floating point scale factor for the boid's bncking */
	float bScale;
	
	/*! Scale factor and max value for local flock centring acceleration */
	float scaleLocalFC, maxLocalFC;
	
	/*! Scale factor and max value for global flock centring acceleration */
	float scaleGlobalFC, maxGlobalFC;
	
	/*! Scale factor and max value for goal flock centring acceleration */
	float scaleGoalFC, maxGoalFC;
	
	/*! Scale factor and max value for collision avoidance acceleration */
	float scaleCA, maxCA;
	
	/*! Scale factor and max value for velocity matching acceleration */
	float scaleVM, maxVM;
	
	/*! Scale factor and max value for object aviodance acceleration */
	float scaleOA, maxOA;
	
	/*! Scale factor and max value for hunting acceleration */
	float scaleHunt, maxHunt;
	
	/*! Scale factor and max value for fleeing aviodance acceleration */
	float scaleFlee, maxFlee;
	
	/*! Maximum boid velocity in the flock */
	float maxVel;
	
	/*! Minimum boid velocity in the flock */
	float minVel;
	
	/*! Maximum total boid acceleration in the flock */
	float maxAcc;
	
	/*! 3d point with co-ordinates of the flock centre. Recalculated at every time step */
	Imath::V3f flockCentre;
	
	/*! Default Null vector for reseting other vector easily */
	Imath::V3f Null;
	
	/*! 3d vector specifying the gravitational down direction */
	Imath::V3f gravity;
	
	/*! Test radius for boid-object interactions */
	float objectTR;
	
	/*! Test radius for boid-boid interactions */
	float boidTR;
	
	/*! Test radius for hunter-prey interactions */
	float fleeTR;
	
	/*! Acceleration to apply when the boids stray out of the bounds of the world */
	float containmentAcc;
	
	/*! STL vector with pointers to all the boids in the flock */
	std::vector<Boid*> boids;
	
	/*! STL vector with pointers to all the particles created by the boids killing other boids */
	std::vector<Particle*> particles;
	
	/*! World pointer to the world containing the flock */
	World *container;
	
	/*! Default empty constructor for the class */
	Flock();
	
	/*! \brief this constructor method creates a flock with ID fID and a pointer to the world 
		\param fID - the flock ID for the flock being created
		\param theContainer - the reference of the world object */
	Flock(int fID, World& theContainer);
	
	/*! \brief method used to clamp a vectors length to maxValue 
		\param value - the vector that is being clamped
		\param maxValue - the length that the vector is clamped to */
	void Clamp(Imath::V3f &value, float maxValue);
	
	/*! \brief method used to add an instance of a boid to a flock 
		\param addBoid - a pointer to the boid that is to be added to the flock */
	void AddBoid(Boid* addBoid);
	
	
	/*! \brief method to find the nearest 'n' neighbours to a boid in the flock 
		\param neighbourBoids - an STL vector passed to the function which is then filled out with pointers to the neighbouring boids 
		\param homeBoid - a pointer to the boid under consideration
		\param numNeighbours - the number of neighbouring boids to find */
	void NearestNeighbours(std::vector<Boid*>& neighbourBoids, Boid* homeBoid, int numNeighbours);
	
	/*! \brief method to find the centre of the flock and assign it to the flockCentre variable */
	void GetFlockCentre();
	
	/*! \brief method to implement the local flock centring behaviour */
	void LocalFlockCentring();
	
	/*! \brief method to implement the global flock centring behaviour */
	void GlobalFlockCentring();
	
	/*! \brief method to implement the goal flock centring behaviour 
		\param &target - the reference of the goal that the flock is centring on */
	void GoalFlockCentring(Imath::V3f &target);
	
	/*! \brief method to implement collision avoidance between boids in the flock */
	void CollisionAvoidance();
	
	/*! \brief method to implement velocity matching between boids in the flock */
	void VelMatching();
	
	
	/*! \brief method to implement object avoidance based in a radial force field */
	void CentralObjectAvoidance();
	
	/*! \brief method to implement object avoidance around cylindricl objects */
	void CylindricalObjectAvoidance();
	
	/*! \brief method to implement object avoidance around spherical objects */
	void SphericalObjectAvoidance();
	
	/*! \brief method to create hunting behaviour between flocks */
	void Hunt();
	
	/*! \brief method to create fleeing behaviour away from predator flocks */
	void Flee();
	
	/*! \brief method to check for boid collisions between flocks resulting in killing of prey */
	void Kill();
	
	/*! \brief method that checks the boids positions against the size of the world and loops them back in the other side if they leave the confines */
	void Contain();
	
	/*! \brief method to update particle motion */
	void ParticleUpdate();
	
	/*! \brief method to run all the behaviours and update the boid's motions based on the resultant accelerations */
	void Update(Imath::V3f &target);
	
	/*! \brief method clear the STL vector of boids */
	void Clear();
	
	/*! \brief method cycles through all the boids and particles and calls the appropriate draw method */
	void Draw();
	
	void OBJExport(int frame);

};

#endif
