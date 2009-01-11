#include "Flock.h"

#include "Boid.h"
#include "World.h"
#include "Particle.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <GL/gl.h>

/*!
\file Flock.cpp
\brief contains methods for the flock class
\author Michael Jones
\version 1
\date 06/02/06
*/

namespace Flock {

/* Constructor:
*  ---------------------
*	Sets default values for flock properties
*/
Flock::Flock(int fID, World& theContainer)
 :	m_id( fID ),
	m_container( theContainer ),
	m_rank( 0 ),
	m_containmentAcc( 500 ),
	m_colour( 1.0f, 1.0f, 1.0f, 1.0f ),
	m_behaviour( 1.0f, 3.0f ),
	m_boidTR( 5.0f ),
	m_objectTR( 10.0f ),
	m_fleeTR( 10.f ),
	m_gravity( 0.0f, -9.8f, 0.0f ),
	m_null( 0.0f, 0.0f, 0.0f )
{

}



/* Clamp function:
*  ---------------
*	Receives a vector and if the length exceeds the maxValue
*	it resizes the vector to the maxValue.
*/
void Flock::Clamp( Imath::V3f &value, float maxValue) 
{
	if( value.length() > maxValue )	
	{
		value.normalize();
		value = value * maxValue;
	}
}

/* Multiple Nearest Neighbours method:
*  -----------------------------------
*	Finds a user-specified number of nearest neighbours to the 
*	boid passed to the function. It returns a vector containing
*	pointers to those nearest boids.
*/
void Flock::NearestNeighbours(std::vector<Boid*>& neighbourBoids, Boid* homeBoid, int numNeighbours)
{
	// Create a copy of the current boid vector so that
	// elements can be deleted as needed.
	std::vector<Boid*> dummyBoids;
	dummyBoids.assign(m_boids.begin(), m_boids.end());
	
	// Set iterators
	std::vector<Boid*>::iterator currentBoid = dummyBoids.begin();
	std::vector<Boid*>::iterator endBoid = dummyBoids.end();
	std::vector<Boid*>::iterator nearestBoid;

	// Loop through the dummyBoid vector and delete the homeBoid
	// so that it cannot appear as its own neighbour
	while(currentBoid != endBoid)
	{
		if((*currentBoid)->id() == homeBoid->id())
		{
			dummyBoids.erase(currentBoid);
			endBoid = dummyBoids.end();
		}	
		else
		{
			++currentBoid;
		}
	}

	Imath::V3f distVec;
	float distance;

	// Cycle through the flock once for each neighbour you wish to find
	for(int i=0; i<numNeighbours; ++i)
	{		
		currentBoid = dummyBoids.begin();
		nearestBoid = currentBoid;
		endBoid = dummyBoids.end();

		// Set minDistance initial value to the distance of the first boid
		// in the vector simply as a starting point. 
		distVec = (*currentBoid)->pos() - homeBoid->pos();
		distance = distVec.length();
	
		float minDistance = distance;
		nearestBoid = currentBoid;


		while(currentBoid != endBoid)
		{
			// Find distance to the flock mate
			distVec = (*currentBoid)->pos() - homeBoid->pos();
			distance = distVec.length();

			// If that distance is less than the minDistance 
			// then set as new minDistance and remember the Boid (as nearestBoid)
			if(distance < minDistance)
			{
				nearestBoid = currentBoid;
				minDistance = distance;
			}

			++currentBoid;
		}

		// Cycle is finished, so store nearestBoid in the neighbour vector
		// and delete the nearestBoid from the dummyBoids so that the next
		// nearest can be found on the next cycle.
		
		neighbourBoids.push_back((*nearestBoid));

		dummyBoids.erase(nearestBoid);
	}
}


/* GetFlockCentre:
*  ---------------------
*	Sums up all the boids' position vectors and 
*	divides them by the total number of boids to 
*	get an average position.
*/
void Flock::GetFlockCentre()
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	m_flockCentre.setValue(0,0,0);

	int count = 0;

	// Cycle through boids
	while(currentBoid != endBoid)
	{
		// Sum up positions
		m_flockCentre = m_flockCentre + (*currentBoid)->pos();
		++count;
		++currentBoid;
	}
	
	// Divide by number of boids to get the average.
	m_flockCentre = m_flockCentre / count;
	
}


/* Local Flock Centring:
*  ---------------------
*	Adds an acceleration to each of the boids to guide them
*	towards the average position of their local flock mates.
*	This helps to keep the flock together.
*/
void Flock::LocalFlockCentring()
{
	if( m_numMembers > 1 )
	{
		// Create the vector to store the local flock mates.
		std::vector<Boid*> nearestNeighbours;
	
		std::vector<Boid*>::iterator currentNeigh;
		std::vector<Boid*>::iterator endNeigh;
		
		std::vector<Boid*>::iterator currentBoid = m_boids.begin();
		std::vector<Boid*>::iterator endBoid = m_boids.end();
	
		Imath::V3f AveragePos(0,0,0);
		Imath::V3f Accelerate;
	
		int numNeighbours = 5;
		
		if(numNeighbours > m_numMembers-1)
			numNeighbours = m_numMembers - 1; 
		
	
		// Cycle through all the boids in the flock.
		while(currentBoid != endBoid)
		{
			// Method populates the nearestNeighbours vector with 
			// pointers to the nearest 'n' flock mates of the current boid.
			NearestNeighbours(nearestNeighbours, (*currentBoid), numNeighbours);
	
			currentNeigh = nearestNeighbours.begin();
			endNeigh = nearestNeighbours.end();
	
			//  Cycle through the neighbours and add up their position vectors.
			while(currentNeigh != endNeigh)
			{
				AveragePos = AveragePos + (*currentNeigh)->pos();
				++currentNeigh;
			}
	
			// Divide the total position vector by the number of neighbour
			// to find the local average position
			AveragePos = AveragePos / numNeighbours;
		
			// Use the average position and the boid's position to create
			// a vector from the boid to the local flock centre. Use it as
			// as acceleration.
			Accelerate = AveragePos - (*currentBoid)->pos();
	
			Accelerate = Accelerate * m_behaviour.localFC.scale;
			// Clamp it off if it is too high.
			Clamp( Accelerate, m_behaviour.localFC.max );
	
			// Add it to the boid's current acceleration.
			(*currentBoid)->accelerate( Accelerate );
	
			nearestNeighbours.clear();
			++currentBoid;
		}
	}
}


/* Global Flock Centring:
*  ----------------------
*	Finds the average position of the entire flock
*	and accelerates each boid towards it.
*/
void Flock::GlobalFlockCentring()	// Clamped
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	Imath::V3f Accelerate;
	
	// Cycle through boids
	while(currentBoid != endBoid)
	{
		// Generate acceleration from difference between boid pos and the flock centre pos.
		Accelerate = m_flockCentre - (*currentBoid)->pos();

		// Scale and clamp appropriately
		Accelerate = Accelerate * m_behaviour.globalFC.scale;
		Clamp(Accelerate, m_behaviour.globalFC.max);

		// Add accel to current boids accel vector.
		(*currentBoid)->accelerate( Accelerate );

		++currentBoid;
	}
}


/* Goal Flock Centring:
*  ---------------------
*	Adds an acceleration to each of the boids to guide them
*	towards the position of the flock's goal.
*/
void Flock::GoalFlockCentring(Imath::V3f &target)	// Clamped
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	Imath::V3f Accelerate;

	// cycle through all the boids
	while(currentBoid != endBoid)
	{
		// Generate acceleration from difference between boid pos and the goal pos.
		Accelerate = target - (*currentBoid)->pos();
		
		// Scale and clamp appropriately.
		Accelerate = Accelerate * m_behaviour.goalFC.scale;
		Clamp(Accelerate, m_behaviour.goalFC.max);
		
		(*currentBoid)->accelerate( Accelerate );

		++currentBoid;
	}
}

/* Collision Avoidance:
*  --------------------
*	For each boid this tests if any flock mates are within 
*	a set radius 'boidTR' and creates an acceleration away
*	from them where necessary.
*/
void Flock::CollisionAvoidance()	// Clamped
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator otherBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Accelerate = m_null;

	int count = 0;

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		// For each boid cycle through all the boids
		while(otherBoid != endBoid)
		{
			// Check to make sure the boid isn't the one we're
			// testing against.
			if((*otherBoid)->id() != (*currentBoid)->id())
			{
				distVec =  (*currentBoid)->pos() - (*otherBoid)->pos();
				distance = distVec.length();
				
				// Check to see if distance to otherBoid is within test radius BoidTR.
				if(distance < m_boidTR)
				{
					// Create an acceleration proportional to the distance to the otherBoid.
					Accelerate = Accelerate + (distVec / (distance * distance));
					++count;
				}
			
			}
			++otherBoid;
		}

		Accelerate = Accelerate * m_behaviour.collisionAvoidance.scale;
		// Clamp it off if it is too high.
		Clamp(Accelerate, m_behaviour.collisionAvoidance.max);

		// Add it to the boid's current acceleration.
		(*currentBoid)->accelerate( Accelerate );

		otherBoid = m_boids.begin();
		Accelerate = m_null; // Comment out for cool flocking.
		++currentBoid;
	}
}

/* Local Velocity Matching:
*  ------------------------
*	Finds average of neighbouring flock mates' velocities
*	and attempts to match the boid's velocity to the 
*	average.//std::vector<Boid>::iterator
*/
void Flock::VelMatching()	// Clamped
{
	if(m_numMembers > 1)
	{
		
		// Create the vector to store the local flock mates.
		std::vector<Boid*> nearestNeighbours;
	
		std::vector<Boid*>::iterator currentNeigh;
		std::vector<Boid*>::iterator endNeigh;
		
		std::vector<Boid*>::iterator currentBoid = m_boids.begin();
		std::vector<Boid*>::iterator endBoid = m_boids.end();
	
		Imath::V3f Velocity;
		Imath::V3f Accelerate;

		// Cycle through all the boids in the flock.
		while(currentBoid != endBoid)
		{
			int numNeighbours = 10;
		
			if(numNeighbours > m_numMembers-1)
				numNeighbours = m_numMembers - 1; 
		
			// Method populates the nearestNeighbours vector with 
			// pointers to the nearest 'n' flock mates of the current boid.
			NearestNeighbours(nearestNeighbours, (*currentBoid), numNeighbours);
	
			currentNeigh = nearestNeighbours.begin();
			endNeigh = nearestNeighbours.end();
	
			//  Cycle through the neighbours and add up their velocity vectors.
			while(currentNeigh != endNeigh)
			{
				Velocity = Velocity + (*currentNeigh)->vel();
				++currentNeigh;
			}
	
			// Divide the total velocity vector by the number of neighbour
			// to find the local velocity average
			Velocity = Velocity / numNeighbours;
		
			// Use the average velocity and the boid's velocity to create
			// a vector from the boid to the local flock centre. Use it as
			// as acceleration.
			Accelerate = Velocity - (*currentBoid)->vel();
		
			Accelerate = Accelerate * m_behaviour.velocityMatching.scale;
			// Clamp it off if it is too high.
			Clamp(Accelerate, m_behaviour.velocityMatching.max);
	
			// Add it to the boid's current acceleration.
			(*currentBoid)->accelerate( Accelerate );
	
			nearestNeighbours.clear();
			++currentBoid;
		}
	}
	
}

/* Central Object Avoidance:
*  -------------------------
*	Works with a radial force repelling the boids from
*	any objects in the scene. Produces unrealistic results
*	and has been replaced by the spherical object avoidance below.
*/
void Flock::CentralObjectAvoidance()
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	std::vector<Object*>::iterator currentObject = m_container.objects.begin();
	std::vector<Object*>::iterator endObject = m_container.objects.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Accelerate = m_null;

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		while(currentObject != endObject)
		{
			distVec =  (*currentBoid)->pos() - (*currentObject)->pos();
			distance = distVec.length();	
			
			// Check to see if distance to otherBoid is within test radius BoidTR.
			if(distance < m_objectTR)
			{

				// Create an acceleration proportional to the distance to the otherBoid.
				Accelerate = Accelerate + (distVec / (distance * distance));
			}
			++currentObject;
		}
	
		Accelerate = Accelerate * m_behaviour.objectAvoidance.scale;
		// Clamp it off if it is too high.
		Clamp(Accelerate, m_behaviour.objectAvoidance.max);
	
		// Add it to the boid's current acceleration.
		(*currentBoid)->accelerate( Accelerate );

		currentObject = m_container.objects.begin();

		Accelerate = m_null; // Comment out for cool flocking.
		++currentBoid;
	}
}

/* Cylindrical Object Avoidance:
*  -----------------------------
*	A method to navigate around columns in the world.
*	Regrettably this has not been fully implemented.
*/
void Flock::CylindricalObjectAvoidance()
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	std::vector<Object*>::iterator currentObject = m_container.objects.begin();
	std::vector<Object*>::iterator endObject = m_container.objects.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Imath::V3f TempRight;
	Imath::V3f TempLeft;
	Accelerate = m_null;

	Imath::V3f Up;
	Up.setValue(0,1,0);

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		while(currentObject != endObject)
		{
			distVec =  (*currentObject)->pos() - (*currentBoid)->pos();
			distance = distVec.length();
			
			// Check to see if distance to object is within test radius ObjectTR.
			if(distance < m_objectTR)
			{
				// test to see if object is infront of boid
				if((*currentBoid)->vel().dot(distVec) > 0)
				{
					TempRight = distVec.cross(Up);

					// test to see if boid should loop to left or right
					if(TempRight.dot((*currentBoid)->vel()) > 0)
						Accelerate = Accelerate + (TempRight/distance);
					else 
						Accelerate = Accelerate + (distVec.cross(-Up)/distance);
				}
			}
			++currentObject;
		}
	
		Accelerate = Accelerate * m_behaviour.objectAvoidance.scale;
		// Clamp it off if it is too high.
		Clamp(Accelerate, m_behaviour.objectAvoidance.max);
	
		// Add it to the boid's current acceleration.
		(*currentBoid)->accelerate( Accelerate );

		currentObject = m_container.objects.begin();

		Accelerate = m_null; // Comment out for cool flocking.
		++currentBoid;
	}
}

/* Spherical Object Avoidance:
*  ---------------------------
*	A method for navigating around spherical objects in the
*	world. Works with vector products to smoothly move the
*	boid around the object without slowing it down.
*/
void Flock::SphericalObjectAvoidance()
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	std::vector<Object*>::iterator currentObject = m_container.objects.begin();
	std::vector<Object*>::iterator endObject = m_container.objects.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Imath::V3f TempRight;
	Imath::V3f TempLeft;
	Accelerate = m_null;

	Imath::V3f inPlane;
	float inPlaneFactor;

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		while(currentObject != endObject)
		{
			distVec =  (*currentObject)->pos() - (*currentBoid)->pos();
			distance = distVec.length();
			
			// Check to see if distance to object is within test radius ObjectTR.
			if(distance < m_objectTR)
			{
				// test to see if object is infront of boid
				if((*currentBoid)->vel().dot(distVec) > 0)
				{
					Imath::V3f normVel = (*currentBoid)->vel();
					normVel.normalize();
					
					Imath::V3f normDist = distVec;
					normDist.normalize();
				
					
					inPlaneFactor = normDist.dot(normVel);
					
					// Create vector perpendicular to boids velocity
					inPlane = (normVel*inPlaneFactor) - normDist;
					
					float inPlaneLength = inPlane.length();

					// Create accleration from vector
					Imath::V3f Accel = inPlane/(inPlaneLength*inPlaneLength*inPlaneLength*inPlaneLength);
					
					// Scale down acceleration if boids is far from object or already to the side of the object
					Accel = Accel * (1-(distance/m_objectTR)) * inPlaneFactor;
					
					Accelerate = Accelerate + Accel;
					
				}
			}
			++currentObject;
		}
	
		Accelerate = Accelerate * m_behaviour.objectAvoidance.scale;
		// Clamp it off if it is too high.
		Clamp(Accelerate, m_behaviour.objectAvoidance.max);
	
		// Add it to the boid's current acceleration.
		(*currentBoid)->accelerate( Accelerate );

		currentObject = m_container.objects.begin();

		Accelerate = m_null; // Comment out for cool flocking.
		++currentBoid;
	}
}

/* Hunt:
*  -----
*	The flock searches the world for any flocks that
*	are lower in the food chain and accelerates towards
*	the nearest prey boid to the flock centre.
*/
void Flock::Hunt()
{
	std::vector<Boid*> preyBoids;	

	std::vector<Flock*>::iterator otherFlock = m_container.flocks.begin();
	std::vector<Flock*>::iterator endFlock = m_container.flocks.end();
		
	std::vector<Boid*>::iterator currentLocalBoid = m_boids.begin();
	std::vector<Boid*>::iterator endLocalBoid = m_boids.end();

	
	if(m_rank != 0)
	{
		for(; otherFlock != endFlock; ++otherFlock)
		{
			// check for m_id is unnecessary as no flock will have a rank less than its own but it is included for completeness.
			if((*otherFlock)->m_rank < m_rank && ((*otherFlock)->m_id != m_id && !(*otherFlock)->m_boids.empty()))  
			{
	
				// Create a copy of the current boid vector so that
				// elements can be deleted as needed.
				std::vector<Boid*> dummyBoids;
				dummyBoids.assign((*otherFlock)->m_boids.begin(), (*otherFlock)->m_boids.end());
				
				// Set iterators
				std::vector<Boid*>::iterator currentBoid;
				std::vector<Boid*>::iterator endBoid = dummyBoids.end();
				std::vector<Boid*>::iterator nearestBoid;
	
				Imath::V3f distVec;
				float distance;
	
				int numPrey = 1;
				
				if((*otherFlock)->m_numMembers < numPrey)
					numPrey = (*otherFlock)->m_numMembers;
				
				// Cycle through the flock once for each neighbour you wish to find
				for(int i=0; i<numPrey; ++i)
				{
					currentBoid = dummyBoids.begin();
					endBoid = dummyBoids.end();
					nearestBoid = currentBoid;
			
					// Set minDistance initial value to the distance of the first boid
					// in the vector simply as a starting point. 
					distVec = (*currentBoid)->pos() - m_flockCentre;
					distance = distVec.length();
				
					float minDistance = distance;
					nearestBoid = currentBoid;
			
			
					while(currentBoid != endBoid)
					{
						// Find distance to the flock mate
						distVec = (*currentBoid)->pos() - m_flockCentre;
						distance = distVec.length();
			
						// If that distance is less than the minDistance 
						// then set as new minDistance and remember the Boid (as nearestBoid)
						if(distance < minDistance)
						{
							nearestBoid = currentBoid;
							minDistance = distance;
						}
			
						++currentBoid;
					}
			
					// Cycle is finished, so store nearestBoid in the neighbour vector
					// and delete the nearestBoid from the dummyBoids so that the next
					// nearest can be found on the next cycle.
					
					preyBoids.push_back((*nearestBoid));
			
					dummyBoids.erase(nearestBoid);
				}
				
				currentBoid = preyBoids.begin();
				endBoid = preyBoids.end();
			
				Imath::V3f AveragePreyPos(0,0,0);
				Imath::V3f Accelerate;
			
				int count = 0;
			
				// Cycle
				for(; currentBoid != endBoid; ++currentBoid)
				{
					AveragePreyPos = AveragePreyPos + (*currentBoid)->pos();
					++count;
				}
				
				
				AveragePreyPos = AveragePreyPos / count;
							
				currentLocalBoid = m_boids.begin();
			
				for(; currentLocalBoid != endLocalBoid; ++currentLocalBoid)
				{
					// Accelerate each boid towards the prey
					Accelerate = AveragePreyPos - (*currentLocalBoid)->pos();
			
					Accelerate = Accelerate * m_behaviour.hunt.scale;
					Clamp(Accelerate, m_behaviour.hunt.max);
			
					(*currentLocalBoid)->accelerate( Accelerate );
			
				}
			}
		} 
	}
}

/* Kill:
*  -----
*	Any collisions between predator and prey boids
*	result in the prey boid "dying". To emphasise the
*	effect, a small explosion of particles is generated
*	at the point of death.
*/
void Flock::Kill()
{

	std::vector<Flock*>::iterator otherFlock = m_container.flocks.begin();
	std::vector<Flock*>::iterator endFlock = m_container.flocks.end();

	std::vector<Boid*>::iterator otherBoid;
	std::vector<Boid*>::iterator endBoid;
		
	std::vector<Boid*>::iterator currentLocalBoid = m_boids.begin();
	std::vector<Boid*>::iterator endLocalBoid = m_boids.end();

	Imath::V3f difference;
	float distance;
	
	if(m_rank != 0)
	{			
		for(; otherFlock != endFlock; ++otherFlock)
		{
			// check for m_id is unnecessary as no flock will have a m_rank less than its own but it is included for completeness.
			if((*otherFlock)->m_rank < m_rank && ((*otherFlock)->m_id != m_id && !(*otherFlock)->m_boids.empty()))  
			{
				// cycle through all boids in current flock
				for(; currentLocalBoid != endLocalBoid; ++currentLocalBoid)
				{
					otherBoid = (*otherFlock)->m_boids.begin();
					endBoid = (*otherFlock)->m_boids.end();
					
					// cycle through all boids in other flocks
					while(otherBoid != endBoid)
					{
						difference = (*currentLocalBoid)->pos() - (*otherBoid)->pos();
						distance = difference.length();
						
						// Test is see if predator and prey boids are close.
						if(distance < 0.7)
						{
							// Create a shower of particles at boid death position
							for(int i=0; i <30; ++i)
							{
								Particle* newParticle = new Particle((*otherBoid)->pos(), (*otherFlock)->m_colour, m_container.minY);
								m_particles.push_back(newParticle);
							}
							// remove dead boid from its flock 
							(*otherFlock)->m_boids.erase(otherBoid);
							(*otherFlock)->m_numMembers -= 1;
							endBoid = (*otherFlock)->m_boids.end();
						} else {
							// only increment is nothing is removed from vector
							++otherBoid;
						}
					}
				}
				
				currentLocalBoid = m_boids.begin();
			}
		}
	}
}

/* Flee:
*  -----
*	The flock searches for any predator boids
*	and accelerates away from them if they are 
*	closer than a certain distance.
*/
void Flock::Flee()
{
	std::vector<Flock*>::iterator otherFlock = m_container.flocks.begin();
	std::vector<Flock*>::iterator endFlock = m_container.flocks.end();
	
	std::vector<Boid*>::iterator otherBoid;
	std::vector<Boid*>::iterator endBoid;
	
	std::vector<Boid*>::iterator currentLocalBoid = m_boids.begin();
	std::vector<Boid*>::iterator endLocalBoid = m_boids.end();
	
	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Accelerate = m_null;
	
	int count = 0;

	for(; otherFlock != endFlock; ++otherFlock)
	{
		// check for m_id is unnecessary as no flock will have a m_rank greater than its own but it is included for completeness. 
		if((*otherFlock)->m_rank > m_rank && ((*otherFlock)->m_id != m_id && !(*otherFlock)->m_boids.empty())) 
		{
			// Cycle through all the boids.
			for(; currentLocalBoid != endLocalBoid; ++currentLocalBoid)
			{
				otherBoid = (*otherFlock)->m_boids.begin();
				endBoid = (*otherFlock)->m_boids.end();
				
				// For each boid cycle through all the boids
				for(; otherBoid != endBoid; ++otherBoid)
				{
						distVec =  (*currentLocalBoid)->pos() - (*otherBoid)->pos();
						distance = distVec.length();
						
						// Check to see if distance to otherBoid is within test radius BoidTR.
						if( distance < m_fleeTR )
						{
							// Create an acceleration proportional to the distance to the otherBoid.
							Accelerate = Accelerate + (distVec / (distance * distance));
							++count;
						}
				}
		
				Accelerate = Accelerate * m_behaviour.flee.scale;
				// Clamp it off if it is too high.
				Clamp(Accelerate, m_behaviour.flee.max);
		
				// Add it to the boid's current acceleration.
				(*currentLocalBoid)->accelerate( Accelerate );
		
				Accelerate = m_null;
			}
		}
	} 
}



/* Contain:
*  --------
*	Accelerates any boids that leave the world, back into the world.
*/
void Flock::Contain()
{
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	// Cycle through all the boids in the flock.
	while(currentBoid != endBoid)
	{
		// test to see if boid is out of bounds then accelerate back into world if necessary
		if((*currentBoid)->pos().x > m_container.maxX)
			(*currentBoid)->accelerate(
					Imath::V3f( - m_containmentAcc * ((*currentBoid)->pos().x - m_container.maxX), 0.0f, 0.0f )
					); 
		else if((*currentBoid)->pos().x < m_container.minX)
			(*currentBoid)->accelerate(
					Imath::V3f( - m_containmentAcc * ((*currentBoid)->pos().x - m_container.minX), 0.0f, 0.0f )
					);
		else if((*currentBoid)->pos().y > m_container.maxY)
			(*currentBoid)->accelerate(
					Imath::V3f( 0.0f, - m_containmentAcc * ((*currentBoid)->pos().y - m_container.maxY), 0.0f )
					);
			
		// accelerate any boid that's close to the ground upwards
		else if((*currentBoid)->pos().y < m_container.minY + 5)
		{
			(*currentBoid)->accelerate( Imath::V3f( 0.0f, m_containmentAcc, 0.0f ) );
		}
		else if((*currentBoid)->pos().z > m_container.maxZ)
			(*currentBoid)->accelerate(
					Imath::V3f( 0.0f, 0.0f, - m_containmentAcc * ((*currentBoid)->pos().z - m_container.maxZ ) )
					);
		else if((*currentBoid)->pos().z < m_container.minZ)
			(*currentBoid)->accelerate( 
					Imath::V3f( 0.0f, 0.0f, - m_containmentAcc * ((*currentBoid)->pos().z - m_container.minZ) )
					);
		
		++currentBoid;
	}
}

/* Clear:
*  ------
*	Empties the STL vector contain the boids in the flock.
*	Important for reseting the system.
*/
void Flock::Clear()
{
	m_boids.clear();
}

/* AddBoid:
*  --------
*	Inserts the passed pointer into the vector of boids
*	stored within the flock.
*/
void Flock::AddBoid(Boid* addBoid)
{
	m_boids.push_back(addBoid);
}

/* ParticleUpdate:
*  ---------------
*	Loops through all the particles associated with the flock
*	and calls the appropriate update method.
*/
void Flock::ParticleUpdate()
{
	std::vector<Particle*>::iterator currentPart = m_particles.begin();
	std::vector<Particle*>::iterator endPart = m_particles.end();
	
	while(currentPart != endPart)
	{
		// test to see if particle is below ground level of world, if so delete it.
		if((*currentPart)->Pos.y < m_container.minY)
		{
			delete (*currentPart);
			m_particles.erase(currentPart);
			endPart = m_particles.end();
		}
		else
		{
			(*currentPart)->Update();
			++currentPart;
		}
	}
}

/* Update:
*  ---------------
*	Loops through all the boids in the flock and 
*	calls the behaviours before calculating the 
*	effects of the acceleration on the boid at each time step.
*	Roll, Pitch and Yaw are also calculated here.
*/
void Flock::Update(Imath::V3f &target)
{
	ParticleUpdate();
	
	// Check flock isn't empty (ie. already hunted to extinction)
	if(m_boids.empty()) { return; }
	
	Kill(); // kill any boids before they're processed

	// Get info
	GetFlockCentre();

	// Run behaviours 
	LocalFlockCentring();
	GlobalFlockCentring();
	CollisionAvoidance();
	VelMatching();
	SphericalObjectAvoidance();
	GoalFlockCentring(target);
	Contain();
	
	Hunt();
	Flee();
	

	// Update
	std::vector<Boid*>::iterator currentBoid = m_boids.begin();
	std::vector<Boid*>::iterator endBoid = m_boids.end();

	float Vx, Vy, Vz;

	for( ; currentBoid != endBoid; ++currentBoid )
	{
		(*currentBoid)->update( m_behaviour );
	}
}


/* Draw:
*  ---------------
*	Loops through all the boids and particles associated with the flock
*	and calls the appropriate draw methods.
*/
void Flock::Draw()
{
	if(m_boids.empty()) { return; }

	std::vector<Boid*>::iterator currentB = m_boids.begin();
	std::vector<Boid*>::iterator endB = m_boids.end();
	
	while(currentB != endB)
	{
		glColor4f( m_colour.r, m_colour.b, m_colour.g, m_colour.a );
		(*currentB)->Draw(m_container.minY);
		++currentB;
	}
	
	std::vector<Particle*>::iterator currentPart = m_particles.begin();
	std::vector<Particle*>::iterator endPart = m_particles.end();
	
	for(; currentPart != endPart; ++currentPart)
	{
		(*currentPart)->Draw();
	}

}



void Flock::OBJExport(int frame)
{
// 	if(boids.empty()) { return; }

	std::vector<Boid*>::iterator currentB = m_boids.begin();
	std::vector<Boid*>::iterator endB = m_boids.end();

	int offset = 1;

	std::ostringstream objName;
	objName << "export/flock" << m_id << "." << std::setfill ('0') << std::setw(4) << frame << ".obj";
	
	std::ofstream OBJFile;
	OBJFile.open(objName.str().c_str(), std::fstream::trunc);
	
		for(; currentB != endB; ++currentB)
		{
			OBJFile << "v " <<  (*currentB)->pos().x << " " << (*currentB)->pos().y << " " << (*currentB)->pos().z << std::endl;
			OBJFile << "vn " <<  (*currentB)->vel().x << " " << (*currentB)->vel().y << " " << (*currentB)->vel().z << std::endl;
			
			OBJFile << "f " << offset << "//" << offset << std::endl;
		
			++offset;
			
		}
	
	OBJFile.close();
}

} // Flock

