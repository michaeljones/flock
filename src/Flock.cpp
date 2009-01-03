#include "Flock.h"
#include "Boid.h"
#include "World.h"
#include "Particle.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

/*!
\file Flock.cpp
\brief contains methods for the flock class
\author Michael Jones
\version 1
\date 06/02/06
*/

// Empty constructor method
Flock::Flock()
{

}

/* Constructor:
*  ---------------------
*	Sets default values for flock properties
*/
Flock::Flock(int fID, World& theContainer)
{
	container = &theContainer;
	flockID = fID;
	
	rank = 0;
	bDepth = 3;
	bScale = 2;
	
	containmentAcc = 500;
	
	colour.setValue( 1.0, 1.0, 1.0, 1.0 );

	// Default test radius values	
	boidTR = 5.0;
	objectTR = 10.0;
	fleeTR = 10.0;
	
	// Default min, max and scale values
	maxVel = 5.0;
	minVel = 0.0;
	maxAcc = 1.0;	

	scaleLocalFC = scaleGlobalFC = scaleGoalFC = scaleCA = scaleOA = scaleVM = 1.0;
	maxLocalFC   = maxGlobalFC   = maxGoalFC   = maxCA   = maxOA   = maxVM   = 3.0;
	
	scaleHunt = scaleFlee = 1.0;
	maxHunt = maxFlee = 3.0;
	
	// Gravitational down direction
	gravity.setValue(0.0, -9.8, 0.0);

	Null.setValue( 0.0, 0.0, 0.0 );
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
	dummyBoids.assign(boids.begin(), boids.end());
	
	// Set iterators
	std::vector<Boid*>::iterator currentBoid = dummyBoids.begin();
	std::vector<Boid*>::iterator endBoid = dummyBoids.end();
	std::vector<Boid*>::iterator nearestBoid;

	// Loop through the dummyBoid vector and delete the homeBoid
	// so that it cannot appear as its own neighbour
	while(currentBoid != endBoid)
	{
		if((*currentBoid)->boidID == homeBoid->boidID)
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
		distVec = (*currentBoid)->Pos - homeBoid->Pos;
		distance = distVec.length();
	
		float minDistance = distance;
		nearestBoid = currentBoid;


		while(currentBoid != endBoid)
		{
			// Find distance to the flock mate
			distVec = (*currentBoid)->Pos - homeBoid->Pos;
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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	flockCentre.setValue(0,0,0);

	int count = 0;

	// Cycle through boids
	while(currentBoid != endBoid)
	{
		// Sum up positions
		flockCentre = flockCentre + (*currentBoid)->Pos;
		++count;
		++currentBoid;
	}
	
	// Divide by number of boids to get the average.
	flockCentre = flockCentre / count;
	
}


/* Local Flock Centring:
*  ---------------------
*	Adds an acceleration to each of the boids to guide them
*	towards the average position of their local flock mates.
*	This helps to keep the flock together.
*/
void Flock::LocalFlockCentring()
{
	if(numMembers > 1)
	{
		// Create the vector to store the local flock mates.
		std::vector<Boid*> nearestNeighbours;
	
		std::vector<Boid*>::iterator currentNeigh;
		std::vector<Boid*>::iterator endNeigh;
		
		std::vector<Boid*>::iterator currentBoid = boids.begin();
		std::vector<Boid*>::iterator endBoid = boids.end();
	
		Imath::V3f AveragePos(0,0,0);
		Imath::V3f Accelerate;
	
		int numNeighbours = 5;
		
		if(numNeighbours > numMembers-1)
			numNeighbours = numMembers - 1; 
		
	
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
				AveragePos = AveragePos + (*currentNeigh)->Pos;
				++currentNeigh;
			}
	
			// Divide the total position vector by the number of neighbour
			// to find the local average position
			AveragePos = AveragePos / numNeighbours;
		
			// Use the average position and the boid's position to create
			// a vector from the boid to the local flock centre. Use it as
			// as acceleration.
			Accelerate = AveragePos - (*currentBoid)->Pos;
	
			Accelerate = Accelerate * scaleLocalFC;
			// Clamp it off if it is too high.
			Clamp(Accelerate, maxLocalFC);
	
			// Add it to the boid's current acceleration.
			(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;
	
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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	Imath::V3f Accelerate;
	
	// Cycle through boids
	while(currentBoid != endBoid)
	{
		// Generate acceleration from difference between boid pos and the flock centre pos.
		Accelerate = flockCentre - (*currentBoid)->Pos;

		// Scale and clamp appropriately
		Accelerate = Accelerate * scaleGlobalFC;
		Clamp(Accelerate, maxGlobalFC);

		// Add accel to current boids accel vector.
		(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;

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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	Imath::V3f Accelerate;

	// cycle through all the boids
	while(currentBoid != endBoid)
	{
		// Generate acceleration from difference between boid pos and the goal pos.
		Accelerate = target - (*currentBoid)->Pos;
		
		// Scale and clamp appropriately.
		Accelerate = Accelerate * scaleGoalFC;
		Clamp(Accelerate, maxGoalFC);
		
		(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;

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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator otherBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Accelerate = Null;

	int count = 0;

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		// For each boid cycle through all the boids
		while(otherBoid != endBoid)
		{
			// Check to make sure the boid isn't the one we're
			// testing against.
			if((*otherBoid)->boidID != (*currentBoid)->boidID)
			{
				distVec =  (*currentBoid)->Pos - (*otherBoid)->Pos;
				distance = distVec.length();
				
				// Check to see if distance to otherBoid is within test radius BoidTR.
				if(distance < boidTR)
				{
					// Create an acceleration proportional to the distance to the otherBoid.
					Accelerate = Accelerate + (distVec / (distance * distance));
					++count;
				}
			
			}
			++otherBoid;
		}

		Accelerate = Accelerate * scaleCA;
		// Clamp it off if it is too high.
		Clamp(Accelerate, maxCA);

		// Add it to the boid's current acceleration.
		(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;

		otherBoid = boids.begin();
		Accelerate = Null; // Comment out for cool flocking.
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
	if(numMembers > 1)
	{
		
		// Create the vector to store the local flock mates.
		std::vector<Boid*> nearestNeighbours;
	
		std::vector<Boid*>::iterator currentNeigh;
		std::vector<Boid*>::iterator endNeigh;
		
		std::vector<Boid*>::iterator currentBoid = boids.begin();
		std::vector<Boid*>::iterator endBoid = boids.end();
	
		Imath::V3f Velocity;
		Imath::V3f Accelerate;

		// Cycle through all the boids in the flock.
		while(currentBoid != endBoid)
		{
			int numNeighbours = 10;
		
			if(numNeighbours > numMembers-1)
				numNeighbours = numMembers - 1; 
		
			// Method populates the nearestNeighbours vector with 
			// pointers to the nearest 'n' flock mates of the current boid.
			NearestNeighbours(nearestNeighbours, (*currentBoid), numNeighbours);
	
			currentNeigh = nearestNeighbours.begin();
			endNeigh = nearestNeighbours.end();
	
			//  Cycle through the neighbours and add up their velocity vectors.
			while(currentNeigh != endNeigh)
			{
				Velocity = Velocity + (*currentNeigh)->Vel;
				++currentNeigh;
			}
	
			// Divide the total velocity vector by the number of neighbour
			// to find the local velocity average
			Velocity = Velocity / numNeighbours;
		
			// Use the average velocity and the boid's velocity to create
			// a vector from the boid to the local flock centre. Use it as
			// as acceleration.
			Accelerate = Velocity - (*currentBoid)->Vel;
		
			Accelerate = Accelerate * scaleVM;
			// Clamp it off if it is too high.
			Clamp(Accelerate, maxVM);
	
			// Add it to the boid's current acceleration.
			(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;
	
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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	std::vector<Object*>::iterator currentObject = container->objects.begin();
	std::vector<Object*>::iterator endObject = container->objects.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Accelerate = Null;

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		while(currentObject != endObject)
		{
			distVec =  (*currentBoid)->Pos - (*currentObject)->Pos;
			distance = distVec.length();	
			
			// Check to see if distance to otherBoid is within test radius BoidTR.
			if(distance < objectTR)
			{

				// Create an acceleration proportional to the distance to the otherBoid.
				Accelerate = Accelerate + (distVec / (distance * distance));
			}
			++currentObject;
		}
	
		Accelerate = Accelerate * scaleOA;
		// Clamp it off if it is too high.
		Clamp(Accelerate, maxOA);
	
		// Add it to the boid's current acceleration.
		(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;

		currentObject = container->objects.begin();

		Accelerate = Null; // Comment out for cool flocking.
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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	std::vector<Object*>::iterator currentObject = container->objects.begin();
	std::vector<Object*>::iterator endObject = container->objects.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Imath::V3f TempRight;
	Imath::V3f TempLeft;
	Accelerate = Null;

	Imath::V3f Up;
	Up.setValue(0,1,0);

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		while(currentObject != endObject)
		{
			distVec =  (*currentObject)->Pos - (*currentBoid)->Pos;
			distance = distVec.length();
			
			// Check to see if distance to object is within test radius ObjectTR.
			if(distance < objectTR)
			{
				// test to see if object is infront of boid
				if((*currentBoid)->Vel.dot(distVec) > 0)
				{
					TempRight = distVec.cross(Up);

					// test to see if boid should loop to left or right
					if(TempRight.dot((*currentBoid)->Vel) > 0)
						Accelerate = Accelerate + (TempRight/distance);
					else 
						Accelerate = Accelerate + (distVec.cross(-Up)/distance);
				}
			}
			++currentObject;
		}
	
		Accelerate = Accelerate * scaleOA;
		// Clamp it off if it is too high.
		Clamp(Accelerate, maxOA);
	
		// Add it to the boid's current acceleration.
		(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;

		currentObject = container->objects.begin();

		Accelerate = Null; // Comment out for cool flocking.
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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	std::vector<Object*>::iterator currentObject = container->objects.begin();
	std::vector<Object*>::iterator endObject = container->objects.end();

	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Imath::V3f TempRight;
	Imath::V3f TempLeft;
	Accelerate = Null;

	Imath::V3f inPlane;
	float inPlaneFactor;

	// Cycle through all the boids.
	while(currentBoid != endBoid)
	{
		while(currentObject != endObject)
		{
			distVec =  (*currentObject)->Pos - (*currentBoid)->Pos;
			distance = distVec.length();
			
			// Check to see if distance to object is within test radius ObjectTR.
			if(distance < objectTR)
			{
				// test to see if object is infront of boid
				if((*currentBoid)->Vel.dot(distVec) > 0)
				{
					Imath::V3f normVel = (*currentBoid)->Vel;
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
					Accel = Accel * (1-(distance/objectTR)) * inPlaneFactor;
					
					Accelerate = Accelerate + Accel;
					
				}
			}
			++currentObject;
		}
	
		Accelerate = Accelerate * scaleOA;
		// Clamp it off if it is too high.
		Clamp(Accelerate, maxOA);
	
		// Add it to the boid's current acceleration.
		(*currentBoid)->Acc = (*currentBoid)->Acc + Accelerate;

		currentObject = container->objects.begin();

		Accelerate = Null; // Comment out for cool flocking.
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

	std::vector<Flock*>::iterator otherFlock = container->flocks.begin();
	std::vector<Flock*>::iterator endFlock = container->flocks.end();
		
	std::vector<Boid*>::iterator currentLocalBoid = boids.begin();
	std::vector<Boid*>::iterator endLocalBoid = boids.end();

	
	if(rank != 0)
	{
		for(; otherFlock != endFlock; ++otherFlock)
		{
			// check for flockID is unnecessary as no flock will have a rank less than its own but it is included for completeness.
			if((*otherFlock)->rank < rank && ((*otherFlock)->flockID != flockID && !(*otherFlock)->boids.empty()))  
			{
	
				// Create a copy of the current boid vector so that
				// elements can be deleted as needed.
				std::vector<Boid*> dummyBoids;
				dummyBoids.assign((*otherFlock)->boids.begin(), (*otherFlock)->boids.end());
				
				// Set iterators
				std::vector<Boid*>::iterator currentBoid;
				std::vector<Boid*>::iterator endBoid = dummyBoids.end();
				std::vector<Boid*>::iterator nearestBoid;
	
				Imath::V3f distVec;
				float distance;
	
				int numPrey = 1;
				
				if((*otherFlock)->numMembers < numPrey)
					numPrey = (*otherFlock)->numMembers;
				
				// Cycle through the flock once for each neighbour you wish to find
				for(int i=0; i<numPrey; ++i)
				{
					currentBoid = dummyBoids.begin();
					endBoid = dummyBoids.end();
					nearestBoid = currentBoid;
			
					// Set minDistance initial value to the distance of the first boid
					// in the vector simply as a starting point. 
					distVec = (*currentBoid)->Pos - flockCentre;
					distance = distVec.length();
				
					float minDistance = distance;
					nearestBoid = currentBoid;
			
			
					while(currentBoid != endBoid)
					{
						// Find distance to the flock mate
						distVec = (*currentBoid)->Pos - flockCentre;
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
					AveragePreyPos = AveragePreyPos + (*currentBoid)->Pos;
					++count;
				}
				
				
				AveragePreyPos = AveragePreyPos / count;
							
				currentLocalBoid = boids.begin();
			
				for(; currentLocalBoid != endLocalBoid; ++currentLocalBoid)
				{
					// Accelerate each boid towards the prey
					Accelerate = AveragePreyPos - (*currentLocalBoid)->Pos;
			
					Accelerate = Accelerate * scaleHunt;
					Clamp(Accelerate, maxHunt);
			
					(*currentLocalBoid)->Acc = (*currentLocalBoid)->Acc + Accelerate;
			
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

	std::vector<Flock*>::iterator otherFlock = container->flocks.begin();
	std::vector<Flock*>::iterator endFlock = container->flocks.end();

	std::vector<Boid*>::iterator otherBoid;
	std::vector<Boid*>::iterator endBoid;
		
	std::vector<Boid*>::iterator currentLocalBoid = boids.begin();
	std::vector<Boid*>::iterator endLocalBoid = boids.end();

	Imath::V3f difference;
	float distance;
	
	if(rank != 0)
	{			
		for(; otherFlock != endFlock; ++otherFlock)
		{
			// check for flockID is unnecessary as no flock will have a rank less than its own but it is included for completeness.
			if((*otherFlock)->rank < rank && ((*otherFlock)->flockID != flockID && !(*otherFlock)->boids.empty()))  
			{
				// cycle through all boids in current flock
				for(; currentLocalBoid != endLocalBoid; ++currentLocalBoid)
				{
					otherBoid = (*otherFlock)->boids.begin();
					endBoid = (*otherFlock)->boids.end();
					
					// cycle through all boids in other flocks
					while(otherBoid != endBoid)
					{
						difference = (*currentLocalBoid)->Pos - (*otherBoid)->Pos;
						distance = difference.length();
						
						// Test is see if predator and prey boids are close.
						if(distance < 0.7)
						{
							// Create a shower of particles at boid death position
							for(int i=0; i <30; ++i)
							{
								Particle* newParticle = new Particle((*otherBoid)->Pos, (*otherFlock)->colour, container->minY);
								particles.push_back(newParticle);
							}
							// remove dead boid from its flock 
							(*otherFlock)->boids.erase(otherBoid);
							(*otherFlock)->numMembers -= 1;
							endBoid = (*otherFlock)->boids.end();
						} else {
							// only increment is nothing is removed from vector
							++otherBoid;
						}
					}
				}
				
				currentLocalBoid = boids.begin();
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
	std::vector<Flock*>::iterator otherFlock = container->flocks.begin();
	std::vector<Flock*>::iterator endFlock = container->flocks.end();
	
	std::vector<Boid*>::iterator otherBoid;
	std::vector<Boid*>::iterator endBoid;
	
	std::vector<Boid*>::iterator currentLocalBoid = boids.begin();
	std::vector<Boid*>::iterator endLocalBoid = boids.end();
	
	Imath::V3f distVec;
	float distance;

	Imath::V3f Accelerate;
	Accelerate = Null;
	
	int count = 0;

	for(; otherFlock != endFlock; ++otherFlock)
	{
		// check for flockID is unnecessary as no flock will have a rank greater than its own but it is included for completeness. 
		if((*otherFlock)->rank > rank && ((*otherFlock)->flockID != flockID && !(*otherFlock)->boids.empty())) 
		{
			// Cycle through all the boids.
			for(; currentLocalBoid != endLocalBoid; ++currentLocalBoid)
			{
				otherBoid = (*otherFlock)->boids.begin();
				endBoid = (*otherFlock)->boids.end();
				
				// For each boid cycle through all the boids
				for(; otherBoid != endBoid; ++otherBoid)
				{
						distVec =  (*currentLocalBoid)->Pos - (*otherBoid)->Pos;
						distance = distVec.length();
						
						// Check to see if distance to otherBoid is within test radius BoidTR.
						if(distance < fleeTR)
						{
							// Create an acceleration proportional to the distance to the otherBoid.
							Accelerate = Accelerate + (distVec / (distance * distance));
							++count;
						}
				}
		
				Accelerate = Accelerate * scaleFlee;
				// Clamp it off if it is too high.
				Clamp(Accelerate, maxFlee);
		
				// Add it to the boid's current acceleration.
				(*currentLocalBoid)->Acc = (*currentLocalBoid)->Acc + Accelerate;
		
				Accelerate = Null;
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
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	// Cycle through all the boids in the flock.
	while(currentBoid != endBoid)
	{
		// test to see if boid is out of bounds then accelerate back into world if necessary
		if((*currentBoid)->Pos.x > container->maxX)
			(*currentBoid)->Acc.x = (*currentBoid)->Acc.x - containmentAcc * ((*currentBoid)->Pos.x - container->maxX); 
		else if((*currentBoid)->Pos.x < container->minX)
			(*currentBoid)->Acc.x = (*currentBoid)->Acc.x - containmentAcc * ((*currentBoid)->Pos.x - container->minX);
		else if((*currentBoid)->Pos.y > container->maxY)
			(*currentBoid)->Acc.y = (*currentBoid)->Acc.y - containmentAcc * ((*currentBoid)->Pos.y - container->maxY);
			
		// accelerate any boid that's close to the ground upwards
		else if((*currentBoid)->Pos.y < container->minY + 5)
		{
			(*currentBoid)->Acc.y = (*currentBoid)->Acc.y + containmentAcc;
		}
		else if((*currentBoid)->Pos.z > container->maxZ)
			(*currentBoid)->Acc.z = (*currentBoid)->Acc.z - containmentAcc * ((*currentBoid)->Pos.z - container->maxZ);
		else if((*currentBoid)->Pos.z < container->minZ)
			(*currentBoid)->Acc.z = (*currentBoid)->Acc.z - containmentAcc * ((*currentBoid)->Pos.z - container->minZ);
		
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
	boids.clear();
}

/* AddBoid:
*  --------
*	Inserts the passed pointer into the vector of boids
*	stored within the flock.
*/
void Flock::AddBoid(Boid* addBoid)
{
	boids.push_back(addBoid);
}

/* ParticleUpdate:
*  ---------------
*	Loops through all the particles associated with the flock
*	and calls the appropriate update method.
*/
void Flock::ParticleUpdate()
{
	std::vector<Particle*>::iterator currentPart = particles.begin();
	std::vector<Particle*>::iterator endPart = particles.end();
	
	while(currentPart != endPart)
	{
		// test to see if particle is below ground level of world, if so delete it.
		if((*currentPart)->Pos.y < container->minY)
		{
			delete (*currentPart);
			particles.erase(currentPart);
			endPart = particles.end();
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
	if(boids.empty()) { return; }
	
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
	
	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	float Vx, Vy, Vz;

	for(;currentBoid != endBoid; ++currentBoid)
	{
		float preClampAcc = (*currentBoid)->Acc.length();
	
		Clamp((*currentBoid)->Acc, maxAcc);

		// Increment velocity by acceleration
		(*currentBoid)->Vel = (*currentBoid)->Vel + (*currentBoid)->Acc*0.04;

		Clamp((*currentBoid)->Vel, maxVel);

		// lower bound clamp on velocity.
		if((*currentBoid)->Vel.length() < minVel)
		{
			(*currentBoid)->Vel.normalize();
			(*currentBoid)->Vel = (*currentBoid)->Vel * minVel;
		}

		// increment position by velocity
		(*currentBoid)->Pos = (*currentBoid)->Pos + (*currentBoid)->Vel*0.04;


		
		Vx = (*currentBoid)->Vel.x;
		Vy = (*currentBoid)->Vel.y;
		Vz = (*currentBoid)->Vel.z;
		
		// Pitch
		(*currentBoid)->Dir.x = -atan(Vy/sqrt(Vx*Vx + Vz*Vz));

		// Yaw - Working
		(*currentBoid)->Dir.y = atan2(Vx, Vz);
		
		// Roll
		Imath::V3f Up(0.0, 1.0, 0.0);
		
// 		// calculate local xAxis for boid
		Imath::V3f xAxis = (*currentBoid)->Vel.cross(Up);  
		xAxis.normalize();
		
		float totalAcc = maxLocalFC + maxGlobalFC + maxGoalFC + maxCA + maxOA + maxVM + maxHunt + maxFlee;
		
		float AccWeight = preClampAcc/totalAcc;

		Imath::V3f AccNorm = (*currentBoid)->Acc;
		float tilt = xAxis.dot(AccNorm);
		tilt = bScale * tilt * AccWeight * maxAcc;
		
		// Save roll in roll vector
		(*currentBoid)->oldRoll.insert((*currentBoid)->oldRoll.begin(), tilt);
		
		// remove last roll if necessary
		if((*currentBoid)->oldRoll.size() > bDepth)
			(*currentBoid)->oldRoll.pop_back();
		

		std::vector<float>::iterator currentRoll = (*currentBoid)->oldRoll.begin();
		std::vector<float>::iterator endRoll = (*currentBoid)->oldRoll.end();

		int count = 0;
		float Roll = 0;
		
		// Average over the last 'n' rolls
		for(; currentRoll != endRoll; ++currentRoll)
		{
			Roll = Roll + (*currentRoll);
			++count;
		}

		Roll = Roll / count;
		
		(*currentBoid)->Dir.z = -atan2(Roll, -9.8);

		(*currentBoid)->Acc = Null;
	}
}


/* Draw:
*  ---------------
*	Loops through all the boids and particles associated with the flock
*	and calls the appropriate draw methods.
*/
void Flock::Draw()
{
	if(boids.empty()) { return; }

	std::vector<Boid*>::iterator currentB = boids.begin();
	std::vector<Boid*>::iterator endB = boids.end();
	
	while(currentB != endB)
	{
		// colour.Use();
		(*currentB)->Draw(container->minY);
		++currentB;
	}
	
	std::vector<Particle*>::iterator currentPart = particles.begin();
	std::vector<Particle*>::iterator endPart = particles.end();
	
	for(; currentPart != endPart; ++currentPart)
	{
		(*currentPart)->Draw();
	}

}



void Flock::OBJExport(int frame)
{
// 	if(boids.empty()) { return; }

	std::vector<Boid*>::iterator currentB = boids.begin();
	std::vector<Boid*>::iterator endB = boids.end();

	int offset = 1;

	std::ostringstream objName;
	objName << "export/flock" << flockID << "." << std::setfill ('0') << std::setw(4) << frame << ".obj";
	
	std::ofstream OBJFile;
	OBJFile.open(objName.str().c_str(), std::fstream::trunc);
	
		for(; currentB != endB; ++currentB)
		{
			OBJFile << "v " <<  (*currentB)->Pos.x << " " << (*currentB)->Pos.y << " " << (*currentB)->Pos.z << std::endl;
			OBJFile << "vn " <<  (*currentB)->Vel.x << " " << (*currentB)->Vel.y << " " << (*currentB)->Vel.z << std::endl;
			
			OBJFile << "f " << offset << "//" << offset << std::endl;
		
			++offset;
			
		}
	
	OBJFile.close();
	
}

