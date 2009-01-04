/*
*	Programming for Graphics - Flocking system
*
*	Michael Jones
*/

/*!
\file main.cpp
\brief contains the main program structure for the system
\author Michael Jones
\version 1
\date 06/02/06
*/

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>

// Custom classes
#include "Flock.h"
#include "Boid.h"
#include "World.h"
#include "Goal.h"
#include "Object.h"
#include "Particle.h"

// OpenGl and Glut includes for Linux and Mac (Darwin)
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <ImathGLU.h>

// Escape key defined for use in keyboard function
#define ESCAPE 27

typedef std::vector< Boid* >::iterator BoidIt;
typedef std::vector< Flock* >::iterator FlockIt;


// Screen Dimensions
int WIDTH = 800;
int HEIGHT = 600;

std::string Filename;

int Pause = 0;
bool useflocking = true;

int frame = 0;

World container;

// Vector containers for the Flocks and Boids used in the system
std::vector<Flock*> flocks;
std::vector<Boid*> boids;
	
// CurveFollow *targetCurve;
// Goal target(container);

std::vector<Object*> objects;

// 
//  Cam;
// enum CAMMODE{MOVEEYE,MOVELOOK,MOVEBOTH,MOVESLIDE};
// int CamMode=MOVESLIDE;
// 
// int spinxface = 0 ;
// int spinyface = 0 ;
// int spinzface = 0 ;
// int origx, origy, origz, RotateXY, RotateXZ=0;
// int Rotate;


/* Display:
*  -------
*	Runs draw function for flocks, objects, world, goal and curve.
*/
void Display()
{
	// clear the current buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// save the current transformation matrix
	// turn on the lights
	glPushMatrix();
		//rotate the global scene based on the mouse rotations
		// glRotated ((GLdouble) spinxface, 1.0, 0.0, 0.0);
		// glRotated ((GLdouble) spinyface, 0.0, 1.0, 0.0);
		// glRotated ((GLdouble) spinzface, 0.0, 0.0, 1.0);
		// create pointers to the begining and end of the particle list
	
		// DrawAxis(8);

		// target.Draw();
		container.DrawGround();
		// targetCurve->DrawCurve();
		
		std::vector<Object*>::iterator currentObject = objects.begin();
		std::vector<Object*>::iterator endObject = objects.end();

		glBegin( GL_POINTS );
			glVertex3f( 0.0, 0.0, 0.0 );
		glEnd();
	
		// Cycle through all the flocks and call the draw methods for each one
		while(currentObject != endObject)
		{
			(*currentObject)->Draw();
			++currentObject;
		}

		std::vector<Flock*>::iterator currentFlock = flocks.begin();
		std::vector<Flock*>::iterator endFlock = flocks.end();
	
		// Cycle through all the flocks and call the draw methods for each one
		while(currentFlock != endFlock)
		{
			(*currentFlock)->Draw();
			++currentFlock;
		}
	
	glPopMatrix();
	// render the scene
	glutSwapBuffers();
}

/* Update:
*  -------
*	Runs update function for flock and goal.
*/
void Update(int i)
{
	std::vector<Flock*>::iterator currentFlock = flocks.begin();
	std::vector<Flock*>::iterator endFlock = flocks.end();

	if(!Pause) {

		// target.Update();
		
		//Cycle through the flocks calling the appropriate behaviours and finally the Update method.
		while(currentFlock != endFlock)
		{
			Imath::V3f centre( 0.0, 0.0, 0.0 );
			(*currentFlock)->Update( centre );
			// (*currentFlock)->OBJExport(frame);
			++currentFlock;
			
		}
		++frame;
		
	}
	
	glutPostRedisplay();
	glutTimerFunc(40,Update,0);
}

/* CreateWorld:
*  -------
*	Sets world bounds
*/
void CreateWorld(float px, float nx, float py, float ny, float pz, float nz)
{
	container.maxX = px;
	container.minX = nx;
	container.maxY = py;
	container.minY = ny;
	container.maxZ = pz;
	container.minZ = nz;
}

/* CreateObject:
*  -------------
*	Creates objects in the scene.
*/
void CreateObject(float x, float y, float z)
{
	Object* newObject = new Object(container, x, y, z);
	objects.push_back(newObject);
}


/* cleanup:
*  -------
*	Sorts out memory management for the program. Must be called before exiting.
*/
void cleanup() {
	{
		std::vector<Boid*>::iterator itBoid = boids.begin();
		std::vector<Boid*>::iterator endBoid = boids.end();
		for(; itBoid != endBoid; ++itBoid)
		{
			delete (*itBoid);
		}
		boids.clear();
	}
	{
		std::vector<Flock*>::iterator itFlock = flocks.begin();
		std::vector<Flock*>::iterator endFlock = flocks.end();
		
		std::vector<Particle*>::iterator itPart;
		std::vector<Particle*>::iterator endPart;
		
		for(; itFlock != endFlock; ++itFlock)
		{
			itPart = (*itFlock)->particles.begin();
			endPart = (*itFlock)->particles.end();
			
			for(; itPart != endPart; ++itPart)
			{
				delete (*itPart);
			}
				
			delete (*itFlock);
		}
		flocks.clear();
		
		container.flocks.clear();
		container.objects.clear();
	}
}

/* SetTargetPath:
*  -------
*	Set path for goal object to travel along.
*/
void SetTargetPath()
{
	// Point3 targetPath[4];
	// 
	// targetPath[0].set(-40,0,-15);
	// targetPath[1].set(-20,5,0);
	// targetPath[2].set(20,-5,0);
	// targetPath[3].set(40,0,15);
	// 
	// targetCurve = new CurveFollow(targetPath);
	// 
	// target.setCurve(*targetCurve);
}


/* CreateFlock:
*  ------------
*	Creates a flock and populates it with the correct number of boids
*/
void CreateFlock(int flockID, int numBoids, double x, double y, double z, double spread)
{

	Flock* newFlock = new Flock(flockID, container);
	
	newFlock->numMembers = numBoids;
	
	Boid* newBoid = NULL;
	for(int b=0; b < numBoids; ++b)
	{
		newBoid = new Boid(b, flockID, x, y, z, spread);
		boids.push_back(newBoid);
	}

	std::vector<Boid*>::iterator currentBoid = boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();
		
	while(currentBoid != endBoid)
	{
		if(newFlock->flockID == (*currentBoid)->flockID)
		{
			newFlock->AddBoid(*currentBoid);
		}
		
		++currentBoid;
	}
	
	flocks.push_back(newFlock);

}


/* Update:
*  -------
*	Adds all flocks and objects to the STL vectors in the world (container).
*/
void CompleteFlocks()
{

	std::vector<Object*>::iterator currentObject = objects.begin();
	std::vector<Object*>::iterator endObject = objects.end();
	
	while(currentObject != endObject)
	{
		container.AddObject((*currentObject));

		++currentObject;
	}

	FlockIt currentFlock = flocks.begin();
	FlockIt endFlock = flocks.end();

	while(currentFlock != endFlock)
	{
		container.AddFlock((*currentFlock));

		++currentFlock;
	}
}


/* Tokenize modified from 
http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
C++ Programming HOW-TO
Al Dev (Alavoor Vasudevan) alavoor[AT]yahoo.com
v42.9, 17 Sep 2002
*/
void Tokenize(const std::string& str, std::vector< std::string >& tokens, const std::string& delimiters = " ")
{
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos = str.find_first_of(delimiters, lastPos);

	while( std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

/* ParseConfigFile:
*  -------
*	Parses the config file.
*/
bool ParseConfigFile()
{
	std::fstream FileIn;
	std::vector< std::string > tokens;
	
	// open the file and check to see if it was ok
	FileIn.open(Filename.c_str(), std::ios::in);

	if (!FileIn.is_open())
	{
		std::cout <<"File Not Found"<<std::endl;
		return false;
	}

	int flockID = 1;
	Flock *lastFlock = NULL;
	
	// this is the buffer which holds the current line from the file
	std::string LineBuffer;
	// now loop till end of the file and parse
	while(!FileIn.eof())
	{
		// get the line in
		getline(FileIn,LineBuffer,'\n');
		// then clear the tokens list and tokenize them
		tokens.clear();
		

		if(LineBuffer != "")
		{ 
			// now create the token list
			Tokenize(LineBuffer, tokens," \t\n");	
	
			// now check for the entries to parse using ==
			// look for the 
			if(tokens[0] == "BeginFlocks") { continue; }
			else if(tokens[0] == "StartFlock")
			{
				if(atoi(tokens[1].c_str()) == 0)
				{
					std::cout << "Error: Flock of zero size has been specified. Please specify a non-zero size." << std::endl;
					cleanup();
					exit(0);
				}
				CreateFlock(flockID, atoi(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), atof(tokens[4].c_str()), atof(tokens[5].c_str()));
					
				lastFlock = flocks.back();
			}
			else if(tokens[0] == "EndFlock")
			{
				++flockID;
			}
			else if(tokens[0] == "EndFlocks")
			{
				
			}
			else if(tokens[0] == "FlockColour") {
				(*lastFlock).colour.r = atof(tokens[1].c_str());
				(*lastFlock).colour.g = atof(tokens[2].c_str());
				(*lastFlock).colour.b = atof(tokens[3].c_str());
				(*lastFlock).colour.a = atof(tokens[4].c_str());
			}
			
			else if(tokens[0] == "FoodChain") { lastFlock->rank = atoi(tokens[1].c_str()); }
			
			else if(tokens[0] == "BankingDepth") { lastFlock->bDepth = atoi(tokens[1].c_str()); }
			else if(tokens[0] == "BankingScale") { lastFlock->bScale = atoi(tokens[1].c_str()); }

			else if(tokens[0] == "MaxHunt") { lastFlock->maxHunt = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleHunt") { lastFlock->scaleHunt = atof(tokens[1].c_str()); }
			
			else if(tokens[0] == "MaxFlee") { lastFlock->maxFlee = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleFlee") { lastFlock->scaleFlee = atof(tokens[1].c_str()); }
			else if(tokens[0] == "FleeTestRadius") { lastFlock->fleeTR = atof(tokens[1].c_str()); }
			
			else if(tokens[0] == "MaxVelocity") { lastFlock->maxVel = atof(tokens[1].c_str()); }
			else if(tokens[0] == "MinVelocity") { (*lastFlock).minVel = atof(tokens[1].c_str()); }
			else if(tokens[0] == "MaxAcceleration") { (*lastFlock).maxAcc = atof(tokens[1].c_str()); }
	
			else if(tokens[0] == "MaxCollisionAvoidance") { (*lastFlock).maxCA = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleCollisionAvoidance") { (*lastFlock).scaleCA = atof(tokens[1].c_str()); }

			else if(tokens[0] == "MaxVelocityMatching") { (*lastFlock).maxVM = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleVelocityMatching") { (*lastFlock).scaleVM = atof(tokens[1].c_str()); }

			else if(tokens[0] == "MaxGoalCentring") { (*lastFlock).maxGoalFC = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleGoalCentring") { (*lastFlock).scaleGoalFC = atof(tokens[1].c_str()); }

			else if(tokens[0] == "MaxLocalFlockCentring") { (*lastFlock).maxLocalFC = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleLocalFlockCentring") { (*lastFlock).scaleLocalFC = atof(tokens[1].c_str()); }

			else if(tokens[0] == "MaxGlobalFlockCentring") { (*lastFlock).maxGlobalFC = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleGlobalFlockCentring") { (*lastFlock).scaleGlobalFC = atof(tokens[1].c_str()); }

			else if(tokens[0] == "MaxObjectAvoidance") { (*lastFlock).maxOA = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ScaleObjectAvoidance") { (*lastFlock).scaleOA = atof(tokens[1].c_str()); }
	
			else if(tokens[0] == "BoidTestRadius") { (*lastFlock).boidTR = atof(tokens[1].c_str()); }
			else if(tokens[0] == "ObjectTestRadius") { (*lastFlock).objectTR = atof(tokens[1].c_str()); }

			else if(tokens[0] == "StartObject") 
			{ 
				CreateObject(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
			}
			
			else if(tokens[0] == "EndObject") 
			{ 
				
			}

			else if(tokens[0] == "CreateWorld") 
			{ 
				CreateWorld(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), atof(tokens[4].c_str()), atof(tokens[5].c_str()), atof(tokens[6].c_str()));
			}			
			
			
			else if(tokens[0] == "//") { /* ignore comments */ }

			else 
			{ 
				std::cout <<"Unknown token "<<tokens[0]<<std::endl;
			}
		}
	}

	CompleteFlocks();
	
	FileIn.close();
		
	return true;
}


void MouseButton(int button, int down, int x, int y) 
{
	// Rotate = 0;
	// switch(button)
	// {
	// 	case GLUT_LEFT_BUTTON:
	// 		if (down == GLUT_DOWN)
	// 		{
	// 			origx = x;
	// 			origy = y;
	// 			Rotate = 1;
	// 		}
	// 		else Rotate = 0;
	// 	break;
	// }
}

void MouseMotion(int x, int y) 
{
	// if(Rotate) {
	// 	spinyface = ( spinyface + (x - origx) ) % 360 ;
	// 	spinxface = ( spinxface + (y - origy) ) % 360 ;
	// 	origx = x;
	// 	origy = y;
	// 	glutPostRedisplay();
	// }
}

void Keyboard(unsigned char ch, int x, int y) 
{
	std::vector<Flock*>::iterator currentFlock = flocks.begin();
	std::vector<Flock*>::iterator endFlock = flocks.end();

	std::vector<Boid*>::iterator currentBoid= boids.begin();
	std::vector<Boid*>::iterator endBoid = boids.end();

	switch(ch)
	{
		case ESCAPE: 
		case 'q':
		case 'Q':
			cleanup();
			exit(0);
		break;

		case 'f':
		case 'F':
			useflocking ^= true;
		break;

		case 'z':
		case 'Z':
			if(!Pause)
				Pause = 1;
			else
				Pause = 0;
		break;

		case ' ':
			// Cycle through the flocks calling the appropriate behaviours and finally the Update method.
			cleanup();
			
			ParseConfigFile();
			
		break;
		
		// case 'd' : CamMode=MOVESLIDE; break;
		// case 'e' : CamMode=MOVEEYE; break;
		// case 'l' : CamMode=MOVELOOK; break;
		// case 'b' : CamMode=MOVEBOTH; break;
		}
}

void SpecialKey(int key, int x, int y) 
{
	// switch(CamMode)
	// {
	// case MOVESLIDE :
	// 	switch (key)
	// 	{
	// 	// slide the camera left right up and down
	// 	case GLUT_KEY_LEFT	:
	// 		Cam.slide(0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_UP	:
	// 		Cam.slide(0.0,0.1,0.0);
	// 	break;
	// 	case GLUT_KEY_RIGHT	:
	// 		Cam.slide(-0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_DOWN	:
	// 		Cam.slide(0.0,-0.1,0.0);
	// 	break;
	// 	//slide the camera along the z axis
	// 	case GLUT_KEY_PAGE_UP	:
	// 		Cam.slide(0.0,0.0,0.1);
	// 		break;
	// 	case GLUT_KEY_PAGE_DOWN	:
	// 		Cam.slide(0.0,0.0,-0.1);
	// 		break;
	// 	
	// 	}
	// break;
	// 
	// case MOVEEYE :
	// 	switch (key)
	// 	{
	// 	// slide the camera left right up and down
	// 	case GLUT_KEY_LEFT	:
	// 		Cam.moveEye(0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_UP	:
	// 		Cam.moveEye(0.0,0.1,0.0);
	// 	break;
	// 	case GLUT_KEY_RIGHT	:
	// 		Cam.moveEye(-0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_DOWN	:
	// 		Cam.moveEye(0.0,-0.1,0.0);
	// 	break;
	// 	//slide the camera along the z axis
	// 	case GLUT_KEY_PAGE_UP	:
	// 		Cam.moveEye(0.0,0.0,0.1);
	// 		break;
	// 	case GLUT_KEY_PAGE_DOWN	:
	// 		Cam.moveEye(0.0,0.0,-0.1);
	// 		break;
	// 
	// 	}
	// break;
	// 
	// case MOVELOOK :
	// 	switch (key)
	// 	{
	// 	// slide the camera left right up and down
	// 	case GLUT_KEY_LEFT	:
	// 		Cam.moveLook(0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_UP	:
	// 		Cam.moveLook(0.0,0.1,0.0);
	// 	break;
	// 	case GLUT_KEY_RIGHT	:
	// 		Cam.moveLook(-0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_DOWN	:
	// 		Cam.moveLook(0.0,-0.1,0.0);
	// 	break;
	// 	//slide the camera along the z axis
	// 	case GLUT_KEY_PAGE_UP	:
	// 		Cam.moveLook(0.0,0.0,0.1);
	// 		break;
	// 	case GLUT_KEY_PAGE_DOWN	:
	// 		Cam.moveLook(0.0,0.0,-0.1);
	// 		break;
	// 	
	// 	}
	// break;
	// 
	// case MOVEBOTH :
	// 	switch (key)
	// 	{
	// 	// slide the camera left right up and down
	// 	case GLUT_KEY_LEFT	:
	// 		Cam.moveBoth(0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_UP	:
	// 		Cam.moveBoth(0.0,0.1,0.0);
	// 	break;
	// 	case GLUT_KEY_RIGHT	:
	// 		Cam.moveBoth(-0.1,0.0,0.0);
	// 	break;
	// 	case GLUT_KEY_DOWN	:
	// 		Cam.moveBoth(0.0,-0.1,0.0);
	// 	break;
	// 	//slide the camera along the z axis
	// 	case GLUT_KEY_PAGE_UP	:
	// 		Cam.moveBoth(0.0,0.0,0.1);
	// 		break;
	// 	case GLUT_KEY_PAGE_DOWN	:
	// 		Cam.moveBoth(0.0,0.0,-0.1);
	// 		break;
	// 	
	// 	}
	// break;
	// 
	// 
	// }
}


void InitialiseGL()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);			   // Grey Background
	// render things smoothly
	glShadeModel(GL_SMOOTH);
	// enable depth testing for drawing
	glEnable(GL_DEPTH_TEST);
	// enable the default light
	glEnable(GL_LIGHT0);
	// set the ambient colour to dark grey
	GLfloat AmbColour[]={0.2,0.2,0.2};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,AmbColour);
	// setup the 
	float val = 30.0f;
	Imath::V3f eye(val, val, val);
	Imath::V3f look(0.0f,0.0f,0.0f);
	Imath::V3f up(0.0f,1.0f,0.0f); //Y == UP
	glMatrixMode(GL_MODELVIEW);
	gluPerspective( 45, 640.0/480.0, 0.5, 150 ); 
	gluLookAt( eye, look, up );
	// Cam.set(Eye,Look,Up);
	// Cam.setShape(45,640.0/480.0,0.5,150,PERSPECTIVE);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
}

// application main loop
int main(int argc, char **argv)
{
	if(argc < 2)
	{
		std::cout <<"usage " << argv[0] << " [config file]"<<std::endl;
		exit(1);
	}

	// Create default world.
	CreateWorld(60, -60, 30, -30, 30, -30);
	
	SetTargetPath();
	
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA |GLUT_ACCUM);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("Flocking System - Version 1 - Michael Jones");

	glutDisplayFunc(Display);

	glutTimerFunc(40,Update,0);

	glutMouseFunc(MouseButton);
	glutMotionFunc(MouseMotion);

	glutKeyboardFunc(Keyboard);
	glutSpecialFunc(SpecialKey);

	InitialiseGL();

	Filename = argv[1];
	ParseConfigFile();	

	glutMainLoop();
	
	return 0;
}
