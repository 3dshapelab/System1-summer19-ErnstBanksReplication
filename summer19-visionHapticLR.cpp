// this is the script that blocked the trials by feedback: Visual only, haptic only or visual + haptic
// spring19-jovan-pictureGraspStereopsis.cpp : Defines the entry point for the console application.
//
// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once


#include <stdio.h>
#include <tchar.h>


// TODO: reference additional headers your program requires here
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <queue>



/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "CylinderPointsStimulus.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "TrialGenerator.h"
#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"
#include "BrownPhidgets.h"
/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;


#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"
#include "BrownMotorFunctions.h"
using namespace BrownMotorFunctions;
using namespace BrownPhidgets;

/***** CALIBRATION FILE *****/
#include "LatestCalibration.h"
/********* #DEFINE DIRECTIVES **************************/
#define TIMER_MS 11 // 85 hz
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
//static const double SCREEN_WIDE_SIZE = 386.;    // millimeters
const float DEG2RAD = M_PI/180;
/*************** Variable Declarations ********************/

/********* 18 October 2011   CALIBRATION ON CHIN REST *****/
//const Vector3d calibration(160,179,-75);
//static const Vector3d objCalibration(199.1, -149.2, -319.6);
// Alignment between optotrak z axis and screen z axis
//double alignmentX = 33.5;
//double alignmentY = 33;// - 49 - 55.0/2.0;
//double focalDistance= -270.0, homeFocalDistance=-270.0;
static const Vector3d center(0,0,focalDistance);
double mirrorAlignment=0.0, screenAlignmentY=0.0, screenAlignmentZ=0.0;
Screen screen;

//static const Vector3d centercal(29.75,-133.94,-296.16); //updated 9/25/14


/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode=true;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
Optotrak2 *optotrak;
CoordinatesExtractor headEyeCoords, thumbCoords, indexCoords, thumbJointCoords, indexJointCoords;
/********** STREAMS **************/
ofstream responseFile;
/********** EYES AND MARKERS **********************/

Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
static double interoculardistance=60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;
// finger markers
Vector3d ind, indJoint, thm, thmJoint;
Vector3d indexCalibrationPoint(0, 0, 0), thumbCalibrationPoint(0, 0, 0);
int ind1 = 13, ind2 = 14, ind3 = 16;
int thu1 = 15, thu2 = 17, thu3 = 18;
int calibration1 = 1, calibration2 = 2;
double markerXOffset = 10;
double markerYOffset = 10;
bool markers_status = false;
bool allVisibleIndex = markers_status;
bool allVisibleThumb = markers_status;
bool allVisibleFingers = markers_status;
/********** VISUALIZATION AND STIMULI ***************/
Timer presentation_timer;
Timer globalTimer;
Timer trial_time;
bool visibleInfo = true;

int standard_objId = 0;
int comparison_objId = 1;
int centercalMarker = 12;
int obj0 = 12, obj1 = 9, obj2 = 8, obj3 = 7, obj4 = 4, obj5 = 23, obj6 = 3, obj7 = 24, obj8 = 11;
int objMarkers[9] = { obj0, obj1, obj2, obj3, obj4, obj5, obj6, obj7, obj8};

int objId = 12;
double target_angle = 0.0;
double objSizes[9] = { 55.0, 56.0, 57.0, 59.0, 51.0, 54.0, 48.0, 53.0, 62.0 };
double x_shft = -5.5;
double y_shift[9] = { -5.2, -2.8, -4.1, -5.9, -6.8, -5, -5.8, -6.8, -4.2 };
double z_shft = -15;
double objStepperPosns[9] = { 0, 40, 80, 120, 160, -160, -120, -80, -40 };

Vector3d moveTo(50, 0, -500);
Vector3d centercal(19.7, -298.83, -273.6);
CPhidgetStepperHandle rotTable;
double stimulus_width;
// height the height of the standard visual stimulus (random dot stereogram)
double standard_height = 55;
// height the height of the comparison visual stimulus (random dot stereogram)
double comparison_height = 55;
// height the height of the standard haptic stimulus
double haptic_height_s = 0;
// height the height of the comparison haptic stimulus
double haptic_height_c = 0;

double dot_visangle = .133333;

int trial_noise;
double percentComplete;
/********** Trial Control ***************************/
ParametersLoader parameters;
ParametersLoader parameters_V;
ParametersLoader parameters_H;
ParametersLoader parameters_VH;

//controls the factors 
BalanceFactor<double> trial;
bool training = true;
bool screenBack = false; //has the screen been placed at projection distance
bool finished = false;
int repetitions = 2;
int trialNumBlk = 0;
double ElapsedTime;
double presentationTime;
int timer_started;
double time_checkpoint;
int blkNum = 0;
int blkOrder[2] = {0, 2}; // r8 {0,2} r9 {2, 1} r10 {2,0} r11 {1, 2} r12 {0,2}...
int totalBlks = 2;
int trialNumber = 1;
int totaltrialNum = 100;
double display_distance;
int image_number = 0;
const double inter_stimulus = 1000;
const double pres_time = 1000;
const double trial_start = 1000;
std::vector<Vector3d> standard_container;
std::vector<Vector3d> comparison_container;
std::vector<Vector3d> container[2];
int hapticIDs[2] = {0, 1};

bool randomizeBlock = false;
bool rotate_one = false;
bool rotate_two = false;
bool skipTrial = false;


// task 0 is vision only, task 1 is haptic only, 
// task 2 is vision and haptic combine in congruent way
// task 3 is vision and haptic combine in incongruent way with vision 54 and haptic 56
// task 4 is vision and haptic combine in incongruent way with vision 53 and haptic 57
// task 5 is vision and haptic combine in incongruent way with vision 57 and haptic 53
// task 6 is vision and haptic combine in incongruent way with vision 56 and haptic 54
// task 99 is the training
int task = 0;
int dummy_task = 0;

int fingerCalibrationDone = 0;
bool fingersCalibrated = false;
int lastTrialInBlk = 0;
//First grasp initializes both visual only and the haptic conditions
enum graspStages {first_grasp, first_release, second_grasp, second_release ,
first_visual,second_visual, make_responce, showProgBar, breakTime };
graspStages currentStage = first_grasp;

/***************** DISTANCE VARIABLES******************/
double lastTargetOriginX = 42, lastTargetOriginY = 0.0, lastTargetOriginZ = -400;
double targetOriginX = 42, targetOriginY = 0.0, targetOriginZ = -400; // holders for the current target
double targetMarkerX, targetMarkerY, targetMarkerZ;
// The starting position
double start_dist;

// Position variables for keeping track of finger movement
double grip_Origin_X;
double grip_Origin_Y;
double grip_Origin_Z;
double grip_aperture = 0;
double old_grip_aperture;
double oldDistanceGripCenterToObject;
double distanceGripCenterToObject = 0;
double vel_gripCntrToObj;
double oldGrip_aperture;
double vel_gripAp;
double x_dist = 999;
double y_dist = 999;
double z_dist = 999;

bool handOnObject = false;
bool handSafeDistance = false;
bool handTouchingObject = false;
int fingersOccluded = 0;

// response vars
bool stdFirst = true;
bool firstTaller = true;
bool respond_stdTaller = true;

/*************************** EXPERIMENT SPECS ****************************/
string subjectName;

// experiment directory

string experiment_directory = "C:/Users/visionlab/Documents/data/jovan/summer19-visionHapticLR";

// paramters file directory and name
string parametersFile_directory = experiment_directory + "/parameters_summer19-visionHapticLR.txt";
string parametersFile_directory_V = experiment_directory + "/parameters_summer19-visionHapticLR_V.txt";
string parametersFile_directory_H = experiment_directory + "/parameters_summer19-visionHapticLR_H.txt";
string parametersFile_directory_VH = experiment_directory + "/parameters_summer19-visionHapticLR_VH.txt";

// response file headers
string responseFile_headers = "subjName\tIOD\tblockN\ttrialN\ttask\tNoiseLevel\tstdVisual\tstdHaptic\ttestVisual\ttestHaptic\tStdIsFirst\tfirstIsTaller\trespondStdTaller\ttraining";

/*************************** FUNCTIONS ***********************************/
void initOptotrak();
void initMotors();
void initRendering();
std::vector<Vector3d> buildRandomDots(double stimulus_height);
void initVariables();
void initStreams();
void handleResize(int w, int h);
void drawStimulus();
void initBlock();
void initTrial();
void advanceTrial();
void updateTheMarkers();
void drawInfo();
void beepOk(int tone);
void cleanup();
void initProjectionScreen(double _focalDist, const Affine3d &_transformation=Affine3d::Identity(),bool synchronous=true);
void online_apparatus_alignment();
void update(int value);
void calibration_fingers(int phase);
void online_fingers();
//void draw_thumb();
void draw_fingers();
void draw_gripCenter();
void testValidTrial();
void nextTilValidTrial();
void writeResponse();
void drawPanels();
void drawProgressBar();
bool isPointWithinAnySphere(Eigen::Vector3d vert, double radius);
/*******************************************************************************
END OF DEFINITIONS AND INITIALIZATIONS


CODE MAIN BODY
********************************************************************************/

void shutdown(){
	responseFile.close();
	stepper_rotate(rotTable, 0, 238.67);
	stepper_close(rotTable);
	homeEverything(5000,4500);
	cleanup();
	exit(0);
}
void cleanup()
{
	// Stop the optotrak
	optotrak->stopCollection();
	delete optotrak;
}
void initProjectionScreen(double _focalDist, const Affine3d &_transformation, bool synchronous)
{
	focalDistance = _focalDist;	
	screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE*SCREEN_HEIGHT/SCREEN_WIDTH);
	screen.setOffset(alignmentX,alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if ( synchronous )
		moveScreenAbsolute(_focalDist,homeFocalDistance,4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist,homeFocalDistance,4500);
}
void initOptotrak()
{
	optotrak=new Optotrak2();
	optotrak->setTranslation(calibration);
	int numMarkers=22;
	float frameRate=85.0f;
	float markerFreq=4600.0f;
	float dutyCycle=0.4f;
	float voltage = 7.0f;

	if ( optotrak->init("C:/cncsvisiondata/camerafiles/Aligned20111014",numMarkers, frameRate, markerFreq, dutyCycle,voltage) != 0)
	{   cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
	cin.ignore(1E6,'\n');
	exit(0);
	}

	// Read 10 frames of coordinates and fill the markers vector
	for (int i=0; i<10; i++)
	{
		updateTheMarkers();
	}
}
void updateTheMarkers()
{

	optotrak->updateMarkers();
	markers = optotrak->getAllMarkers();

}
void initMotors()
{
	//specify the speed for (objects,screen)
	homeEverything(5000,4500);
	rotTable = stepper_connect();
}
void initRendering()
{   glClearColor(0.0,0.0,0.0,1.0);
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
/* Set depth buffer clear value */
glClearDepth(1.0);
/* Enable depth test */
glEnable(GL_DEPTH_TEST);
/* Set depth function */
glDepthFunc(GL_LEQUAL);
// scommenta solo se vuoi attivare lo shading degli stimoli

glMatrixMode(GL_MODELVIEW);
glLoadIdentity();
// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
// ballerine (le vedi vibrare)
glLineWidth(1.5);
}

//Extracts constants from parameters file
void initVariables()
{
if(randomizeBlock){
	for (int i = totalBlks - 1; i > 0; i--) {
		int j = rand() % (i + 1);
		int tempy = blkOrder[i];
		blkOrder[i] = blkOrder[j];
		blkOrder[j] = tempy;
	}
}
	trial.init(parameters);	
	trial.next();
    //nextTilValidTrial();
	//initBlock();
	
	interoculardistance = atof(parameters.find("IOD").c_str());
	display_distance = str2num<double>(parameters.find("display_distance"));
	// eye coordinates
	eyeRight = Vector3d(interoculardistance/2,0,0);
	eyeLeft = Vector3d(-interoculardistance/2,0,0);
	eyeMiddle = Vector3d(0,0,0);

	// Subject name
	subjectName = parameters.find("SubjectName");


}
// Inizializza gli stream, apre il file per poi scriverci
void initStreams()
{
	// Initializza il file parametri partendo dal file parameters.txt, se il file non esiste te lo dice
	ifstream parametersFile;
	parametersFile.open(parametersFile_directory.c_str());
	parameters.loadParameterFile(parametersFile);

	ifstream parametersFile_V;
	parametersFile_V.open(parametersFile_directory_V.c_str());
	parameters_V.loadParameterFile(parametersFile_V);

	ifstream parametersFile_H;
	parametersFile_H.open(parametersFile_directory_H.c_str());
	parameters_H.loadParameterFile(parametersFile_H);

	ifstream parametersFile_VH;
	parametersFile_VH.open(parametersFile_directory_VH.c_str());
	parameters_VH.loadParameterFile(parametersFile_VH);

    //tempBool = str2num<double>(parameters.find("completeBlockBool"));
	initVariables();
	// Subject name
	//string subjectName = parameters.find("SubjectName");
	if (util::fileExists(experiment_directory +"/"+subjectName+".txt") && subjectName != "junk")
	{
		string error_on_file_io = experiment_directory+"/"+subjectName+".txt" + string(" already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.",NULL, NULL);
		shutdown();
	}
	string responseFileName = experiment_directory +"/"+ subjectName + ".txt";
	// Principal streams files
	if (util::fileExists(experiment_directory +"/"+subjectName+".txt") && subjectName != "junk")
	{
		string error_on_file_io = experiment_directory+"/"+subjectName+".txt" + string(" already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.",NULL, NULL);
		shutdown();
	}
	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;
	

	globalTimer.start();
}
//Central function for projecting image onto the screen

void calibration_fingers(int phase)
{
	switch (phase)
	{
	case 1:
		{
			if (isVisible(markers[calibration1].p) && isVisible(markers[calibration2].p))
			{
				indexCalibrationPoint = markers.at(calibration1).p;
				indexCalibrationPoint[0] = indexCalibrationPoint[0] - markerXOffset;
				indexCalibrationPoint[1] = indexCalibrationPoint[1] + markerYOffset;
				thumbCalibrationPoint = markers.at(calibration2).p;
				thumbCalibrationPoint[0] = thumbCalibrationPoint[0] - markerXOffset;
				thumbCalibrationPoint[1] = thumbCalibrationPoint[1] - markerYOffset;
			}
		} break;
	case 2:
		{
			indexCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
			thumbCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);
		} break;
	case 3:
		{
			indexJointCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
			thumbJointCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);
		} break;

	}
}

void drawProgressBar(){
		glPushMatrix();
	glLoadIdentity();
	//glTranslated(targetOriginX, targetOriginY, targetOriginZ);
	glTranslated(targetOriginX,targetOriginY,targetOriginZ);
			glColor3f(0.7,0.7,0.7);
			glBegin(GL_LINE_LOOP);
			glVertex3f(-50,65,5);
			glVertex3f( 50,65,5);
			glVertex3f(50,55,5);
			glVertex3f(-50,55,5);
			glEnd();

			glColor3f(0.1,0.5,0.1);
			glBegin(GL_POLYGON);
			glVertex3f(-50,65,4);
			glVertex3f(-50+percentComplete,65,4);
			glVertex3f(-50+percentComplete,55,4);
			glVertex3f(-50,55,4);
			glEnd();
glPopMatrix();
	glLineWidth(1.0);

}

// Funzione che gestisce il ridimensionamento della finestra
void drawGLScene()
{

	// Draw left eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0,0.0,0.0,1.0);
	cam.setEye(eyeLeft);
	drawStimulus();
	drawInfo();

	// Draw right eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0,0.0,0.0,1.0);
	cam.setEye(eyeRight);
	drawStimulus();
	drawInfo();

	glutSwapBuffers();
}
void update(int value)
{
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}

void drawInfo(){

	if ( finished ){
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);

		GLText text;

		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();
		glColor3fv(glWhite);
		text.draw("The experiment is over. Thank you! :)");
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);

	}else if (currentStage == breakTime){

		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);

		GLText text;

		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();
		glColor3fv(glWhite);
		text.draw("break time. Press A to continue...");
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);

	}else if(!fingersCalibrated){
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);

		GLText text;

		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();
		switch (fingerCalibrationDone) {

		case 0:
			text.draw("Press F to record calibration point positions.");
			break;
		case 1:
			text.draw("Press F with index and thumb TIPS on calibration points.");
			break;
		case 2:
			text.draw("Press F with index and thumb JOINTS on calibration points.");
			break;
		case 3:
			text.draw("Press F to set apparatus in place.");
			break;
		case 4:
			text.draw("Press F to begin!");
			break;
		}
		// the following is drawn depending on which calibration stage it is
		if (fingerCalibrationDone <= 2) {

			if (isVisible(markers[calibration1].p) && isVisible(markers[calibration1].p) && isVisible(markers[calibration1].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Index Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration1].p.transpose()));

			if (isVisible(markers[calibration2].p) && isVisible(markers[calibration2].p) && isVisible(markers[calibration2].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Thumb Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration2].p.transpose()));


			// INDEX FINGER ///////
			glColor3fv(glWhite);
			text.draw(" ");
			text.draw("Index");
			if (isVisible(markers[13].p) && isVisible(markers[14].p) && isVisible(markers[16].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Marker " + stringify<int>(13) + stringify< Eigen::Matrix<double, 1, 3> >(markers[13].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(14) + stringify< Eigen::Matrix<double, 1, 3> >(markers[14].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(16) + stringify< Eigen::Matrix<double, 1, 3> >(markers[16].p.transpose()) + " [mm]");

			// THUMB //////
			glColor3fv(glWhite);
			text.draw(" ");
			text.draw("Thumb");
			if (isVisible(markers[15].p) && isVisible(markers[17].p) && isVisible(markers[18].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Marker " + stringify<int>(15) + stringify< Eigen::Matrix<double, 1, 3> >(markers[15].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(17) + stringify< Eigen::Matrix<double, 1, 3> >(markers[17].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(18) + stringify< Eigen::Matrix<double, 1, 3> >(markers[18].p.transpose()) + " [mm]");
			if ( abs(mirrorAlignment - 45.0) < 0.2 )
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("# Mirror Alignment = " +stringify<double>(mirrorAlignment));

			// check if monitor is calibrated
			if ( screenAlignmentY < 89.0 )
				glColor3fv(glRed);
			else
				glColor3fv(glGreen);
			text.draw("# Screen Alignment Y = " +stringify<double>(screenAlignmentY));
			if ( abs(screenAlignmentZ) < 89.0 )
				glColor3fv(glRed);
			else
				glColor3fv(glGreen);
			text.draw("# Screen Alignment Z = " +stringify<double>(screenAlignmentZ));
			glColor3fv(glWhite);
			// X and Z coords of simulated fixation
			text.draw("# Fixation Z = " +stringify<double>(markers[19].p.x()-120.0)+ " [mm]");
			text.draw("# Fixation X = " +stringify<double>(markers[19].p.z()+363.0)+ " [mm]");
			text.draw(" ");

		}else{
			glColor3fv(glWhite);
			text.draw("--------------------");
			if (allVisibleIndex)
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Index= " + stringify< Eigen::Matrix<double, 1, 3> >(ind.transpose()));
			if (allVisibleThumb)
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Thumb= " + stringify< Eigen::Matrix<double, 1, 3> >(thm.transpose()));
			glColor3fv(glWhite);
			text.draw("--------------------");
			text.leaveTextInputMode();
		}

	}else if(visibleInfo && fingersCalibrated){		

		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);

		GLText text;

		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);
		text.enterTextInputMode();
		//text.draw("# Name: " + parameters.find("SubjectName"));		
		//text.draw("# IOD: " +stringify<double>(interoculardistance));
        text.draw("# Distance to Ojbect: " + stringify<double>(distanceGripCenterToObject));
		text.draw("# Center Markers: "+ stringify< Eigen::Matrix<double, 1, 3> >(markers[12].p.transpose()) + " [mm]");		
		text.draw("# trial: " +stringify<int>(trialNumber));
		text.draw("# time: " +stringify<int>(ElapsedTime));
		//text.draw("# stage:" + stringify<int>(currentStage));
		//text.draw("# task:" + stringify<int>(task));
		//text.draw("# noise level: " + stringify<double>(trial_noise));
		/*
		text.draw("--------------------");
        text.draw("# target X: " + stringify<double>(targetOriginX));
		text.draw("# target Y: " + stringify<double>(targetOriginY));
		text.draw("# target Z: " + stringify<double>(targetOriginZ));
		text.draw("--------------------");
        text.draw("# x_dist: " + stringify<double>(x_dist));
		text.draw("# y_dist: " + stringify<double>(y_dist));
		text.draw("# z_dist: " + stringify<double>(z_dist));
		text.draw("--------------------");
        text.draw("# grip X: " + stringify<double>(grip_Origin_X));
		text.draw("# grip Y: " + stringify<double>(grip_Origin_Y));
		text.draw("# grip Z: " + stringify<double>(grip_Origin_Z));		//text.draw("# visual height (standard): " + stringify<double>(standard_height));
		*/
		//text.draw("# haptic height (standard): " + stringify<double>(haptic_height_s));
		//text.draw("# visual height (comparison): " + stringify<double>(comparison_height));
		//text.draw("# haptic height (comparison): " + stringify<double>(haptic_height_c));


		text.leaveTextInputMode();
	}
	
}



void online_fingers()
{
	// Visibility check
	allVisibleIndex = isVisible(markers.at(ind1).p) && isVisible(markers.at(ind2).p) && isVisible(markers.at(ind3).p);
	allVisibleThumb = isVisible(markers.at(thu1).p) && isVisible(markers.at(thu2).p) && isVisible(markers.at(thu3).p);
	allVisibleFingers = allVisibleIndex && allVisibleThumb;

	// fingers coordinates, fingersOccluded and framesOccluded
	if (allVisibleFingers)
	{
		indexCoords.update(markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
		thumbCoords.update(markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);
		indexJointCoords.update(markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
		thumbJointCoords.update(markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);

	}

	if (fingerCalibrationDone >= 2)
	{
		// index coordinates
		if (allVisibleIndex) {
			ind = indexCoords.getP1();
			if (fingerCalibrationDone >= 3)
				indJoint = indexJointCoords.getP1();
		}
		// thumb coordinates
		if (allVisibleThumb) {
			thm = thumbCoords.getP1();
			if (fingerCalibrationDone >= 3)
				thmJoint = thumbJointCoords.getP1();
		}
	}

	if (fingersCalibrated )
	{
		if (!allVisibleFingers)
		{
			fingersOccluded = 1;
		}
		else {
			fingersOccluded = 0;
		}

		double indXNow = ind.x();
		double indYNow = ind.y();
		double indZNow = ind.z();

		double thmXNow = thm.x();
		double thmYNow = thm.y();
		double thmZNow = thm.z();

		grip_Origin_X = (indXNow + thmXNow) / 2;
		grip_Origin_Y = (indYNow + thmYNow) / 2;
		grip_Origin_Z = (indZNow + thmZNow) / 2;
		oldGrip_aperture = grip_aperture;
		// compute grip aperture
		grip_aperture = sqrt(
			(indXNow - thmXNow) * (indXNow - thmXNow) +
			(indYNow - thmYNow) * (indYNow - thmYNow) +
			(indZNow - thmZNow) * (indZNow - thmZNow)
			);

		vel_gripAp = abs(oldGrip_aperture - grip_aperture);


		// find distance from grip center to object center
		x_dist = abs(grip_Origin_X - targetOriginX);
		y_dist = abs(grip_Origin_Y - targetOriginY);
		z_dist = abs(grip_Origin_Z - targetOriginZ);
		oldDistanceGripCenterToObject = distanceGripCenterToObject;
		distanceGripCenterToObject = sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2));
		//if (PositionSet) {
		vel_gripCntrToObj = abs(oldDistanceGripCenterToObject - distanceGripCenterToObject);
		//}


	}
}

void draw_gripCenter(){
	// Index Segment
	glPushMatrix();
	glLoadIdentity();
	//glBegin(GL_LINE_STRIP);
	glTranslated(grip_Origin_X, grip_Origin_Y, grip_Origin_Z);
	glutSolidSphere(.8, 10, 10);
	//glLineWidth(3.0);
	//glVertex3f(ind.x(), ind.y() + offsetY, ind.z());
	//glVertex3f(indJoint.x(), indJoint.y() + offsetY, indJoint.z());
	//glEnd();
	glPopMatrix();
}
// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0,0,SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

}

bool isPointWithinAnySphere(Eigen::Vector3d vert, double radius, std::vector<Vector3d> sphere_storage, int sphere_count){

	// go through all spheres
	for (int zz=0; zz<sphere_count; zz++){
		Vector3d sphere = sphere_storage.at(zz); // retrieve the centroid of this sphere
		// compute distance from point to sphere
		double dist = sqrt(pow(vert[0]-sphere[0],2)+pow(vert[1]-sphere[1],2)+pow(vert[2]-sphere[2],2));

		if(dist<=radius)
			return true; // return true as soon as you find one containing sphere
	}

	return false; // if we went through all spheres without a match, return false.
}


std::vector<Vector3d> buildRandomDots(double stimulus_height)
{
	std::vector<Vector3d> dot_container;
	int background_width = 200;
	int background_height = 150;
	int diagonal = sqrt(pow(background_width,2.) + pow(background_height,2.));
	//double visual_angle = tan(DEG2RAD*visual_angle/2) * (abs(display_distance));
	double visual_angle_width = 2 * atan(background_width/2/(abs(display_distance - 30))) / DEG2RAD;
	double visual_angle_height = 2 * atan(background_height/2/(abs(display_distance - 30))) / DEG2RAD;
	int dot_num = visual_angle_width * visual_angle_height * 9;
	//stimulus_height = 55;
	for (int dots_placed = 0; dots_placed < dot_num; dots_placed++)
	{
		double x_axis = rand() % background_width - background_width/2;
		double y_axis = rand() % background_height - background_height/2;
		double cur_noise = 0;
		if (trial_noise != 0)
			cur_noise = rand() % trial_noise / 100.;
		double noise_sign = floor(rand() / double(RAND_MAX) - .5) * 2 + 1;
		double z_axis = (30 * cur_noise) * noise_sign / 2;
		
		if (abs(y_axis) <= stimulus_height/2.){
			//cout << 30 + (30 * cur_noise) * noise_sign << endl;
			z_axis = 30 + z_axis;
		} 
		double dot_size = tan(DEG2RAD * dot_visangle/2) * abs(display_distance);
		if (!isPointWithinAnySphere(Vector3d(x_axis,y_axis,z_axis), dot_size, dot_container, dots_placed)){
			
			dot_container.push_back(Vector3d(x_axis,y_axis,z_axis));
		} else {
			dots_placed--;
		}
		/*
		double x_visual_angle =  atan(x_axis/2/(abs(display_distance + z_axis))) / DEG2RAD;
		double y_visual_angle =  atan(y_axis/2/(abs(display_distance + z_axis))) / DEG2RAD;
		if(x_visual_angle <= visual_angle_width/2 && y_visual_angle <= visual_angle_height/2){
			dot_container.push_back(Vector3d(x_axis,y_axis,z_axis));
		} else {
			dots_placed--;
		}*/
	}
	cout << dot_container.size() << endl;

	return dot_container;
}


void drawPanels(){
	int background_width = 150;
	double visual_angle_width = 2 * atan(background_width/2/(abs(display_distance - 30))) / DEG2RAD;
	double new_width = tan(visual_angle_width * DEG2RAD / 2) * abs(targetOriginZ + 50.0) * 2;

	//glDisable(GL_TEXTURE_2D);
	glLoadIdentity();
	glTranslated(targetOriginX,targetOriginY,targetOriginZ + 50.0);
	//glTranslated(0.0, 0, display_distance + 30.0);
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_QUADS); 
	glVertex3f(-new_width/2 - 50.0, 100.0, 0.0f);
	glVertex3f(-new_width/2 , 100.0, 0.0f);
	glVertex3f(-new_width/2 , -100.0, 0.0f);
	glVertex3f(-new_width/2 - 50.0, -100.0, 0.0f);
	glEnd();

	glBegin(GL_QUADS); 
	glVertex3f(new_width/2 , 100.0, 0.0f);
	glVertex3f(new_width/2 + 50.0, 100.0, 0.0f);
	glVertex3f(new_width/2  + 50.0, -100.0, 0.0f);
	glVertex3f(new_width/2, -100.0, 0.0f);
	glEnd();

}


void drawRandomDots(std::vector<Vector3d> dot_container)
{

	
	glLoadIdentity();	
	//glTranslated(targetOriginX,targetOriginY,0);//targetOriginZ - 30);
	glTranslated(targetOriginX,targetOriginY,targetOriginZ - 50);//;
	//glRotated(5,0.,1.,0.);


	for (int i = 0; i < int(dot_container.size()); i++)
	{ 
		Vector3d dot_vector = dot_container.at(i);
		double x_axis = dot_vector[0];
		double y_axis = dot_vector[1];
		double z_axis = dot_vector[2];

		glColor3f(1.0f,0.0f,0.0f);
		glPushMatrix();
		glTranslated(x_axis, y_axis, z_axis);
		//double dot_size = tan(DEG2RAD * dot_visangle/2) * abs(z_axis) * 2;
		double dot_size = tan(DEG2RAD * dot_visangle/2) * abs(display_distance);
		glutSolidSphere(dot_size,5,5);

		glPopMatrix();
	}


}


void drawFingers(double offsetY)
{
	// Index Segment
	if(allVisibleFingers){
		glColor3f(1.0f,0.0f,0.0f);
	glPushMatrix();
	glLoadIdentity();
	//glBegin(GL_LINE_STRIP);
	glTranslated((ind.x() + indJoint.x())/2, (ind.y() + ind.y())/2 + offsetY, (ind.z() + indJoint.z())/2);
	glutSolidSphere(.8, 10, 10);
	//glLineWidth(3.0);
	//glVertex3f(ind.x(), ind.y() + offsetY, ind.z());
	//glVertex3f(indJoint.x(), indJoint.y() + offsetY, indJoint.z());
	//glEnd();
	glPopMatrix();

	// Thumb Segment
	glPushMatrix();
	glLoadIdentity();
	//glBegin(GL_LINE_STRIP);
	glTranslated((thm.x() + thmJoint.x())/2, (thm.y() + thm.y())/2 + offsetY, (thm.z() + thmJoint.z())/2);
	glutSolidSphere(.8, 10, 10);
	//glLineWidth(3.0);
	//glVertex3f(thm.x(), thm.y() - offsetY, thm.z());
	//glVertex3f(thmJoint.x(), thmJoint.y() - offsetY, thmJoint.z());
	//glEnd();
	glPopMatrix();
	} else {
		glColor3f(0.0f,0.8f,0.0f);
			glPushMatrix();
	glLoadIdentity();
	//glBegin(GL_LINE_STRIP);
	glTranslated(targetOriginX, targetOriginY, targetOriginZ);
	glutSolidSphere(.8, 10, 10);
	//glLineWidth(3.0);
	//glVertex3f(ind.x(), ind.y() + offsetY, ind.z());
	//glVertex3f(indJoint.x(), indJoint.y() + offsetY, indJoint.z());
	//glEnd();
	glPopMatrix();
	}

}

void drawCross() {
	glPushMatrix();
	glLoadIdentity();
	//glTranslated(targetOriginX, targetOriginY, targetOriginZ);
	glTranslated(targetOriginX,targetOriginY,targetOriginZ);
	glLineWidth(3.0);
	//slash 1
	glBegin(GL_LINE_LOOP);
	glVertex3d(-10, 0, 0);
	glVertex3d(10, 0, 0);
	glEnd();
	//slash 2
	glBegin(GL_LINE_LOOP);
	glVertex3d(0, -10, 0);
	glVertex3d(0, 10, 0);
	glColor3f(1.0f, 0.0f, 0.0f); //red
	glEnd();
	glPopMatrix();
	glLineWidth(1.0);
}

void presentHapticTarget(int objectId)
{
	cout << objMarkers[objectId] << endl;
	cout << objSizes[objectId] << endl;

	double random_rot = rand() % 120 - 60 + target_angle;
	target_angle = objStepperPosns[objectId];
	stepper_rotate(rotTable, random_rot, 238.67);
	stepper_rotate(rotTable, target_angle, 238.67);

	/*
	if(isVisible(markers[objMarkers[objectId]].p)){
	targetMarkerX = markers[objMarkers[objectId]].p.x();
	targetMarkerY = markers[objMarkers[objectId]].p.y();
	targetMarkerZ = markers[objMarkers[objectId]].p.z();			
	//adjust the current position
	targetOriginX = targetMarkerX + x_shft;
	targetOriginY = targetMarkerY + y_shift[objectId];
	targetOriginZ = targetMarkerZ + z_shft;
	}*/

}

void drawStimulus()
{   /*
			drawRandomDots(container[0]);
			drawPanels();*/

	if(task == 0){
		//Time control for the experiment
		if (ElapsedTime >= trial_start && ElapsedTime  <= pres_time + trial_start){
			if (currentStage == first_grasp) {
				beepOk(4);
				currentStage = first_visual;
			}
			drawRandomDots(container[0]);
			drawPanels();			
			
		} else if (ElapsedTime >= trial_start + pres_time + inter_stimulus && ElapsedTime <= trial_start + pres_time * 2 + inter_stimulus){
			if (currentStage == first_visual) {
				beepOk(4);
				currentStage = second_visual;
			}
			drawRandomDots(container[1]);
			drawPanels();
			
			
		} else if(currentStage == second_visual) {
			currentStage = make_responce;
		} else if(currentStage == showProgBar){
			if(timer_started == 3 && presentationTime <= pres_time){
				drawProgressBar();
			}else{
				advanceTrial();
			}
		}
	}else{

		//intertrial before first presentation
		if (ElapsedTime < trial_start)
		{
			//ready first presentation of haptic
			if (rotate_one && handSafeDistance){
				presentHapticTarget(hapticIDs[0]);
				rotate_one = false;
				//command particpant to grasp object
			} else if (currentStage == first_grasp && ElapsedTime >= trial_start * .95) {
				beepOk(4);
				currentStage = first_release;
			}
			drawFingers(0);
			drawCross();

		} else if (timer_started == 0 && currentStage == first_release){
			//after trial start, before object presentation, show fixation cross and fingers
			if (!handOnObject) {
				drawFingers(0);
				if(stdFirst){
					targetOriginY = y_shift[standard_objId];
				}else{
					targetOriginY = y_shift[comparison_objId];
				}
				drawCross();
			} else {
				presentation_timer.start();
				timer_started = 1;
			}
			//draw first object
		} else if (presentationTime <= pres_time && timer_started == 1 && handOnObject){
			if(task >= 2){
				drawRandomDots(container[0]);
				drawPanels();
				
				}

			time_checkpoint = ElapsedTime;
			//onset of second presentation
		} else if ((timer_started == 1) && (presentationTime >= pres_time && ElapsedTime <= time_checkpoint + inter_stimulus)){
			//reset timer
			presentation_timer.stop();
			//command release of object
			if (currentStage == first_release){
				beepOk(5);
				currentStage = second_grasp;
			}
			
		}else if (currentStage == second_grasp && ElapsedTime >= time_checkpoint + inter_stimulus * .95 && handSafeDistance) {
			//ready second presentation of haptic
			if (rotate_two){
				//rotate_two = false;
				presentHapticTarget(hapticIDs[1]);
				rotate_two = false;

				beepOk(4);
				currentStage = second_release;
				//command second grasp of object
			} 

		} else if (timer_started == 1 && currentStage == second_release) {
			if (!handOnObject) {
				drawFingers(0);
				if(stdFirst){
					targetOriginY = y_shift[comparison_objId];
				}else{
					targetOriginY = y_shift[standard_objId];
				}
				drawCross();
			} else {
				presentation_timer.start();
				presentationTime = presentation_timer.getElapsedTimeInMilliSec();
				timer_started = 2;
			}
			//draw second object
		} else if (timer_started == 2 && task >= 2 && presentationTime <= pres_time && handOnObject){
			
			drawRandomDots(container[1]);
			drawPanels();

			//command final release
		} else if(timer_started == 2){
			if (currentStage == second_release && handOnObject && presentationTime >= pres_time){
				beepOk(5);
				currentStage = make_responce;
				
			}
		} else if(currentStage == showProgBar){
			if(timer_started == 3 && presentationTime <= pres_time){
				drawProgressBar();
			}else{
				advanceTrial();
			}
		
		}
	}
}

void stepperRotate(double destinationLength){
	double stepperMaxLength=60;
	double stepperZeroLength=18;

	if(destinationLength>=stepperZeroLength && destinationLength<=stepperMaxLength){
		destinationLength -= stepperZeroLength; // subtract off the start length (when stepper thinks it's at 0)
		destinationLength /= 0.794; // .794 is the pitch (in mm) of a #10-32 machine screw, so this gives the number of revolutions needed to be at destinationLength mm
		double destinationAngle = destinationLength*360; // convert those revolutions into degrees

		// 1 - to the middle
		//stepper_rotate(rotTable,-360*(stepperMaxLength-stepperZeroLength)/2/.794, 8.888888);
		// 2 - to a random location
		//stepper_rotate(rotTable,-360*(30+rand()%11-20)/.794, 8.888888);

		// 1 - Random Loc 1
		//stepper_rotate(rotTable,-360*((stepperMaxLength-stepperZeroLength)/2/.794-15+rand()%31), 8.888888);
		// 2 - Random Loc 2
		//stepper_rotate(rotTable,-360*((stepperMaxLength-stepperZeroLength)/2/.794-12+rand()%25), 8.888888);
		// 3 - to the destination
		stepper_rotate(rotTable, -destinationAngle, 8.888888); // go to the desired position, negative because of the way the actuator is built
	}
}

void initTrial()
{   
	currentStage = first_grasp;
	timer_started = 0;
	currentStage = first_grasp;

	//stepperRotate(55);

		switch(dummy_task){
		case 0:
			trial_noise = trial.getCurrent()["trialNoise"];
			comparison_objId = trial.getCurrent()["hapticObjId"];
			break;

		case 1:
			task = 1;
			comparison_objId = trial.getCurrent()["hapticObjId"];
			break;

		case 2:
			trial_noise = trial.getCurrent()["trialNoise"];
			comparison_objId = trial.getCurrent()["hapticObjId"];
			task = trial.getCurrent()["taskType"];
			break;

	}

		if(training){
			trial_noise = trial.getCurrent()["trialNoise"];
			comparison_objId = trial.getCurrent()["hapticObjId"];
			task = trial.getCurrent()["taskType"];
		}
	//comparison_objId = trial.getCurrent()["hapticObjId"];

	//trial_noise = trial.getCurrent()["trialNoise"];

	container[0].clear();
	container[1].clear();
	//standard_container.clear();
	//comparison_container.clear();
	//container[0] = standard_container;
	//container[1] = comparison_container;

	rotate_one = true;
	rotate_two = true;



	// task 0 is vision only, task 1 is haptic only, 
	// task 2 is vision and haptic combine in congruent way
	// task 3 is vision and haptic combine in incongruent way with vision 54 and haptic 56
	// task 4 is vision and haptic combine in incongruent way with vision 53 and haptic 57
	// task 5 is vision and haptic combine in incongruent way with vision 57 and haptic 53
	// task 6 is vision and haptic combine in incongruent way with vision 56 and haptic 54
	//double objSizes[9] = { 55.0, 56.0, 57.0, 59.0, 51.0, 54.0, 48.0, 53.0, 62.0 };
	switch(task){
	case 0:
		standard_height = 55.0;
		haptic_height_s = 0.0;
		break;

	case 1:
		standard_height = 0.0;
		standard_objId = 0;
		//haptic_height_s = 55.0;
		break;

	case 2:
		standard_height = 55.0;
		standard_objId = 0;
		//haptic_height_s = 55.0;
		break;

	case 3:
		standard_height = 54.0;
		standard_objId = 1;
		//haptic_height_s = 56.0;
		break;

	case 4:
		standard_height = 53.0;
		standard_objId = 2;
		//haptic_height_s = 57.0;
		break;

	case 5:
		standard_height = 57.0;
		standard_objId = 7;
		//haptic_height_s = 53.0;
		break;

	case 6:
		standard_height = 56.0;
		standard_objId = 5;
		//haptic_height_s = 54.0;
		break;


	}

	haptic_height_s = objSizes[standard_objId];
	haptic_height_c = objSizes[comparison_objId];
	//standard_height = objSizes[standard_objId];

	comparison_height = objSizes[comparison_objId];
	if(task == 1){
		comparison_height = 0;
	}

	stimulus_width = 30;


	//randomize the conditions
	
	if(rand() % 100 > 50) {
		stdFirst = true;
		container[0] = buildRandomDots(standard_height);
		container[1] = buildRandomDots(comparison_height);
		hapticIDs[0] = standard_objId;
		hapticIDs[1] = comparison_objId;

	} else {
		stdFirst = false;
		container[1] = buildRandomDots(standard_height);
		container[0] = buildRandomDots(comparison_height);
		hapticIDs[1] = standard_objId;
		hapticIDs[0] = comparison_objId;
	}

	//container[0] = buildRandomDots(standard_height);

	if(task == 0){
		beepOk(98);
	}else{
		beepOk(99);
	}

	trial_time.start();
	presentation_timer.start(); presentation_timer.stop(); 
	time_checkpoint = 10000;
}


void writeResponse(){
	//"subjName\tIOD\tblockN\ttrialN\ttask\tstdVisual\tstdHaptic\ttestVisual\ttestHaptic\trefIsFirst\tfirstIsTaller\trespondStdTaller\tRT";
	responseFile <<
		parameters.find("SubjectName") << "\t" <<
		interoculardistance << "\t" <<
		blkNum << "\t" <<
		trialNumber << "\t" <<
		task << "\t" <<
		trial_noise  << "\t" <<
		standard_height << "\t" <<
		haptic_height_s << "\t" <<
		comparison_height << "\t" <<
		haptic_height_c << "\t" <<
		stdFirst << "\t" <<
		firstTaller << "\t" <<
		//respond_stdTaller << "\t" <<
		respond_stdTaller << "\t" <<
		training << endl;
	//ElapsedTime - firstFixationTime - interTrialTime - 2 * presentationTime << endl;
}

void testValidTrial(){

	if(!trial.isEmpty()){
		trial.next();

		if(task > 2){
			comparison_objId = trial.getCurrent()["hapticObjId"];
			skipTrial = (comparison_objId != 3);

		}else{
			skipTrial = false;

		}

	}else{
		skipTrial = false; // it hits the end
	}

	if(fingersCalibrated && trial.isEmpty() && !skipTrial){
		lastTrialInBlk ++ ;}

}

void nextTilValidTrial(){

	testValidTrial();
	while(skipTrial){
		testValidTrial();
	}

}


void advanceTrial(){
	/*

	writeResponse();

	nextTilValidTrial();

	if(!trial.isEmpty()){

	trialNumber++;
	initTrial();

	}else if(lastTrialInBlk == 1){
	trialNumber++;
	initTrial();
	}else if(blkNum < totalBlks){
	blkNum++;
	//trial.init(parameters);			
	stepper_rotate(rotTable, 0, 238.67);
	initBlock();
	currentStage = breakTime;
	visibleInfo=true;
	//currentStage = breaktime;

	}else {
	beepOk(4);
	responseFile.close();
	visibleInfo=true;
	finished = true;

	}
*/


	writeResponse();

	if(!trial.isEmpty()){
		trialNumber++;
		trial.next();
		initTrial();
		percentComplete = 100 * (trialNumber)/totaltrialNum;
	}else if(blkNum < totalBlks){
		blkNum++;	
		stepper_rotate(rotTable, 0, 238.67);
		initBlock();
		currentStage = breakTime;
		visibleInfo=true;
	}else {
		beepOk(4);
		responseFile.close();
		visibleInfo=true;
		finished = true;}
	
}

void initBlock(){

	trialNumber = 1;
	//task = trial.getCurrent()["taskType"];

	dummy_task = blkOrder[blkNum - 1];
	lastTrialInBlk = 0;

	switch(dummy_task){
		case 0:
			task = 0;
			trial.init(parameters_V);
			totaltrialNum = 4 * 8 * repetitions; // 4 noise levesls, 8 objects
			break;

		case 1:
			task = 1;
			trial.init(parameters_H);
			totaltrialNum = 8 * repetitions; // 1 noise level, 8 objects
			break;

		case 2:
			trial.init(parameters_VH);
			totaltrialNum = 5 * 4 * 8 ; // 5 tasks, 4 noise levels, 8 objects
			break;

	}

	//nextTilValidTrial();
	trial.next();


}

// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{   switch (key)
{   
		case 'i':
			visibleInfo=!visibleInfo;
			break;
		case 'T':
		case 't':
			if(training){
			beepOk(1);
			blkNum++;	
			stepper_rotate(rotTable, 0, 238.67);
			initBlock();
			currentStage = breakTime;
			visibleInfo=true;
			training = false;
			initBlock();}
			break;
		case 'm':
			{
				interoculardistance += 0.5;
				headEyeCoords.setInterOcularDistance(interoculardistance);
			}
			break;
		case 'n':
			{
				interoculardistance -= 0.5;
				headEyeCoords.setInterOcularDistance(interoculardistance);
			}
			break;
		case 'Q':
		case 'q':
		case 27:	//corrisponde al tasto ESC
			{   
				// Ricorda quando chiami una funzione exit() di chiamare prima cleanup cosi
				// spegni l'Optotrak ed i markers (altrimenti loro restano accesi e li rovini) 
				shutdown();
			}
			break;
		case 'r':
		case 'R':
			{
				//Initializes trial without the use of finger calibrations
				//fingersCalibrated=false;
				if(!screenBack){
					beepOk(0);
					screenBack = true;
				}else{
					beepOk(0);
					visibleInfo=false;
					//trial.next(false);
					initTrial();
				}

			}
			break;

		case '1':
			{
				if (currentStage == make_responce && !handOnObject)
				{
					firstTaller = true;
					beepOk(2);
					if (stdFirst)
						respond_stdTaller = true;
					else
						respond_stdTaller = false;
					if(trialNumber % 16 == 0 ){
						
						presentation_timer.stop();
						currentStage = showProgBar;
						timer_started = 3;
						presentation_timer.start();
					}else{
						advanceTrial();
					}
				}
			}
			break;

		case '4':
			{  // if anything goes wrong, we can restart the trial
				stepper_rotate(rotTable, 0, 238.67);
				initTrial();
			}
			break;

		case '2':
			{
				if (currentStage == make_responce && !handOnObject)
				{
					firstTaller = false;
					beepOk(2);
					//if the second presented stimulus was a test stimulus (first presented stimulus was a reference)
					if (stdFirst)
						respond_stdTaller = false;
					else
						respond_stdTaller = true;

					if(trialNumber % 16 == 0 ){
						
						presentation_timer.stop();
						currentStage = showProgBar;
						timer_started = 3;
						presentation_timer.start();

					}else{
						advanceTrial();
					}
				}
			}
			break;

		case 'a':
		case 'A':
			{
				if(blkNum > 0 && currentStage == breakTime){

					//nextTilValidTrial();
					//trial.next();
					visibleInfo=false;
					initTrial();
				}
				
			}break;

		case 'f':
		case 'F':
			{

				if (fingerCalibrationDone == 0 && allVisibleFingers)
				{
					// get marker locations
					if (isVisible(markers[centercalMarker].p)) {
						fingerCalibrationDone = 1;
						calibration_fingers(fingerCalibrationDone);
						beepOk(0);
					}
					else {
						beepOk(5);
					}
					break;
				}

				if (fingerCalibrationDone == 1 && allVisibleFingers)
				{
					// set fingertip positions
					fingerCalibrationDone = 2;
					calibration_fingers(fingerCalibrationDone);
					beepOk(0);
					break;
				}

				if (fingerCalibrationDone == 2 && allVisibleFingers)
				{
					// set joint/knuckle positions
					fingerCalibrationDone = 3;
					calibration_fingers(fingerCalibrationDone);
					beepOk(0);
					break;
				}

				if (fingerCalibrationDone == 3 && allVisibleFingers && isVisible(markers[centercalMarker].p))
				{
					// move the carousel to the place

					centercal[0] = markers[centercalMarker].p.x();
					centercal[1] = markers[centercalMarker].p.y();
					centercal[2] = markers[centercalMarker].p.z();

					moveTo[2] = display_distance - z_shft;
					initProjectionScreen(display_distance);
					moveObjectAbsolute(moveTo, centercal, 6200);

					fingerCalibrationDone = 4;
					beepOk(0);

					break;
				}

				if (fingerCalibrationDone == 4 && !fingersCalibrated )
				{   

					if( isVisible(markers[centercalMarker].p)){

						//current object updating
						targetMarkerX = markers[centercalMarker].p.x();
						targetMarkerY = markers[centercalMarker].p.y();
						targetMarkerZ = markers[centercalMarker].p.z();
						//adjust the current position
						targetOriginX = targetMarkerX + x_shft;
						targetOriginY = targetMarkerY + y_shift[0];
						targetOriginZ = targetMarkerZ + z_shft;				
					}

					fingersCalibrated = true;

					beepOk(0);

					//initProjectionScreen(display_distance);

					// initialize			
					//blockNum = 1;
					//trialNum = 1;
					//attemptNum = 1;

					visibleInfo = false;

					//initBlock();

					//nextTilValidTrial();

					initTrial();

				}
			}
			break;
}
}
/***** SOUNDS *****/
void beepOk(int tone)
{

	switch(tone)
	{
	case 0:
		// Remember to put double slash \\ to specify directories!!!
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-1.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 1:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-6.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

		//When the participant has either hit the ceiling or floor
	case 3:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-reject.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

		//engage your hand/start of trial
	case 4:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-lowBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 15:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-rising.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 17:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 18:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck-5below.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
		//move your biscuit hooks
	case 5:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-falling.wav", 
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 98:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-look.wav", 
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 99:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-grasp.wav", 
			NULL, SND_FILENAME | SND_ASYNC);
		break;

			case 101:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-missed.wav", 
			NULL, SND_FILENAME | SND_ASYNC);
		break;


	}
	return;
}
void idle()
{
	handOnObject = distanceGripCenterToObject <= 15;
	handSafeDistance = distanceGripCenterToObject >= 30;
	online_fingers();
	online_apparatus_alignment();
	ElapsedTime = trial_time.getElapsedTimeInMilliSec();
	presentationTime = presentation_timer.getElapsedTimeInMilliSec();

}

/*** Online operations ***/
void online_apparatus_alignment()
{
	updateTheMarkers();
	// mirror alignment check
	mirrorAlignment = asin(
		abs((markers.at(mirror1).p.z() - markers.at(mirror2).p.z())) /
		sqrt(
		pow(markers.at(mirror1).p.x() - markers.at(mirror2).p.x(), 2) +
		pow(markers.at(mirror1).p.z() - markers.at(mirror2).p.z(), 2)
		)
		) * 180 / M_PI;

	// screen Y alignment check
	screenAlignmentY = asin(
		abs((markers.at(screen1).p.y() - markers.at(screen3).p.y())) /
		sqrt(
		pow(markers.at(screen1).p.x() - markers.at(screen3).p.x(), 2) +
		pow(markers.at(screen1).p.y() - markers.at(screen3).p.y(), 2)
		)
		) * 180 / M_PI;

	// screen Z alignment check
	screenAlignmentZ = asin(
		abs(markers.at(screen1).p.z() - markers.at(screen2).p.z()) /
		sqrt(
		pow(markers.at(screen1).p.x() - markers.at(screen2).p.x(), 2) +
		pow(markers.at(screen1).p.z() - markers.at(screen2).p.z(), 2)
		)
		) * 180 / M_PI *
		abs(markers.at(screen1).p.x() - markers.at(screen2).p.x()) /
		(markers.at(screen1).p.x() - markers.at(screen2).p.x());

}


int main(int argc, char*argv[])
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();

	// initializes optotrak and velmex motors
	initOptotrak();
	initMotors();

	// initializing glut
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
	glutGameModeString("1024x768:32@85");
	glutEnterGameMode();
	glutFullScreen();


	// initializing experiment's parameters

	initRendering();

	initStreams();

	/*
	if(isVisible(markers[centercalMarker].p)){
	rotTable = stepper_connect();
	centercal[0] = markers[centercalMarker].p.x();
	centercal[1] = markers[centercalMarker].p.y();
	centercal[2] = markers[centercalMarker].p.z();
	//current object updating
	targetMarkerX = markers[objMarkers[objId]].p.x();
	targetMarkerY = markers[objMarkers[objId]].p.y();
	targetMarkerZ = markers[objMarkers[objId]].p.z();
	//adjust the current position
	targetOriginX = targetMarkerX + x_shft;
	targetOriginY = targetMarkerY + y_shift[0];
	targetOriginZ = targetMarkerZ + z_shft;

	}*/
	// glut callback
	glutDisplayFunc(drawGLScene);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutIdleFunc(idle);
	glutTimerFunc(TIMER_MS, update, 0);
	glutSetCursor(GLUT_CURSOR_NONE);

	boost::thread initVariablesThread(&initVariables);
	// Application main loop
	glutMainLoop();

	// When the program exists, clean up
	//cleanup();
	return 0;
}


