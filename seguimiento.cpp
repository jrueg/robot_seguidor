#include <sstream>
#include <string>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <raspicam/raspicam_cv.h>
#include <time.h>
#include "control.h"
#include "main.h"

//#define activa_gui

using namespace cv;

//initial min and max HSV filter values.
//these will be changed using trackbars
//default capture width and height
#define FRAME_WIDTH  320
#define FRAME_HEIGHT  240
//max number of objects to be detected in frame
#define MAX_NUM_OBJECTS 50
//minimum and maximum object area
#define MIN_OBJECT_AREA  20*20
#define MAX_OBJECT_AREA  FRAME_HEIGHT*FRAME_WIDTH/1.5
//names that will appear at the top of each window

#ifdef activa_gui
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
void on_trackbar(int, void*)
{//Esta función se ejecuta cuando cambia la posición de un trackbar

}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(struct mem_global *mem_global){
	//create window for trackbars


	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", (*mem_global).H_MIN);
	sprintf(TrackbarName, "H_MAX", (*mem_global).H_MAX);
	sprintf(TrackbarName, "S_MIN", (*mem_global).S_MIN);
	sprintf(TrackbarName, "S_MAX", (*mem_global).S_MAX);
	sprintf(TrackbarName, "V_MIN", (*mem_global).V_MIN);
	sprintf(TrackbarName, "V_MAX", (*mem_global).V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &(*mem_global).H_MIN, (*mem_global).H_MAX, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &(*mem_global).H_MAX, (*mem_global).H_MAX, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &(*mem_global).S_MIN, (*mem_global).S_MAX, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &(*mem_global).S_MAX, (*mem_global).S_MAX, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &(*mem_global).V_MIN, (*mem_global).V_MAX, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &(*mem_global).V_MAX, (*mem_global).V_MAX, on_trackbar);


}

void drawObject(int x, int y, Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}

#endif

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	//erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	//dilate(thresh,thresh,dilateElement);

}

void trackFilteredObject(struct mem_global *mem_global, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					(*mem_global).x = moment.m10/area;
					(*mem_global).y = moment.m01/area;
					(*mem_global).objetoEncontrado = true;
					refArea = area;
				}
				else{
					(*mem_global).objetoEncontrado = false;
					(*mem_global).x = 160;
					(*mem_global).y = 120;
				}
			}
			#ifdef activa_gui
				//let user know you found an object
				if ((*mem_global).objetoEncontrado == true){
					putText(cameraFeed, "Siguiendo objeto", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
					//draw object location on screen
					drawObject((*mem_global).x, (*mem_global).y, cameraFeed);
				}
			#endif
		}
	#ifdef activa_gui
		else putText(cameraFeed, "DEMASIADO RUIDO, AJUSTA FILTRO!", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	#endif
	}
	
}

void seguimiento(struct mem_global *mem_global)
{
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	//create slider bars for HSV filtering
	#ifdef activa_gui
		createTrackbars(mem_global);
	#endif
	//video capture object to acquire webcam feed
	raspicam::RaspiCam_Cv Camera;
	//Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
	//open capture object at location zero (default location for webcam)
	//capture.open("http://192.168.1.12:8080/videofeed?dummy=param.mjpg");
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	Camera.open();
	//set height and width of capture frame
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	
	//Comprobacion temporal
	//struct timespec tstart = {0,0}, tend = {0,0};

	//Control de servos
	//Servo en x
	int pos0 = 50;
	controlador_p con_s0(0.05, 90, -90, 10);
	servoBlaster(0, pos0);

	//Servo en x
	int pos1 = 60;
	controlador_p con_s1(0.05, 90, -90, 10);
	servoBlaster(1, pos1);

	while ((*mem_global).salida){

		//Comprobacion temporal
		//clock_gettime(CLOCK_MONOTONIC, &tstart);
		
		//store image to matrix
		Camera.grab();
		Camera.retrieve(cameraFeed);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar((*mem_global).H_MIN,(*mem_global).S_MIN,(*mem_global).V_MIN),Scalar((*mem_global).H_MAX,(*mem_global).S_MAX,(*mem_global).V_MAX),threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		trackFilteredObject(mem_global ,threshold,cameraFeed);
		
		#ifdef activa_gui
			//show frames 
			imshow(windowName2, threshold);
			//imshow(windowName1,HSV);
			imshow(windowName, cameraFeed);

			//std::cout << "Valor de x: " << (*mem_global).x << " Valor de y: " << (*mem_global).y << std::endl;

			//delay 30ms so that screen can refresh.
			//image will not appear without this waitKey() command
			waitKey(10);
		#endif

		//Comprobacion temporal
		//clock_gettime(CLOCK_MONOTONIC, &tend);
		
		//Control de servos
		if ((*mem_global).objetoEncontrado){
			//Servo en x
			pos0 -=	con_s0.calculo_realim((*mem_global).x);
			if (pos0 > 90) pos0 = 90;
			if (pos0 < 10) pos0 = 10;
			servoBlaster(0, pos0);
			//std::cout << "u = " << (int)u0 << " y = " << pos0 << std::endl;

			//Servo en y
			pos1 -= con_s1.calculo_realim((*mem_global).y);
			if (pos1 > 70) pos1 = 70;
			if (pos1 < 40) pos1 = 40;
			servoBlaster(1, pos1);
			//std::cout << "u = " << (int)u0 << " y = " << pos0 << std::endl;
		}

	}


	servoBlaster(0, 50);
	servoBlaster(1, 60);

}
