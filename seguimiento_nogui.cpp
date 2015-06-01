#include <sstream>
#include <string>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <raspicam/raspicam_cv.h>
#include <time.h>
#include "main.h"

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
	bool objectFound = false;
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
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;


			}
		}
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
	struct timespec tstart = {0,0}, tend = {0,0};

	while ((*mem_global).salida){

		//Comprobacion temporal
		clock_gettime(CLOCK_MONOTONIC, &tstart);

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
				
		//Comprobacion temporal
		clock_gettime(CLOCK_MONOTONIC, &tend);
		//std::cout << "Valor de x: " << (*mem_global).x << " Valor de y: " << (*mem_global).y << std::endl;
		//std::cout << "Diferencia de tiempo: " << ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec) << std::endl;

	}
}
