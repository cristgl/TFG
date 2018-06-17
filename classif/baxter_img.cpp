#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#define uchar unsigned char
#define _r(x, y, image) ((uchar *)((image)->imageData + ((image)->widthStep) * (x)))[3 * y + 2]
#define _g(x, y, image) ((uchar *)((image)->imageData + ((image)->widthStep) * (x)))[3 * y + 1]
#define _b(x, y, image) ((uchar *)((image)->imageData + ((image)->widthStep) * (x)))[3 * y]
#define _Ma(x, y, mat) ((mat)->data.fl + (mat)->step / 4 * (x))[y]
#include <iostream>

using namespace cv;
using namespace std;
uchar *temp;
int flag = 1;
uint32_t tam_centroid = 0;
int size_ = 0;
char name[30]; 
bool finished = false;
IplImage *head;
IplImage *temp2;
IplImage *left_hand;
IplImage *right_hand;
IplImage *red;
IplImage *green;
IplImage *blue;
IplImage *src_result;

CvMat *TRANS = cvCreateMat(3, 3, CV_32FC1);
CvMat *INVTRANS = cvCreateMat(3, 3, CV_32FC1);
CvMat *DIS = cvCreateMat(1, 4, CV_32FC1);
int WIDTH, HEIGHT, STEP;
int NUM_OBJ = 0;

// Low hsv values for blue colors.
uchar COLOR_blue_low[3] = {103, 85, 47}; 
// High hsv values for blue values.
uchar COLOR_blue_high[3] = {113, 203, 227}; 
// Low hsv values for green colors.
uchar COLOR_green_low[3] = {58, 35, 29}; 
// High hsv values for green values.
uchar COLOR_green_high[3] = {95, 147, 191}; 
// Low hsv values for orange colors.
uchar COLOR_orange_low[3] = {2, 65, 90};
// High hsv values for orange values.
uchar COLOR_orange_high[3] = {37, 180, 255}; 
// Low hsv values for purple colors.
uchar COLOR_purple_low[3] = {111, 40, 40};
// High hsv values for purple values.
uchar COLOR_purple_high[3] = {135, 127, 221};

extern void GetPosition(IplImage *, uchar*, uchar*, int num);
extern int GetColor(IplImage *);
extern void FilterImage(IplImage *, IplImage *, int tope_y);
extern void FilterColor(IplImage *, IplImage *, uchar *, uchar *);

ros::Publisher pos_pub;
ros::Publisher clasificacion;
// The center postion and size of object in image frame.
typedef struct pos 
{
  int x, y;
  float angle;
  float size;
}pos;
// The center position and size of one object in world frame.
typedef struct re_pos 
{
  float x, y, z;
  float angle;
  int flag;
  float size;
}pos_;

// Convert the coordinate from the image frame to world frame.
extern void Get3DPos(pos *src, pos_ *dis);
// Get object center position in world frame which called by Get3DPos.
extern void GetRealPos(float, float, float, float, float, float, float, float, pos_*);
// Buffer for obj pos in img frame.
pos *pos_head = (pos *)malloc(sizeof(pos) * 100);
// Buffer for the points pos in world frame.
//pos_ *rel_pos = (pos_ *)malloc(sizeof(pos_) * 100);

pos_ *good_pos = (pos_ *)malloc(sizeof(pos_) * 100);

// Convert image topic data into IplImage data structure.
void ImageToIpl(uchar *src, IplImage *dis)
{
	for(int i = 0; i != HEIGHT; i ++)
	{
		for(int j = 0; j != WIDTH; j++)
		{
			((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j] = src[dis->widthStep / 3 * 4 * i + 4 * j + 0];
			((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j + 1] = src[dis->widthStep / 3 * 4 * i + 4 * j + 1];
			((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j + 2] = src[dis->widthStep / 3 * 4 * i + 4 * j + 2];    
		}
	}
}

void ProcessAndPublish(geometry_msgs::PoseArray points){

	float m1, n1, x, y, norm_x, norm_y, norm_z;
	int points_size, k=2;
	cv::Mat centros(0, 0, CV_8U);
	pos *pos_a_devolver = (pos *)malloc(sizeof(pos) * 2);
	cv::Mat bestLabels, centers, img2show;
	geometry_msgs::PoseArray publicados;
	geometry_msgs::Pose punto_A, punto_B, punto_medio, inicio, a_devolver;

	points_size = points.poses.size();

	// Normalize data
	for (int i=0; i < points_size; i++){
		norm_x = (float) (points.poses[i].position.x - (-255))/(255.0 - (-255.0));
		centros.push_back(norm_x);

		norm_y = (float) (points.poses[i].position.y - (-255))/(255.0 - (-255.0));
		centros.push_back(norm_y);

		norm_z = (float) (points.poses[i].position.z - 1.0)/(255.0 - 1.0);
		centros.push_back(norm_z);
	}

	centros.convertTo(centros, CV_32F);
	centros = centros.reshape(0,points_size);

	if(!centros.empty()){

		cv::kmeans(centros, k, bestLabels, cv::TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 10, cv::KMEANS_RANDOM_CENTERS, centers);
		bestLabels = bestLabels.reshape(0, centros.rows);
		cv::convertScaleAbs(bestLabels, img2show, int(255/k));
		
		// Denormalize data
		punto_A.position.x = (centers.at<float>(0,0) * (255 - (-255))) + (-255);
		punto_A.position.y = (centers.at<float>(0,1) * (255 - (-255))) + (-255);
		punto_A.position.z = -0.09;
		punto_A.orientation.x = centers.at<float>(0,2);
		punto_A.orientation.y = 0;
		punto_A.orientation.z = 0;
		punto_A.orientation.w = 0;
		
		punto_B.position.x = (centers.at<float>(1,0) * (255 - (-255))) + (-255);
		punto_B.position.y = (centers.at<float>(1,1) * (255 - (-255))) + (-255);
		punto_B.position.z = -0.09;
		punto_B.orientation.x = centers.at<float>(1,2);
		punto_B.orientation.y = 0;
		punto_B.orientation.z = 0;
		punto_B.orientation.w = 0;

		// Calculate y = mx + n
		// m = (y2 - y1)/(x2 - x1)
		m1 = (punto_B.position.y - punto_A.position.y) / (punto_B.position.x - punto_A.position.x);
		
		// n = y - mx
		n1 = punto_A.position.y - m1 * punto_A.position.x;

		// Get the middle point between A and B: (xA+xB/2, yA+yB/2)
		punto_medio.position.x = (punto_A.position.x + punto_B.position.x)/2;
		punto_medio.position.y = (punto_A.position.y + punto_B.position.y)/2;
		punto_medio.position.z = -0.09;
		punto_medio.orientation.x = m1;
		punto_medio.orientation.y = 0;
		punto_medio.orientation.z = 0;
		punto_medio.orientation.w = 0;

		pos_a_devolver->x = punto_medio.position.x;
		pos_a_devolver->y = punto_medio.position.y;
		pos_a_devolver->angle = punto_medio.position.z;
		pos_a_devolver->size = punto_medio.orientation.x;
		
		Get3DPos(pos_a_devolver, good_pos);

		a_devolver.position.x = good_pos->x / 1000+0.07+0.57;
		a_devolver.position.y = good_pos->y / 1000-0.032;
		a_devolver.position.z = -0.09;
		a_devolver.orientation.x = m1; 
		a_devolver.orientation.y = 0;
		a_devolver.orientation.z = 0;
		a_devolver.orientation.w = 0;

		publicados.poses.push_back(a_devolver);
		clasificacion.publish(publicados);

	}else
		int n = 0;

    if(finished){
    	free(pos_a_devolver);
    	free(good_pos);
    	free(pos_head);
    }
}

// Subscriber call back function.
void ImageCallBack(const sensor_msgs::Image &msg)
{
	if (flag == 1)
	{
	// Set the global parameters for the image.
		WIDTH = msg.width;
		HEIGHT = msg.height;
		STEP = msg.step;
		size_ = msg.step * msg.height;
		temp = (uchar *)malloc(size_);
		flag ++;
		head = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
		temp2 = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
		red = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
		green = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
		blue = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
		src_result = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
	}
	
	for(int i = 0; i != size_; i++)
	{
		temp[i] = msg.data[i];
	}
	
	ImageToIpl(temp, temp2);

	FilterImage(temp2, head, 120);

	src_result = cvCloneImage(head);
	GetPosition(head, COLOR_blue_low, COLOR_blue_high, 1);
	geometry_msgs::PoseArray points;
	geometry_msgs::Pose pose;

	for(int i = 0; i != NUM_OBJ; i++)
	{
		// Assign values to pose and add offsets (in cm) to get the position 
		// in the baxter coordinate system (blue).
		pose.position.x = pos_head[i].x;
		pose.position.y = pos_head[i].y;
		pose.position.z = 230; //230
		pose.orientation.x = pos_head[i].size;
		// Publish the obj pos and orientation and size.
		points.poses.push_back(pose);
	}

	GetPosition(head, COLOR_green_low, COLOR_green_high, 2);
	for(int i = 0; i != NUM_OBJ; i++)
	{
		// Assign values to pose and add offsets (in cm) to get the position 
		// in the baxter coordinate system (green).
		pose.position.x = pos_head[i].x;
		pose.position.y = pos_head[i].y;
		pose.position.z = 255; //255
		pose.orientation.x = pos_head[i].size;
		// Publish the obj pos and orientation and size.
		points.poses.push_back(pose);
	}

	GetPosition(head, COLOR_orange_low, COLOR_orange_high, 3);
	for(int i = 0; i != NUM_OBJ; i++)
	{
		// Assign values to pose and add offsets (in cm) to get the position 
		// in the baxter coordinate system (orange).
		pose.position.x = pos_head[i].x;
		pose.position.y = pos_head[i].y;
		pose.position.z = pos_head[i].angle;
		pose.orientation.x = pos_head[i].size; 
		// Publish the obj pos and orientation and size.
		points.poses.push_back(pose);
	}

	GetPosition(head, COLOR_purple_low, COLOR_purple_high, 4);
	// src_result is the image with all the lined rectangles. 
	cvShowImage("detected objects", src_result);
	//cvSaveImage("/home/maik/Schreibtisch/save/detected objects.jpg", src);
	for(int i = 0; i != NUM_OBJ; i++)
	{
		// Assign values to pose and add offsets (in cm) to get the position 
		// in the baxter coordinate system (purple).
		pose.position.x = pos_head[i].x;
		pose.position.y = pos_head[i].y;
		pose.position.z = pos_head[i].angle;
		pose.orientation.x = pos_head[i].size; 
		// Publish the obj pos and orientation and size.
		points.poses.push_back(pose);
	}
	
	ProcessAndPublish(points);

}

void FreeMemory(const std_msgs::String::ConstPtr& msg){
	finished = true;
}

int main(int arg, char** argv)
{  
	// The Intrinsic Matrix (calibration paramter)
	// Focal length: fx
	_Ma(0, 0, TRANS) = 411.78954;	
	_Ma(0, 1, TRANS) = 0;
	// Principal point offset: cx
	_Ma(0, 2, TRANS) = 307.70709;
	_Ma(1, 0, TRANS) = 0;
	// Focal length: fy
	_Ma(1, 1, TRANS) = 411.10182;
	// Principal point offset: cy
	_Ma(1, 2, TRANS) = 236.83502;
	_Ma(2, 0, TRANS) = 0;	
	_Ma(2, 1, TRANS) = 0;
	_Ma(2, 2, TRANS) = 1;  
	// The inverse of the parameter matrix.   
	_Ma(0, 0, INVTRANS) = 0.0022042;
	_Ma(0, 1, INVTRANS) = 0;
	_Ma(0, 2, INVTRANS) = -0.74724;
	_Ma(1, 0, INVTRANS) = 0;
	_Ma(1, 1, INVTRANS) = 0.0022049;
	_Ma(1, 2, INVTRANS) = -0.57610;
	_Ma(2, 0, INVTRANS) = 0;
	_Ma(2, 1, INVTRANS) = 0;
	_Ma(2, 2, INVTRANS) = 1;
	// The distortion matrix.
	_Ma(0, 0, DIS) = 0.01386;
	_Ma(0, 1, DIS) = -0.058;
	_Ma(0, 2, DIS) = 0.00134;
	_Ma(0, 3, DIS) = 0.00227;

	ros::init(arg, argv, "test");
	ros::NodeHandle n;
	pos_pub = n.advertise<geometry_msgs::PoseArray>("detected_objects", 2);
	ros::Subscriber sub = n.subscribe("/cameras/right_hand_camera/image", 1, ImageCallBack);
	clasificacion = n.advertise<geometry_msgs::PoseArray>("clasificacion", 1);
	ros::Subscriber freeMem = n.subscribe("finished", 10, FreeMemory);
	ros::spin();
	return 0;
}

// Get object center position in world frame which called by Get3DPos.
void GetPosition(IplImage *src, uchar* COLOR, uchar* COLOR_, int num)
{
	IplImage *temp_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	IplImage *src_copy = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	IplImage *gray_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage *con = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage *con_;
	// Copy the original image to use for every color the same one.
	src_copy = cvCloneImage(src);
	NUM_OBJ = 0;
	// Do the gaussian filtering for the noise.
	cvSmooth(src_copy, src_copy, CV_GAUSSIAN, 5, 5);
	// Pick up the object and make everything else black.
	FilterColor(src_copy, blue, COLOR, COLOR_);
	/*
	// The variable num is only for looking at one specific color (example: green=2)
	if (num==2){
	  cvSaveImage("/home/maik/Schreibtisch/save/black.jpg", blue );
	  }
	*/
	// Remove the little holes on the object.
	cvErode(blue, temp_, NULL, 4);

	IplImage *gray = cvCreateImage(cvGetSize(src_copy), IPL_DEPTH_8U, 1);
	// Convert coloful img into mono.
	cvCvtColor(temp_, gray, CV_BGR2GRAY);
	// Convert the mono image into binary image.
	cvThreshold(gray, gray_, 20, 255, CV_THRESH_BINARY);
	con_ = cvCloneImage(gray_);
	CvMemStorage *st = cvCreateMemStorage(0);
	CvSeq *first = NULL;
	// Find the connective areas in the image.
	cvFindContours(con_, st, &first, sizeof(CvContour), CV_RETR_LIST);
	// CvSeq is a dynamic data structures to represent growable 1d arrays.
	for(CvSeq *c = first; c != NULL; c = c->h_next) 
	{
		// Use a square with angle to fit the connective area.
		CvBox2D rect = cvMinAreaRect2(c, st);
		// Square size of the rectangles.
		int size= (int) rect.size.width*rect.size.height; 
		// Ignore very small and very big objects.
		if(500<size & size<4000){
			//printf("size: %d \n", size);
			CvPoint2D32f boxPoints[4];
			// Store the edge points of the rectangle.
			cvBoxPoints(rect, boxPoints); 
			CvScalar color_frame = CV_RGB(0, 0, 0);
			// Draw lines between the points.
			cvLine(src_result,cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y),cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y),color_frame,3,8,0);
			cvLine(src_result,cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y),cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y),color_frame,3,8,0);
			cvLine(src_result,cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y),cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y),color_frame,3,8,0);
			cvLine(src_result,cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y),cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y),color_frame,3,8,0);

			// Assume that the real center is the same like the center of the image.
			pos_head[NUM_OBJ].x = (int) rect.center.x- WIDTH / 2; 
			pos_head[NUM_OBJ].y = -((int) rect.center.y- HEIGHT / 2);   
			pos_head[NUM_OBJ].angle = rect.angle;
			// The size in one length of the square in cm.
			pos_head[NUM_OBJ].size = (rect.size.width+rect.size.height)/2;   
			// Get the world frame postions.
			//Get3DPos(pos_head + NUM_OBJ, rel_pos + NUM_OBJ); 
			NUM_OBJ++; 
		}

	}

	waitKey(1);
	// Release the memory we have malloced.
	cvReleaseMemStorage(&st); 
	cvReleaseImage(&gray);
	cvReleaseImage(&temp_);
	cvReleaseImage(&gray_);
	cvReleaseImage(&src_copy);
	cvReleaseImage(&con);
	cvReleaseImage(&con_);
	//ROS_INFO("Detect [%d] objects", NUM_OBJ);
}

void FilterImage(IplImage *src, IplImage *dis, int tope_y){
	IplImage *temp_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	cvCvtColor(src, temp_, CV_BGR2HSV);

	for(int i = 0; i < src->height; i++){
		for(int j = 0; j < src->width; j++){
			if(i < 425 - tope_y || i > 348 + tope_y){
				_r(i, j, dis) = 0;
				_g(i, j, dis) = 0;
				_b(i, j, dis) = 0;
			} else{
				_r(i, j, dis) = _r(i, j, src);
				_g(i, j, dis) = _g(i, j, src);
				_b(i, j, dis) = _b(i, j, src);
			}
		}
	}
	cvReleaseImage(&temp_);
}

// Filter the color.
void FilterColor(IplImage *src, IplImage *dis, uchar* COLOR, uchar* COLOR_)
{
	IplImage *temp_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	cvCvtColor(src, temp_, CV_BGR2HSV);
	for(int i = 0; i != src->height; i++)
	{
		for(int j = 0; j != src->width; j++)
		{
			if(COLOR[0] <= _b(i, j, temp_) && COLOR_[0] >= _b(i, j,temp_) && COLOR[1] <= _g(i, j, temp_) && COLOR_[1] >= _g(i, j, temp_) && COLOR[2] <= _r(i, j, temp_) && COLOR_[2] >= _r(i, j, temp_))
			{
				_r(i, j, dis) = _r(i, j, src);
				_g(i, j, dis) = _g(i, j, src);
				_b(i, j, dis) = _b(i, j, src);
			}
			else
			{
				_r(i, j, dis) = 0;
				_g(i, j, dis) = 0;
				_b(i, j, dis) = 0;
			}
		}
	}
	cvReleaseImage(&temp_);
}

void Get3DPos(pos *src, pos_ *dis)
{
	// The factors 0.015 and 0.058 correct the small deviations from (int) rect.center.x- WIDTH / 2
	// and -((int) rect.center.y- HEIGHT / 2).
	float tempx = _Ma(0, 0, INVTRANS) * src->x + 0.015;  _Ma(0, 2, INVTRANS);
	float tempy = _Ma(1, 1, INVTRANS) * src->y + 0.058;  _Ma(1, 2, INVTRANS);

	GetRealPos(tempx, tempy, 0, 0, 1.0, 0, 1, 475, dis);
	dis->angle = src->angle;
	dis->flag = 1;
	// The sizes of the rectangles must be converted from pixels in cm in the real world.
	// real_size= image_size*(z/f), z: distance between camera and object, f: focal length
	// z/f=47,5/411.8= 0.115 but the size of the detected objects is allways a bit smaller. 
	// This is the reason for the adjustment of the parameter (calculation in readme).
	dis->size = (src->size)*0.128783;
}

void GetRealPos(float x, float y, float xz, float yz, float zz, float xp, float yp, float zp, pos_ *dis) 
{
	// xp ...zp is the coordinate of one point on the table, and the xz ...zz is the orientation of 
	// the  normal vector of the table, both of them are represented in the camera frame.
	// here x*zp. 
	dis->y = x * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz); 
	// here -y*zp. 
	dis->x = -y * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
	// here zp.
	dis->z = -(-xp * xz - yp * yz -zp * zz) / (x * xz + y * yz + zz); 
}


