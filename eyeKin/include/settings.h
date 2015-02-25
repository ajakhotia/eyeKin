#ifndef __SETTINGS_H__
#define __SETTINGS_H__

//---Debug Flags---//
//#define DEBUG_TIME
//#define DEBUG_PROFILER
//#define DEBUG_POINTCLOUD
//#define DEBUG_IMAGE_VIEWER
//#define DEBUG_DRAW_ON_RGB_IMAGE
//#define DEBUG_WRITE_PATCHES_TO_DISK
//#define DEBUG_LOG_MESSAGES
//#define WRITE_TO_VIDEO

//---OpenFrameworks settings---//
#define DEFAULT_SCREEN_WIDTH 1920
#define DEFAULT_SCREEN_HEIGHT 1080

//---RANSAC & Threshold settings---//
#define DEFAULT_MIN_DEPTH_LIMIT 0.8
#define DEFAULT_MAX_DEPTH_LIMIT 1.5
#define DEFAULT_DEPTH_MARGIN 0.007
#define DEFAULT_MAX_RANSAC_ITERATIONS 500
#define DEFAULT_DISTANCE_CUTOFF 0.01
#define DEFAULT_RADIAL_CUTOFF 0.60
#define DEFAULT_CLUSTER_TOLERANCE 0.03//0.018
#define DEFAULT_MINIMUM_CLUSTER_SIZE 25
#define DEFAULT_MAXIMUM_CLUSTER_SIZE 450

//---Canny Edge Detector Settings---//
#define DEFAULT_CANNY_LOW_THRESHOLD 70
#define DEFAULT_CANNY_HIGH_THRESHOLD 120
#define DEFAULT_CANNY_KERNEL_SIZE 3

//---IRCanny Edge Detector Settings---//
#define DEFAULT_IRCANNY_LOW_THRESHOLD 70
#define DEFAULT_IRCANNY_HIGH_THRESHOLD 120
#define DEFAULT_IRCANNY_KERNEL_SIZE 3

//---Object Tracking Settings---//
#define OBJECT_DIFFERENCE_THRESHOLD 150
#define OBJECT_MOVEMENT_THRESHOLD 20

#endif