#ifndef __OBJECT_SEGMENTATION_H__
#define __OBJECT_SEGMENTATION_H__

#include "settings.h"
#include "kinectReader.h"
#include "entity.h"
#include "pcl.h"
#include "timer.h"

namespace personalRobotics
{
	//! Stores characteristic information regarding an @link Entity @endlink.
	/*!	Stores characteristic information regarding each entity for matching
		and keeping track of different objects across frames.*/
	struct IDLookUp
	{
		int id;								//!< ID of the object.
		cv::Point2f centroid;				//!< Location of centroid in the 2D color image.
		float angle;						//!< Angle between the principle axis of the object and the image.
		cv::Size2f boundingSize;			//!< The size, in pixels, of the bounding box oriented along the principle axis of the object. 
		int numFramesSame;					//!< The number of frames over which the object has been observed to be static.
	};

	class ObjectSegmentor : public KinectReader
	{
	protected:
		// Config params
		int maxRansacIters;					//!< Maximum iterations for the RANSAC for estimating the table plane.
		float ransacMargin;					//!< Threshold to distinguish inliers from outliers. Any point is a inlier to a hypothesised plane if the distance between the plane and the point is less than this threshold.
		float distCutoff;					//!< Distance margin within which a point is consider to belong to the table plane.
		float radialThreshold;				//!< [DEPRECATED] The radius of the circle beyond which the depth pixel are ignored. The distances are computed after accounting for the intrinsics of the depth camera matrix(d = sqrt((X/Z)^2 + (Y/Z)^2)).
		float clusterTolerance;				//!< Threshold for including a point in a cluster. The point is included in the cluster if the distance between the point and its closest point in the cluster is less this threshold.
		int minClusterSize;					//!< Minimum size of the cluster below which the cluster is drpped. The size of the cluster is the number of points that belong to that cluster.
		int maxClusterSize;					//!< Maazimum size of the cluster beyond which the cluster is dropped for computaional purposes. The size of the cluster is the number of points that belong to that cluster.
		float minThreshold;					//!< Minimum depth(Z) below which the points are rejected.
		float maxThreshold;					//!< Maximum depth(Z) beyond which the points are rejected.
		float objectDifferenceThreshold;	//!< Threshold score below which two entities in consecutive frames are considered the same. The score is computed by calculateEntityDifferences() function.
		float objectMovementThreshold;		//!< Minimum distance the centroid of an entity needs to translate in two cosecutive frames for the entity to be considered non-static.

		// Containers
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;			//!< Container to store the pruned point cloud. See @link planeNormals @endlink.
		pcl::ModelCoefficients::Ptr planePtr;				//!< Equation of the plane as estimated by findTablePlane() using least squares fit with RANSAC for robust estimation.
		std::vector<personalRobotics::Entity> entityList;	//!< A list of entities detected in the current scene.
		cv::Mat homography;									//!< The homography set by the function setHomography().
		
		//!	Set of 4 plane normals, expressed in the same coordinate system as the point cloud, that connect the origin of the camera space to the 4 edges of the projected screen.
		/*!	Set of 4 plane normals, expressed in the same coordinate system as the point cloud,
			that connect the origin of the camera space to the 4 edges of the projected screen.
			As the planes pass through the camera center, the d term is 0, giving a complete 
			plane equation. Only points that lie with-in the frustum fromed by the four planes
			are considered for further processing like clustering, etc. by the planeSegment()
			function.*/
		std::vector <cv::Point3f> planeNormals;				 
		// Pixel sizes
		cv::Point2f rgbPixelSize;							//!< A rough estimate of the rgb pixel size when projected on to the table plane. The size is expressed in mm.  

		// ID lists
		std::vector<IDLookUp> previousIDList;				//!< A list containing characteristic information of the entities in the last frame for the purpose of entity association across frames.
		std::vector<IDLookUp> currentIDList;				//!< A list containing characteristic information of the entities in the current frame for the purpose of entity association across frames.

		// Container locks
		std::mutex pclPtrLock;								//!< Mutex to prevent race conditions while accessing @link pclPtr @endlink.
		std::mutex entityListLock;							//!< Mutex to prevent race conditions while accessing @link entityList @endlink.

		// Control flags
		MutexBool stopSegmentorFlag;						//!< Control varibale for the while loop in segmentorThreadRoutine(). The loop ends and the @link segementorThread @endlink exits when this is set to true by stopSegmentor() function.
		MutexBool homographySetFlag;						//!< Set to true when homograpgy is set by the setHomography() function.

		// Runner threads
		std::thread segementorThread;						//!< Handle to the thread running the segmentorThreadRoutine().

		//Counters
		bool frameStatic;									//!< Set to true if all the valid entities in the current frame appear to be static. false otherwise.
		bool prevFrameStatic;								//!< Set to true if all the valid entities in the previous frame appeared to be static. false otherwise.
		int objectCount;									//!< Count of valid objects in the frame.

		// Routines
		//!	The core routine of the program which segments out objects on the table and generates a list of entities with relavant information.
		/*!	This is the core routine of the program. The function perfoms the
			following operations.
				@li @c Prunes the point cloud based on the @link planeNormals @endlink to reject all the point outside the region of interest.
				@li @c Perform plane based segmentation and extract all the points that lie above the table plane.
				@li @c Perform statistical outlier removal to suppress noise followed by voxel grid based down-sampling.
				@li @c Perform clustering. Each cluster thus formed corresponds to an entity in the scene.
				@li @c Check the validity of each cluster and rejecting the clusters that are connected to the frustum walls/(@link planeNormals @endlink).
				@li @c Reproject each cluster onto the RGB image and compute the centroid, principle axes and 2D pose. 
				@li @c Match each cluster's attributes with the attributes of the clusters in the previous frame and create an association between the clusters across the frames.
				@li @c Check if the scene has come to rest and generate data for each entity in the list of entities. See @link generateData() @endlink. */
		void planeSegment();
	public:
		// Constructor and Destructor
		//!	Default constructor. Initializes all the parameters to the defaults specified in @link setting.h @endlink .
		/*!	Constructs the object with the default values as specified in @link setting.h @endlink .
			Also starts the kinect reader thread by calling startKinect()*/
		ObjectSegmentor();

		// Default destructor. Stops the object segmentor and the kinect reader threads if not stopped already.
		~ObjectSegmentor();

		// Thread safety measures
		void lockList();					//!< Locks the @link entityList @endlink. The callers should call unlockList() once the processing is completed.
		void unlockList();					//!< Unlocks the @link entityList @endlink. This method should not be called before a call to lockList() by the same thread.
		void lockPcl();						//!< Locks @link pclPtr @endlink. The caller should call unlockPcl() once the processing is completed.
		void unlockPcl();					//!< Unlocks @link pclPtr @endlink. This method should not be called before a call to lockPcl() by the same thread.
		MutexBool newListGenerated;			//!< Set to true each time the planeSegment() method generates a new list of entities. The consumer function can set it to false after acknowleding the arrival of new list and use it as a signal to wait on.
		MutexBool pauseThreadFlag;			//!< Pauses the segmentorRoutine() to pause producing new @link enityList @endlink . Refer @sa pauseSegmentor(), resumeSegmentor()

		// Accessors
		std::vector<personalRobotics::Entity>* getEntityList();		//!< Fetches a pointer to  @link entityList @endlink. The calling function should also use lockList() and unlockList() appropriately to avoid race conditions.
		cv::Point2f* getRGBpixelSize();								//!< Fetches a pointer to @link rgbPixelSize @endlink .
		std::vector<IDLookUp>* getIDList();							//!< Fetches a pointer to @link previousIDList @endlink .
		bool getStatic();											//!< Returns true if the scene is static, false otherwise.

		// Setters
		//!	Sets the homography between the color camera of the kinect and the projected screen followed by, computing the @link planeNormals @endlink .
		/*! Sets the homography between the color camera of the kinect and the projected screen followed by,
			computing the @link planeNormals @endlink . This routine needs the @link planePtr @endlink be
			set by findTablePlane() function.*/
		void setHomography(cv::Mat inhomography, int width, int height);

		// Routines
		//!	Estimates the table plane equation using least squares fit with RANSAC.
		/*!	Estimates the table plane equation using least squares fit. The outlies are pruned using
			RANSAC. The function assumes that the table top is the most dominant planar feature in
			the field of view of the kinects depth camera.*/
		bool findTablePlane();
		void startSegmentor();				//!< Starts the segmentor thread that runs segmentorThreadRoutine().
		void segmentorThreadRoutine();		//!< Runs planeSegment() in a loop, handling the pausing and stopping of the segmentor.
		void stopSegmentor();				//!< Stops the segmentorThreadRoutine and mergers the thread cleanly with the parent thread.
		void pauseSegmentor();				//!< Pauses the production of the @link entityList @endlist .
		void resumeSegmentor();				//!< Resumes the segmetor.
		bool calculateOverallChangeInFrames(std::vector<IDLookUp> cIDList);		//!< Returns true if the scene is static for a certain number of frames.
		float calculateEntityDifferences(cv::Point2f IDcentroid, cv::Point2f objectCentroid, float IDangle, float objectAngle, cv::Size2f IDBoundingSize, cv::Size2f objectBoundingSize); //!< Calculates a score that is a representative of difference between two entities based on pose and size. Smaller the score, better the match.
		bool onBoundingEdges(pcl::PointXYZ point);	//!< Return true if a point is close to the any of the plane in @link planeNormals @endlink . false otherwise.
	};

	void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY); //!< Creates a white bordered checkerboard pattern with the specified width and height in pixels and fills the numBlocksX and numBlocksY with the numbers of interior points in the generated checkerboard in X and Y direction respectively.
}
#endif