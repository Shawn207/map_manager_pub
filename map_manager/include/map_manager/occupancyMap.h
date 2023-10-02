/*
	FILE: occupancyMap,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef MAPMANAGER_OCCUPANCYMAP
#define MAPMANAGER_OCCUPANCYMAP
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <map_manager/raycast.h>
#include <thread>

using std::cout; using std::endl;
namespace mapManager{
	class occMap{
	private:

	protected:
		std::string ns_;
		std::string hint_;

		// ROS
		ros::NodeHandle nh_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> depthPoseSync;
		std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;
		std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> depthOdomSync;
		std::shared_ptr<message_filters::Synchronizer<depthOdomSync>> depthOdomSync_;
		ros::Timer occTimer_;
		ros::Timer inflateTimer_;
		ros::Timer visTimer_;
		ros::Publisher depthCloudPub_;
		ros::Publisher mapVisPub_;
		ros::Publisher inflatedMapVisPub_;

		int localizationMode_;
		std::string depthTopicName_; // depth image topic
		std::string poseTopicName_;  // pose topic
		std::string odomTopicName_; // odom topic 

		// parameters
		// -----------------------------------------------------------------
		// ROBOT SIZE
		Eigen::Vector3d robotSize_;

		// CAMERA
		double fx_, fy_, cx_, cy_; // depth camera intrinsics
		double depthScale_; // value / depthScale
		double depthMinValue_, depthMaxValue_;
		int depthFilterMargin_, skipPixel_; // depth filter margin
		int imgCols_, imgRows_;
		Eigen::Matrix4d body2Cam_; // from body frame to camera frame

		// RAYCASTING
		double raycastMaxLength_;
		double pHitLog_, pMissLog_, pMinLog_, pMaxLog_, pOccLog_; 

		// MAP
		double UNKNOWN_FLAG_ = 0.01;
		double mapRes_;
		double groundHeight_; // ground height in z axis
		Eigen::Vector3d mapSize_, mapSizeMin_, mapSizeMax_; // reserved min/max map size
		Eigen::Vector3i mapVoxelMin_, mapVoxelMax_; // reserved min/max map size in voxel
		Eigen::Vector3d localUpdateRange_; // self defined local update range
		double localBoundInflate_; // inflate local map for some distance
		bool cleanLocalMap_; 

		// VISUALZATION
		double maxVisHeight_;
		Eigen::Vector3d localMapSize_;
		Eigen::Vector3i localMapVoxel_; // voxel representation of local map size
		bool visGlobalMap_;
		bool verbose_;
		// -----------------------------------------------------------------



		// data
		// -----------------------------------------------------------------
		// SENSOR DATA
		cv::Mat depthImage_;
		Eigen::Vector3d position_; // current position
		Eigen::Matrix3d orientation_; // current orientation
		Eigen::Vector3i localBoundMin_, localBoundMax_; // sensor data range


		// MAP DATA
		int projPointsNum_ = 0;
		std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
		std::vector<int> countHitMiss_;
		std::vector<int> countHit_;
		std::queue<Eigen::Vector3i> updateVoxelCache_;
		std::vector<double> occupancy_; // occupancy log data
		std::vector<bool> occupancyInflated_; // inflated occupancy data
		int raycastNum_ = 0; 
		std::vector<int> flagTraverse_, flagRayend_;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions_;
		bool useFreeRegions_ = false;
		

		// STATUS
		bool occNeedUpdate_ = false;
		bool mapNeedInflate_ = false;
		bool esdfNeedUpdate_ = false; // only used in ESDFMap

		// Raycaster
		RayCaster raycaster_;

		// ------------------------------------------------------------------

	public:
		// std::thread visWorker_;

		occMap(); // empty constructor
		occMap(const ros::NodeHandle& nh);
		void initMap(const ros::NodeHandle& nh);
		void initParam();
		void registerCallback();
		void registerPub();

		// callback
		void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
		void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
		void updateOccupancyCB(const ros::TimerEvent& );
		void inflateMapCB(const ros::TimerEvent& );

		// core function
		void projectDepthImage();
		void raycastUpdate();
		void cleanLocalMap();
		void inflateLocalMap();

		// user functions
		bool isOccupied(const Eigen::Vector3d& pos);
		bool isOccupied(const Eigen::Vector3i& idx); // does not count for unknown
		bool isInflatedOccupied(const Eigen::Vector3d& pos);
		bool isInflatedOccupied(const Eigen::Vector3i& idx);
		bool isInflatedOccupiedLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
		bool isFree(const Eigen::Vector3d& pos);
		bool isFree(const Eigen::Vector3i& idx);
		bool isInflatedFree(const Eigen::Vector3d& pos);
		bool isInflatedFreeLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
		bool isUnknown(const Eigen::Vector3d& pos);
		bool isUnknown(const Eigen::Vector3i& idx);
		void setFree(const Eigen::Vector3d& pos);
		void setFree(const Eigen::Vector3i& idx);
		void freeRegion(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
		void updateFreeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
		double getRes();
		void getMapRange(Eigen::Vector3d& mapSizeMin, Eigen::Vector3d& mapSizeMax);
		bool castRay(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxLength=5.0, bool ignoreUnknown=true);

		// Visualziation
		void visCB(const ros::TimerEvent& );
		// void startVisualization();
		void publishProjPoints();
		void publishMap();
		void publishInflatedMap();

		// helper functions
		double logit(double x);
		bool isInMap(const Eigen::Vector3d& pos);
		bool isInMap(const Eigen::Vector3i& idx);
		void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx);
		void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos);
		int posToAddress(const Eigen::Vector3d& idx);
		int posToAddress(double x, double y, double z);
		int indexToAddress(const Eigen::Vector3i& idx);
		int indexToAddress(int x, int y, int z);
		void boundIndex(Eigen::Vector3i& idx);
		bool isInLocalUpdateRange(const Eigen::Vector3d& pos);
		bool isInLocalUpdateRange(const Eigen::Vector3i& idx);
		bool isInFreeRegion(const Eigen::Vector3d& pos, const std::pair<Eigen::Vector3d, Eigen::Vector3d>& freeRegion);
		bool isInFreeRegions(const Eigen::Vector3d& pos);
		Eigen::Vector3d adjustPointInMap(const Eigen::Vector3d& point);
		Eigen::Vector3d adjustPointRayLength(const Eigen::Vector3d& point);
		int updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied);
		void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
		void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix);
	};
	// inline function
	// user function
	inline bool occMap::isOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isOccupied(idx);
	}

	inline bool occMap::isOccupied(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] >= this->pOccLog_;
	}

	inline bool occMap::isInflatedOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInflatedOccupied(idx);
	}

	inline bool occMap::isInflatedOccupied(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancyInflated_[address] == true;
	}

	inline bool occMap::isInflatedOccupiedLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){		
		if (this->isInflatedOccupied(pos1) or this->isInflatedOccupied(pos2)){
			return true;
		}

		Eigen::Vector3d diff = pos2 - pos1;
		double dist = diff.norm();
		Eigen::Vector3d diffUnit = diff/dist;
		int stepNum = int(dist/this->mapRes_);
		Eigen::Vector3d pCheck;
		Eigen::Vector3d unitIncrement = diffUnit * this->mapRes_;
		bool isOccupied = false;
		for (int i=1; i<stepNum; ++i){
			pCheck = pos1 + i * unitIncrement;
			isOccupied = this->isInflatedOccupied(pCheck);
			if (isOccupied){
				return true;
			}
		}
		return false;
	}

	inline bool occMap::isFree(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isFree(idx);
	}

	inline bool occMap::isFree(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return false;
		}
		int address = this->indexToAddress(idx);
		return (this->occupancy_[address] < this->pOccLog_) and (this->occupancy_[address] >= this->pMinLog_);
	}

	inline bool occMap::isInflatedFree(const Eigen::Vector3d& pos){
		if (not this->isInflatedOccupied(pos) and not this->isUnknown(pos)){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool occMap::isInflatedFreeLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){
		if (not this->isInflatedFree(pos1) or not this->isInflatedFree(pos2)){
			return false;
		}

		Eigen::Vector3d diff = pos2 - pos1;
		double dist = diff.norm();
		Eigen::Vector3d diffUnit = diff/dist;
		int stepNum = int(dist/this->mapRes_);
		Eigen::Vector3d pCheck;
		Eigen::Vector3d unitIncrement = diffUnit * this->mapRes_;
		bool isFree = true;
		for (int i=1; i<stepNum; ++i){
			pCheck = pos1 + i * unitIncrement;
			isFree = this->isInflatedFree(pCheck);
			if (not isFree){
				return false;
			}
		}
		return true;
	}


	inline bool occMap::isUnknown(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isUnknown(idx);
	}

	inline bool occMap::isUnknown(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] < this->pMinLog_;		
	}

	inline void occMap::setFree(const Eigen::Vector3d& pos){
		if (not this->isInMap(pos)) return;
		int address = this->posToAddress(pos);
		this->occupancy_[address] = this->pMinLog_;
	}

	inline void occMap::setFree(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)) return;
		int address = this->indexToAddress(idx);
		this->occupancy_[address] = this->pMinLog_;
	}

	inline void occMap::freeRegion(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){
		if (not this->isInMap(pos1) and not this->isInMap(pos2)){
			return;
		}

		Eigen::Vector3i idx1, idx2;
		this->posToIndex(pos1, idx1);
		this->posToIndex(pos2, idx2);
		this->boundIndex(idx1);
		this->boundIndex(idx2);
		for (int xID=idx1(0); xID<=idx2(0); ++xID){
			for (int yID=idx1(1); yID<=idx2(1); ++yID){
				for (int zID=idx1(2); zID<=idx2(2); ++zID){
					this->setFree(Eigen::Vector3d (xID, yID, zID));
				}	
			}
		}
	}

	inline void occMap::updateFreeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		this->freeRegions_ = freeRegions;
		this->useFreeRegions_ = true;
	}


	inline double occMap::getRes(){
		return this->mapRes_;
	}

	inline void occMap::getMapRange(Eigen::Vector3d& mapSizeMin, Eigen::Vector3d& mapSizeMax){
		mapSizeMin = this->mapSizeMin_;
		mapSizeMax = this->mapSizeMax_;
	}

	inline bool occMap::castRay(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxLength, bool ignoreUnknown){
		// return true if raycasting successfully find the endpoint, otherwise return false

		Eigen::Vector3d directionNormalized = direction/direction.norm(); // normalize the direction vector
		int num = ceil(maxLength/this->mapRes_);
		for (int i=1; i<num; ++i){
			Eigen::Vector3d point = this->mapRes_ * directionNormalized * i + start;
			if (ignoreUnknown){
				if (this->isInflatedOccupied(point)){
					end = point;
					return true;
				}
			}
			else{
				if (this->isInflatedOccupied(point) or this->isUnknown(point)){
					end = point;
					return true;
				}	
			}
		}
		end = start;
		return false;
	}
	// end of user functinos

	// helper functions
	inline double occMap::logit(double x){
		return log(x/(1-x));
	}

	inline bool occMap::isInMap(const Eigen::Vector3d& pos){
		if ((pos(0) >= this->mapSizeMin_(0)) and (pos(0) <= this->mapSizeMax_(0)) and 
			(pos(1) >= this->mapSizeMin_(1)) and (pos(1) <= this->mapSizeMax_(1)) and 
			(pos(2) >= this->mapSizeMin_(2)) and (pos(2) <= this->mapSizeMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool occMap::isInMap(const Eigen::Vector3i& idx){
		if ((idx(0) >= this->mapVoxelMin_(0)) and (idx(0) < this->mapVoxelMax_(0)) and
		    (idx(1) >= this->mapVoxelMin_(1)) and (idx(1) < this->mapVoxelMax_(1)) and 
		    (idx(2) >= this->mapVoxelMin_(2)) and (idx(2) < this->mapVoxelMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline void occMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx){
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

	inline void occMap::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	inline int occMap::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	inline int occMap::posToAddress(double x, double y, double z){
		Eigen::Vector3d pos (x, y, z);
		return this->posToAddress(pos);
	}

	inline int occMap::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

	inline int occMap::indexToAddress(int x, int y, int z){
		Eigen::Vector3i idx (x, y, z);
		return this->indexToAddress(idx);
	}

	inline void occMap::boundIndex(Eigen::Vector3i& idx){
		Eigen::Vector3i temp;
		temp(0) = std::max(std::min(idx(0), this->mapVoxelMax_(0)-1), this->mapVoxelMin_(0));
		temp(1) = std::max(std::min(idx(1), this->mapVoxelMax_(1)-1), this->mapVoxelMin_(1));
		temp(2) = std::max(std::min(idx(2), this->mapVoxelMax_(2)-1), this->mapVoxelMin_(2));
		idx = temp;
	}

	inline bool occMap::isInLocalUpdateRange(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInLocalUpdateRange(idx);
	}

	inline bool occMap::isInLocalUpdateRange(const Eigen::Vector3i& idx){
		Eigen::Vector3d rangeMin = this->position_ - this->localUpdateRange_;
		Eigen::Vector3d rangeMax = this->position_ + this->localUpdateRange_;
		
		Eigen::Vector3i rangeMinIdx, rangeMaxIdx;
		this->posToIndex(rangeMin, rangeMinIdx);
		this->posToIndex(rangeMax, rangeMaxIdx);

		this->boundIndex(rangeMinIdx);
		this->boundIndex(rangeMaxIdx);

		bool inRange = (idx(0) >= rangeMinIdx(0)) and (idx(0) <= rangeMaxIdx(0)) and
					   (idx(1) >= rangeMinIdx(1)) and (idx(1) <= rangeMaxIdx(1)) and
					   (idx(2) >= rangeMinIdx(2)) and (idx(2) <= rangeMaxIdx(2));
		return inRange;
	}

	inline bool occMap::isInFreeRegion(const Eigen::Vector3d& pos, const std::pair<Eigen::Vector3d, Eigen::Vector3d>& freeRegion){
		Eigen::Vector3d lowerBound = freeRegion.first;
		Eigen::Vector3d upperBound = freeRegion.second;
		if (pos(0) >= lowerBound(0) and pos(0) <= upperBound(0) and
			pos(1) >= lowerBound(1) and pos(1) <= upperBound(1) and
			pos(2) >= lowerBound(2) and pos(2) <= upperBound(2)){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool occMap::isInFreeRegions(const Eigen::Vector3d& pos){
		for (std::pair<Eigen::Vector3d, Eigen::Vector3d> freeRegion : this->freeRegions_){
			if (this->isInFreeRegion(pos, freeRegion)){
				return true;
			}
		}
		return false;
	}


	inline Eigen::Vector3d occMap::adjustPointInMap(const Eigen::Vector3d& point){
		Eigen::Vector3d pos = this->position_;
		Eigen::Vector3d diff = point - pos;
		Eigen::Vector3d offsetMin = this->mapSizeMin_ - pos;
		Eigen::Vector3d offsetMax = this->mapSizeMax_ - pos;

		double minRatio = 10000000;
		for (int i=0; i<3; ++i){ // each axis
			if (diff[i] != 0){
				double ratio1 = offsetMin[i]/diff[i];
				double ratio2 = offsetMax[i]/diff[i];
				if ((ratio1 > 0) and (ratio1 < minRatio)){
					minRatio = ratio1;
				}

				if ((ratio2 > 0) and (ratio2 < minRatio)){
					minRatio = ratio2;
				}
			}
		}

		return pos + (minRatio - 1e-3) * diff;
	}


	inline Eigen::Vector3d occMap::adjustPointRayLength(const Eigen::Vector3d& point){
		double length = (point - this->position_).norm();
		return (point - this->position_) * (this->raycastMaxLength_/length) + this->position_;
	}

	inline int occMap::updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied){
		Eigen::Vector3i idx;
		this->posToIndex(point, idx);
		int voxelID = this->indexToAddress(idx);
		this->countHitMiss_[voxelID] += 1;
		if (this->countHitMiss_[voxelID] == 1){
			this->updateVoxelCache_.push(idx);
		}
		if (isOccupied){ // if not adjusted set it to occupied, otherwise it is free
			this->countHit_[voxelID] += 1;
		}
		return voxelID;
	}

	inline void occMap::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		// convert body pose to camera pose
		Eigen::Matrix4d map2body; map2body.setZero();
		map2body.block<3, 3>(0, 0) = rot;
		map2body(0, 3) = pose->pose.position.x; 
		map2body(1, 3) = pose->pose.position.y;
		map2body(2, 3) = pose->pose.position.z;
		map2body(3, 3) = 1.0;

		camPoseMatrix = map2body * this->body2Cam_;
	}

	inline void occMap::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		// convert body pose to camera pose
		Eigen::Matrix4d map2body; map2body.setZero();
		map2body.block<3, 3>(0, 0) = rot;
		map2body(0, 3) = odom->pose.pose.position.x; 
		map2body(1, 3) = odom->pose.pose.position.y;
		map2body(2, 3) = odom->pose.pose.position.z;
		map2body(3, 3) = 1.0;

		camPoseMatrix = map2body * this->body2Cam_;
	}
}

#endif