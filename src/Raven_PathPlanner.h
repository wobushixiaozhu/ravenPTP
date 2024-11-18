#ifndef RAVEN_PATHPLANNER_H_
#define RAVEN_PATHPLANNER_H_

#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/ros.h>
#include <pthread.h>
#include <queue>
#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"
#include <cstdio>
#include <cstring>
#include <termios.h>
#include <thread>

#define LEFT_ARM 0
#define RIGHT_ARM 1

#define MAX_SPEED 60   // 60 different speed levels
#define MIN_SPEED 1

#define DEL_POS_THRESHOLD 180	// in micro meter (= 0.18mm = 0.018cm)
#define ROS_PUBLISH_RATE 1000 	// in Hz

#define DEFAULT_MODIFICATION_SCALE  	  	0.000001
#define DEFAULT_MODIFICATION_SPEED_POWER  	0.7
#define DEFAULT_MODIFICATION_DISTANCE_POWER  	1

#define POSITION_THRESHOLD 100 // 新增用于点对点运动的阈值


using namespace raven_2;
using namespace std;

class Raven_PathPlanner
{
	private:
		tfScalar Modi_Scale;
		tfScalar Modi_Speed_Pow;
		tfScalar Modi_Dista_Pow;
		
		tf::Vector3 X_AXIS;
		tf::Vector3 Y_AXIS;
		tf::Vector3 Z_AXIS;

		tf::Vector3 Target_Pos;        // target position for point-to-point movement
		tf::Vector3 Current_Pos;       // current raven position
		tf::Vector3 Delta_Pos;         // movement delta
		tf::Quaternion Current_Ori;    // current raven rotation

		tfScalar Speed;
		tfScalar Distance;             // distance between current pos and target pos
		
		pthread_mutexattr_t data1MutexAttr;
		pthread_mutex_t data1Mutex;

		int ArmType;
		bool FIRST_SEND;
		tfScalar Kp;

	public:
		Raven_PathPlanner();
		void set_Target_Pos(tf::Vector3);     // sets target position
		bool set_Speed(int);
		bool set_ArmType(int);
		bool set_Current_Pos(boost::array<int, 6>);
		bool set_Current_Ori(boost::array<float, 18>);
		tfScalar get_Speed();
		void show_Target_Pos();               // shows the target position
		tfScalar DistanceOf(tf::Vector3,tf::Vector3);

		// Trajectory functions
		tf::Transform ComputePointToPointTrajectory(); // computes point-to-point trajectory
		tf::Transform ComputeNullTrajectory();         // computes stationary trajectory
};

#endif

