#ifndef RAVEN_CONTROLLER_H_
#define RAVEN_CONTROLLER_H_

#include "Raven_PathPlanner.h"

class Raven_Controller
{
	private:
		int SPEED;               // Movement speed
		int PUB_COUNT;           // Publish count for status
		int SUB_COUNT;           // Subscribe count for status
		bool SHOW_STATUS;        // Flag to toggle status output
		bool RECEIVED_FIRST;     // Flag to check first message reception
		bool PAUSE;              // Pause flag

		pthread_t console_thread; // Thread for console I/O
		pthread_t ros_thread;     // Thread for ROS pub/sub

		ros::Publisher RavenAutomove_publisher;   // ROS publisher
		ros::Subscriber RavenState_subscriber;    // ROS subscriber

		raven_state CURR_RAVEN_STATE;             // Current state of Raven
		tf::Transform TF_INCR[2];                 // Transform increments for both arms
		
		Raven_PathPlanner LEFT_PATH;              // Path planner for left arm
		Raven_PathPlanner RIGHT_PATH;             // Path planner for right arm

	public:
		Raven_Controller();                       // Constructor
		void initial(int, char**);                // Initialization and console display
		void init_sys();
		bool init_ros(int, char**);
		void init_pathplanner();
		void init_words();
		bool menu_words(bool);
		void final_words();

		void start_thread();                      // Thread management
		void join_thread();
		void *console_process(void);
		void *ros_process(void);
		static void *static_console_process(void*);
		static void *static_ros_process(void*);

		void publish_raven_automove();            // ROS publish function
		void callback_raven_state(raven_2::raven_state); // ROS subscribe callback

		void output_STATUS();                     // Display ROS and Raven state
		void output_PATHinfo();
		void output_PUBinfo();
		void output_SUBinfo();

		int getKey();                             // Function to get keyboard input

}; // End of class definition

#endif

