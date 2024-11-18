#include "Raven_Controller.h"

/**
*   \fn Raven_Controller()
*
*   \brief this is the constructor
*
*   \param void
*
*   \return none
*/
Raven_Controller::Raven_Controller()
{

}

/**
*   \fn initial()
*
*   \brief initialize everything for this program.
*
*   \param int argc, char** argv
*
*   \return void
*/
void Raven_Controller::initial(int argc, char** argv)
{
    init_sys();
    if (!init_ros(argc, argv))
    {
        ROS_ERROR("Fail to initialize ROS. Exiting!");
        exit(1);
    }

    init_pathplanner();
    init_words();
}

/**
*   \fn void init_sys()
*
*   \brief initialize default system parameter settings.
*
*   \param void
*
*   \return void
*/
void Raven_Controller::init_sys()
{
    this->SPEED = 1;
    this->PUB_COUNT = 0;
    this->SUB_COUNT = 0;
    this->RECEIVED_FIRST = false;
    this->SHOW_STATUS = false;
    this->PAUSE = false;
}

/**
*   \fn int init_ros(int argc, char** argv)
*
*   \brief initialize ROS and connect the pub & sub relation.
*
*   \param int argc, char** argv
*
*   \return bool
*/
bool Raven_Controller::init_ros(int argc, char** argv)
{
    ros::init(argc, argv, "PointToPoint_Controller");

    static ros::NodeHandle n;
    RavenAutomove_publisher = n.advertise<raven_automove>("raven_automove", 1);
    RavenState_subscriber = n.subscribe("ravenstate", 1, &Raven_Controller::callback_raven_state, this);

    return true;
}


void Raven_Controller::callback_raven_state(raven_2::raven_state msg) 
{
	// (1) save the updated raven_state 
	CURR_RAVEN_STATE.runlevel = msg.runlevel;
	CURR_RAVEN_STATE.sublevel = msg.sublevel;
	CURR_RAVEN_STATE.last_seq = msg.last_seq;
	CURR_RAVEN_STATE.dt = msg.dt;

	for(int i=0; i<2; i++)
	{
		CURR_RAVEN_STATE.type[i] = msg.type[i];
		CURR_RAVEN_STATE.grasp_d[i] = msg.grasp_d[i];
	}

	for(int i=0; i<6; i++)
	{
		CURR_RAVEN_STATE.pos[i] = msg.pos[i];
		CURR_RAVEN_STATE.pos_d[i] = msg.pos_d[i];
	}
	
	for(int i=0; i<18; i++)
	{
		CURR_RAVEN_STATE.ori[i] = msg.ori[i];
		CURR_RAVEN_STATE.ori_d[i] = msg.ori_d[i];
	}

	// (2) update recieved data count
	SUB_COUNT ++;
	RECEIVED_FIRST = true;
}



void Raven_Controller::publish_raven_automove()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_automove msg_raven_automove;	

	// (1) wrap up the new command	
	msg_raven_automove.hdr.stamp = msg_raven_automove.hdr.stamp.now(); //hdr

	tf::transformTFToMsg(TF_INCR[LEFT_ARM], msg_raven_automove.tf_incr[LEFT_ARM]);   //tf_incr
	tf::transformTFToMsg(TF_INCR[RIGHT_ARM], msg_raven_automove.tf_incr[RIGHT_ARM]);

	// (2) send new command
	RavenAutomove_publisher.publish(msg_raven_automove);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	PUB_COUNT ++;
}






int Raven_Controller::getKey() 
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    // 保存原始终端设置
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));

    // 设置终端为非阻塞模式（无回显，立即读取）
    new_term_attr.c_lflag &= ~(ECHO | ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    // 从 stdin 读取字符，非阻塞
    character = fgetc(stdin);

    // 恢复原始终端设置
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}




/**
*   \fn void init_pathplanner()
*
*   \brief initialize objects from Raven_PathPlanner class.
*
*   \param void
*
*   \return void
*/
void Raven_Controller::init_pathplanner()
{
    if (!LEFT_PATH.set_ArmType(LEFT_ARM) || !RIGHT_PATH.set_ArmType(RIGHT_ARM))
    {
        ROS_ERROR("Fail to set RAVEN arm type. Exiting!");
        exit(1);
    }

    if (!LEFT_PATH.set_Speed(SPEED) || !RIGHT_PATH.set_Speed(SPEED))
    {
        ROS_ERROR("Fail to set movement speed. Exiting!");
        exit(1);
    }
}

/**
*   \fn void init_words()
*
*   \brief show greeting words on console.
*
*   \param void
*
*   \return void
*/
void Raven_Controller::init_words()
{
    string start = "0";
    do
    {
        cout << endl << endl;
        cout << "Welcome to the Point-to-Point Controller for RAVEN2" << endl << endl;
        cout << "Default settings: SPEED = " << SPEED << endl;
        cout << "Please press \"Enter\" to start!";
        cin.clear();
        getline(std::cin, start);
    } while (start != "");

    cout << "Point-to-Point Controller starting..." << endl;
}

/**
*   \fn bool menu_words()
*
*   \brief show menu words on console.
*
*   \param void
*
*   \return void
*/
bool Raven_Controller::menu_words(bool print_menu)
{
    if (print_menu)
    {
        cout << endl;
        cout << "Point-to-Point Controller Selection Menu:" << endl;
        cout << "----------------------------------------------------" << endl;
        cout << "\t'1' : Increase Raven Moving Speed" << endl;
        cout << "\t'2' : Decrease Raven Moving Speed" << endl;
        cout << "\t'3' : Set Target Position" << endl;
        cout << "\t'4' : Toggle pause/resume" << endl;
        cout << "\t'5' : Toggle console messages" << endl;
        cout << "\t'^C': Quit" << endl << endl;
    }
    return false;
}

/**
*   \fn void final_words()
*
*   \brief show goodbye words on console.
*
*   \param void
*
*   \return void
*/
void Raven_Controller::final_words()
{
    cout << "Terminating the Point-to-Point Controller." << endl;
    cout << "----------------------------------------------------" << endl;
    cout << "GoodBye!" << endl << endl << endl;
}

/**
*   \fn void start_thread()
*
*   \brief start the console_thread and ros_thread
*
*   \param void
*
*   \return void
*/
void Raven_Controller::start_thread()
{
    pthread_create(&console_thread, NULL, Raven_Controller::static_console_process, this);
    pthread_create(&ros_thread, NULL, Raven_Controller::static_ros_process, this);
}

/**
*   \fn void join_thread()
*
*   \brief join the console_thread and ros_thread
*
*   \param void
*
*   \return void
*/
void Raven_Controller::join_thread()
{
    pthread_join(console_thread, NULL);
    pthread_join(ros_thread, NULL);

    final_words();
}

/**
*   \fn void *console_process(void)
*
*   \brief this thread is dedicated to console io
*
*   \param a pointer to void
*
*   \return void
*/
void* Raven_Controller::console_process(void)
{
    if (ros::isInitialized())
    {
        int theKey;
        bool print_menu = true;

        while (ros::ok())
        {
            print_menu = menu_words(print_menu);
            theKey = getKey();

            switch (theKey)
            {
                case '1':
                {
                    SPEED = (SPEED + 1 > MAX_SPEED) ? MAX_SPEED : SPEED + 1;
                    LEFT_PATH.set_Speed(SPEED);
                    RIGHT_PATH.set_Speed(SPEED);
                    cout << "You chose 1 : Increase Raven Moving Speed." << endl;
                    cout << "\tnew SPEED = " << SPEED << endl;
                    print_menu = true;
                    break;
                }
                case '2':
                {
                    SPEED = (SPEED - 1 < MIN_SPEED) ? MIN_SPEED : SPEED - 1;
                    LEFT_PATH.set_Speed(SPEED);
                    RIGHT_PATH.set_Speed(SPEED);
                    cout << "You chose 2 : Decrease Raven Moving Speed." << endl;
                    cout << "\tnew SPEED = " << SPEED << endl;
                    print_menu = true;
                    break;
                }
                case '3':
                {
                    // 定义一系列目标点
                    std::vector<tf::Vector3> target_positions = {

			
                        tf::Vector3(-87087, -26328, -6542),
			tf::Vector3(-89512, -26334, -5116),
			tf::Vector3(-91247, -26378, -3786),
                        tf::Vector3(-93024, -26408, -2122),
			tf::Vector3(-93994, -26421, -1015),
			tf::Vector3(-94404, -26474, -516),
			tf::Vector3(-94913, -26399, 137),
			tf::Vector3(-95466, -26419, 850),
			tf::Vector3(-96005, -26407, 1810),
			tf::Vector3(-96656, -26424, 2936),
			tf::Vector3(-96990, -26430, 3508),
			tf::Vector3(-97154, -26438, 3863),
			tf::Vector3(-97491, -26423, 4549),
			tf::Vector3(-97699, -26457, 5195),
			tf::Vector3(-97919, -26418, 5798),
			tf::Vector3(-98371, -26442, 6795),
			tf::Vector3(-98612, -26451, 7665),
			tf::Vector3(-99108, -26468, 8811),
			tf::Vector3(-99220, -26482, 9302),
			tf::Vector3(-99292, -26483, 9567),
			tf::Vector3(-99499, -26448, 10216),
			tf::Vector3(-99922, -26466, 11146),
			tf::Vector3(-100205, -26412, 12125),
			tf::Vector3(-100729, -26391, 13884),
			tf::Vector3(-100764, -26386, 14030),
                        
                        
                    };

                    // 循环发送每个目标点，间隔1秒
                    for (const auto& target_position : target_positions) {
                        LEFT_PATH.set_Target_Pos(target_position);
                        // RIGHT_PATH.set_Target_Pos(target_position); // 如果需要，可以设置右路径

                        cout << "Set Target Position to (" 
                             << target_position.getX() << ", " 
                             << target_position.getY() << ", " 
                             << target_position.getZ() << ")" << endl;

                        std::this_thread::sleep_for(std::chrono::seconds(1)); // 间隔1秒
                    }
                    
                    print_menu = true;
                    break;
                }
                case '4':
                {
                    PAUSE = !PAUSE;
                    cout << "You chose 4 : Toggle pause/resume." << endl;
                    cout << (PAUSE ? "\tMovement is paused." : "\tMovement is resumed.") << endl;
                    print_menu = true;
                    break;
                }
                case '5':
                {
                    SHOW_STATUS = !SHOW_STATUS;
                    cout << "You chose 5 : Toggle console messages." << endl;
                    cout << (SHOW_STATUS ? "\tConsole message turned on." : "\tConsole message turned off.") << endl;
                    print_menu = true;
                    break;
                }
            }
        }
        cout << "console_process is shutdown." << endl;
    }
    return 0;
}

/**
*   \fn void *ros_process(void)
*
*   \brief this thread is dedicated to ros pub & sub
*
*   \param a pointer to void
*
*   \return void
*/
void* Raven_Controller::ros_process(void)
{
    if (ros::isInitialized())
    {
        if (!RECEIVED_FIRST)
            cout << endl << "Waiting for the first receive of raven_state..." << endl;
        else
        {
            cout << "First raven_state received." << endl;
            cout << "Save as default center position." << endl;
        }

        while (ros::ok() && RECEIVED_FIRST)
        {
            LEFT_PATH.set_Current_Pos(CURR_RAVEN_STATE.pos);
            LEFT_PATH.set_Current_Ori(CURR_RAVEN_STATE.ori);

            RIGHT_PATH.set_Current_Pos(CURR_RAVEN_STATE.pos);
            RIGHT_PATH.set_Current_Ori(CURR_RAVEN_STATE.ori);

            if (PAUSE)
            {
                TF_INCR[LEFT_ARM] = LEFT_PATH.ComputeNullTrajectory();
                TF_INCR[RIGHT_ARM] = RIGHT_PATH.ComputeNullTrajectory();
            }
            else
            {
                TF_INCR[LEFT_ARM] = LEFT_PATH.ComputePointToPointTrajectory();
                TF_INCR[RIGHT_ARM] = RIGHT_PATH.ComputePointToPointTrajectory();
            }

            publish_raven_automove();
        }

        if (RECEIVED_FIRST)
            cout << "ros_process is shutdown." << endl;
    }
    return nullptr; // 注意返回nullptr
}


void * Raven_Controller::static_console_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->console_process();
}



void * Raven_Controller::static_ros_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->ros_process();
}








