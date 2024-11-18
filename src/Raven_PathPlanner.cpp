#include "Raven_PathPlanner.h"

/**
*   \fn Raven_PathPlanner()
*
*   \brief this is the constructor
*
*   \param void
*
*   \return none
*/
Raven_PathPlanner::Raven_PathPlanner()
{
    Modi_Scale =     DEFAULT_MODIFICATION_SCALE;
    Modi_Speed_Pow = DEFAULT_MODIFICATION_SPEED_POWER;
    Modi_Dista_Pow = DEFAULT_MODIFICATION_DISTANCE_POWER;

    X_AXIS.setValue(1,0,0);
    Y_AXIS.setValue(0,1,0);
    Z_AXIS.setValue(0,0,1);

    pthread_mutexattr_init(&data1MutexAttr);
    pthread_mutexattr_setprotocol(&data1MutexAttr,PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&data1Mutex,&data1MutexAttr);
}

/**
*   \fn void set_Target_Pos(tf::Vector3 target)
*
*   \brief 设置目标位置
*
*   \param tf::Vector3
*
*   \return void
*/
void Raven_PathPlanner::set_Target_Pos(tf::Vector3 target)
{
    Target_Pos = target;
}

/**
*   \fn bool set_Current_Pos(boost::array<int, 6> currpos)
*
*   \brief stores the current position received from raven_state
*
*   \param boost::array<int, 6>
*
*   \return bool
*/
bool Raven_PathPlanner::set_Current_Pos(boost::array<int, 6> currpos)
{
    tfScalar X,Y,Z;

    if(ArmType == LEFT_ARM)
    {
        X = currpos[0];
        Y = currpos[1];
        Z = currpos[2];
    }
    else if(ArmType == RIGHT_ARM)
    {
        X = currpos[3];
        Y = currpos[4];
        Z = currpos[5];
    }
    else
    {
        return false;
    }

    Current_Pos.setValue(X,Y,Z);
    return true;
}








/**
*   \fn tf::Transform ComputePointToPointTrajectory()
*
*   \brief this function generates delta motion commands for RAVEN to move toward a target point
*
*   \param void
*
*   \return tf::Transform
*/
tf::Transform Raven_PathPlanner::ComputePointToPointTrajectory()
{


    tf::Transform TF_INCR;

    pthread_mutex_lock(&data1Mutex);

    // (1) Calculate delta vector from current position to target position
    tf::Vector3 Delta_Vector = Target_Pos - Current_Pos;

    //std::cout << Target_Pos << std::endl;

    //std::cout << Current_Pos << std::endl;

    // (2) Check if the distance to the target is greater than a small threshold
    if (Delta_Vector.length() > POSITION_THRESHOLD) {
        // Normalize and scale delta vector by Speed
        Delta_Vector = Delta_Vector.normalized() * Speed;
    } else {
        // Close enough to the target; stop moving
        Delta_Vector.setValue(0, 0, 0);
    }

    // Set position increment
    Delta_Pos = Delta_Vector;

    // (3) Set rotation increment to identity (no rotation change)
    tfScalar W = 1;
    tfScalar QX = 0;
    tfScalar QY = 0;
    tfScalar QZ = 0;
    tf::Quaternion Delta_Ori(QX, QY, QZ, W);

    // (4) Add increment to return variable
    TF_INCR.setOrigin(Delta_Pos);
    TF_INCR.setRotation(Delta_Ori);
    
    pthread_mutex_unlock(&data1Mutex);

    return TF_INCR;
}


bool Raven_PathPlanner::set_ArmType(int armtype) {
    ArmType = armtype;
    return true; // 简单实现
}

bool Raven_PathPlanner::set_Speed(int speed) {
    Speed = speed;
    return true; // 简单实现
}

bool Raven_PathPlanner::set_Current_Ori(boost::array<float, 18> ori) {
    // 存储当前的 orientation
    return true; // 简单实现
}



/**
*   \fn tf::Transform ComputeNullTrajectory()
*
*   \brief this function generates commands to let RAVEN stay stationary
*
*   \param void
*
*   \return tf::Transform
*/
tf::Transform Raven_PathPlanner::ComputeNullTrajectory()
{
    tf::Transform TF_INCR;

    tfScalar X = 0;
    tfScalar Y = 0;
    tfScalar Z = 0;
    Delta_Pos.setValue(X,Y,Z); 

    tfScalar W = 1;
    tfScalar QX = 0;
    tfScalar QY = 0;
    tfScalar QZ = 0;
    tf::Quaternion q_temp(QX,QY,QZ,W);

    TF_INCR.setOrigin(Delta_Pos);
    TF_INCR.setRotation(q_temp);

    return TF_INCR;
}

/**
*   \fn void show_Target_Pos()
*
*   \brief displays Target position value to console
*
*   \param void
*
*   \return void
*/
void Raven_PathPlanner::show_Target_Pos()
{
    tfScalar x = Target_Pos.getX();
    tfScalar y = Target_Pos.getY();
    tfScalar z = Target_Pos.getZ();

    if(ArmType == LEFT_ARM)
    {
        cout << "\tTarget Position [LEFT] = (" << x << "," << y << "," << z << ")" << endl;
    }
    else if(ArmType == RIGHT_ARM)
    {
        cout << "\tTarget Position [RIGHT] = (" << x << "," << y << "," << z << ")" << endl;
    }
}

/**
*   \fn tfScalar DistanceOf(tf::Vector3 point1, tf::Vector3 point2)
*
*   \brief computes the distance between two tf::Vector3 points
*
*   \param tf::Vector3, tf::Vector3
*
*   \return tfScalar
*/
tfScalar Raven_PathPlanner::DistanceOf(tf::Vector3 point1, tf::Vector3 point2)
{
    tfScalar DX = point1.getX() - point2.getX();
    tfScalar DY = point1.getY() - point2.getY();
    tfScalar DZ = point1.getZ() - point2.getZ();

    return sqrt(DX*DX + DY*DY + DZ*DZ);
}

