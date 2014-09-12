#include <ros/ros.h>
#include <ISLH_msgs/inMessage.h>
#include <ISLH_msgs/outMessage.h>
#include <ISLH_msgs/taskInfo2LeaderMessage.h>
#include <ISLH_msgs/cmdFromLeaderMessage.h>
#include <ISLH_msgs/cmdFromCoordinatorMessage.h>
#include <ISLH_msgs/cmd2RobotsFromLeaderMessage.h>
#include <ISLH_msgs/taskInfoFromLeaderMessage.h>
#include <ISLH_msgs/cmd2LeadersMessage.h>
#include <ISLH_msgs/taskInfoFromRobotMessage.h>
#include <ISLH_msgs/taskInfo2CoordinatorMessage.h>
#include <ISLH_msgs/newLeaderMessage.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>


enum RawMessageType
{
    MT_TASK_INFO_FROM_ROBOT_TO_LEADER = 1,
    MT_TASK_INFO_FROM_LEADER_TO_ROBOT = 2,
    MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR = 3,
    MT_TASK_INFO_FROM_COORDINATOR_TO_LEADER = 4//,
    //MT_TASK_INFO_FROM_OLDLEADER_TO_NEWLEADER = 5
};


enum Robot2LeaderInfoMgs
{
    INFO_R2L_NEW_TASK_INFO = 1,
    INFO_R2L_REACHED_TO_TASK = 2,
    INFO_R2L_REACHED_TO_GOAL = 3
};

enum Leader2RobotCmdMsgs
{
    CMD_L2R_START_HANDLING = 1,
    CMD_L2R_MOVE_TO_TASK_SITE = 2,
    CMD_L2R_MOVE_TO_GOAL_POSE = 3,
    CMD_L2R_SPLIT_FROM_COALITION = 4,
    CMD_L2R_LEADER_CHANGED = 5
};

enum Leader2CoordinatorInfoMgs
{
    INFO_L2C_INSUFFICIENT_RESOURCE = 1,
    INFO_L2C_START_HANDLING = 2,
    INFO_L2C_START_HANDLING_WITH_TASK_INFO = 3,
    INFO_L2C_TASK_COMPLETED = 4,
    INFO_L2C_SPLITTING = 5,
    INFO_L2C_SPLITTING_AND_LEADER_CHANGED = 6,
    INFO_L2C_WAITING_GOAL_POSE = 7,
    INFO_L2C_WAITING_TASK_SITE_POSE = 8
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

private:
     bool shutdown;

     ros::NodeHandle n;


     // Get incoming messages from communicationISLH
     ros::Subscriber messageInSub;

    // Send outgoing messages to communicationISLH
     ros::Publisher messageOutPub;


     //Get task info from taskHandlerISLH node
     ros::Subscriber messageTaskInfo2LeaderSub;

     // Send taskHandlerISLH the command received from the leader
     ros::Publisher messageCmdFromLeaderPub;

     // Send coalitionLeaderISLH the command received from the task coordinator
     ros::Publisher messageCmdFromCoordinatorPub;

     // Send taskCoordinatorISLH the task info message received from the leader
     ros::Publisher messageTaskInfoFromLeaderPub;

     // Send coalitionLeaderISH the new leader info message received from the old leader
     ros::Publisher messageNewLeaderPub;

     // Send command to robots in the coalition
     ros::Subscriber messageCmd2RobotsSub;

     // Send the coalition leaders the command message from the task coordinator
     ros::Subscriber messageCmd2LeadersSub;

     // Send coalitionLeaderISLH the task info message received from a member robot
     ros::Publisher messageTaskInfoFromRobotPub;

     ros::Subscriber mesageTaskInfo2CoordinatorSub;

     void handleIncomingMessage(ISLH_msgs::inMessage msg);

     void pubTaskInfoFromRobot(ISLH_msgs::inMessage msg);

     void pubCmdFromLeader(ISLH_msgs::inMessage msg);

     void pubTaskInfoFromLeader(ISLH_msgs::inMessage msg);

     void pubCmdFromCoordinator(ISLH_msgs::inMessage msg);

     //void sendNewLeaderInfoFromOldLeader(communicationISLH::inMessage msg);

     void sendTaskInfo2Leader(ISLH_msgs::taskInfo2LeaderMessage taskInfoMsg);

     void sendCmd2Robots(ISLH_msgs::cmd2RobotsFromLeaderMessage msg);

     void sendCmd2Leaders(ISLH_msgs::cmd2LeadersMessage msg);

     void sendTaskInfo2Coordinator(ISLH_msgs::taskInfo2CoordinatorMessage taskInfoMsg);

     std::string makeDataPackage(int messageType, int messageSubType, QString data);

     bool readConfigFile(QString filename);

     int ownRobotID;

     int leaderRobotID;


public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
