#include <ros/ros.h>
#include <communicationISLH/inMessage.h>
#include <communicationISLH/outMessage.h>
#include <taskHandlerISLH/taskInfo2LeaderMessage.h>
#include <messageDecoderISLH/cmdFromLeaderMessage.h>
#include <messageDecoderISLH/cmdFromCoordinatorMessage.h>
#include <coalitionLeaderISLH/cmd2RobotsFromLeaderMessage.h>
#include <messageDecoderISLH/taskInfoFromLeaderMessage.h>
#include <taskCoordinatorISLH/cmd2LeadersMessage.h>
#include <messageDecoderISLH/taskInfoFromRobotMessage.h>
#include <coalitionLeaderISLH/taskInfo2CoordinatorMessage.h>
#include <messageDecoderISLH/newLeaderMessage.h>
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
    INFO_L2C_TASK_COMPLETED = 3,
    INFO_L2C_SPLITTING = 4,
    INFO_L2C_LEADER_CHANGED = 5
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

     void handleIncomingMessage(communicationISLH::inMessage msg);

     void pubTaskInfoFromRobot(communicationISLH::inMessage msg);

     void pubCmdFromLeader(communicationISLH::inMessage msg);

     void pubTaskInfoFromLeader(communicationISLH::inMessage msg);

     void pubCmdFromCoordinator(communicationISLH::inMessage msg);

     //void sendNewLeaderInfoFromOldLeader(communicationISLH::inMessage msg);

     void sendTaskInfo2Leader(taskHandlerISLH::taskInfo2LeaderMessage taskInfoMsg);

     void sendCmd2Robots(coalitionLeaderISLH::cmd2RobotsFromLeaderMessage msg);

     void sendCmd2Leaders(taskCoordinatorISLH::cmd2LeadersMessage msg);

     void sendTaskInfo2Coordinator(coalitionLeaderISLH::taskInfo2CoordinatorMessage taskInfoMsg);

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
