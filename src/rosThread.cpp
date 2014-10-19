#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>



RosThread::RosThread()
{
    shutdown = false;
}



void RosThread::work(){

    QString pathConf = QDir::homePath();
    pathConf.append("/ISL_workspace/src/configISL.json");

    if(!readConfigFile(pathConf)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();

        return;
    }



    if(!ros::ok()){

        emit this->rosFinished();

        return;
    }

    emit rosStarted();

    ros::Rate loop(10);

    messageInSub = n.subscribe("communicationISLH/messageIn",5,&RosThread::handleIncomingMessage,this);
    messageOutPub = n.advertise<ISLH_msgs::outMessage>("messageDecoderISLH/messageOut",queueSize);


    messageCmdFromCoordinatorPub= n.advertise<ISLH_msgs::cmdFromCoordinatorMessage>("messageDecoderISLH/cmdFromCoordinator",queueSize);

    messageCmdFromLeaderPub = n.advertise<ISLH_msgs::cmdFromLeaderMessage>("messageDecoderISLH/cmdFromLeader", queueSize);

    messageTaskInfoFromLeaderPub = n.advertise<ISLH_msgs::taskInfoFromLeaderMessage>("messageDecoderISLH/taskInfoFromLeader", queueSize);

    messageTaskInfoFromRobotPub = n.advertise<ISLH_msgs::taskInfoFromRobotMessage>("messageDecoderISLH/taskInfoFromRobot", queueSize);

    messageNewLeaderPub = n.advertise<ISLH_msgs::newLeaderMessage>("messageDecoderISLH/newLeader", queueSize);

    messageTargetPoseListPub = n.advertise<ISLH_msgs::targetPoseListMessage>("messageDecoderISLH/targetPoseList", queueSize);



    messageTaskInfo2LeaderSub = n.subscribe("taskHandlerISLH/taskInfo2Leader", queueSize, &RosThread::sendTaskInfo2Leader,this);

    messageCmd2RobotsSub = n.subscribe("coalitionLeaderISLH/cmd2Robots", queueSize, &RosThread::sendCmd2Robots,this);

    messageCmd2LeadersSub = n.subscribe("taskCoordinatorISLH/cmd2Leaders", queueSize, &RosThread::sendCmd2Leaders,this);

    mesageTaskInfo2CoordinatorSub = n.subscribe("coalitionLeaderISLH/taskInfo2Coordinator", queueSize, &RosThread::sendTaskInfo2Coordinator, this);



    while(ros::ok())
    {


        ros::spinOnce();

        loop.sleep();

    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}

/*
 handle the received message from another robot
*/
void RosThread::handleIncomingMessage(ISLH_msgs::inMessage msg)
{
    QString package = QString::fromStdString(msg.message);

    QStringList packageParts = package.split("*",QString::SkipEmptyParts);

    int messageType = packageParts.at(1).toInt();

    qDebug()<<"InComing Message->  MessageType: "<<messageType<<" Message: "<<package;

    switch(messageType)
    {
    case MT_TASK_INFO_FROM_LEADER_TO_ROBOT:
        pubCmdFromLeader(msg);
        break;
    case MT_TASK_INFO_FROM_ROBOT_TO_LEADER:
        pubTaskInfoFromRobot(msg);
        break;
    case MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR:
        pubTaskInfoFromLeader(msg);
        break;
    case MT_TASK_INFO_FROM_COORDINATOR_TO_LEADER:
        pubCmdFromCoordinator(msg);
        break;
/*    case MT_TASK_INFO_FROM_OLDLEADER_TO_NEWLEADER:
        sendNewLeaderInfoFromOldLeader(msg);
        break;
*/    default:
        break;
    }

}


/*
 Incoming task info message from the coalition member robot to its leader
*/
void RosThread::pubTaskInfoFromRobot(ISLH_msgs::inMessage msg)
{
    ISLH_msgs::taskInfoFromRobotMessage taskInfoMsg;

    QString package = QString::fromStdString(msg.message);

    QStringList packageParts = package.split("*",QString::SkipEmptyParts);

    int infoMessageSubType = packageParts.at(2).toInt();

    QStringList dataParts = packageParts.at(4).split(";",QString::SkipEmptyParts);

    if (infoMessageSubType == INFO_R2L_NEW_TASK_INFO)
    {
        taskInfoMsg.infoMessageType = infoMessageSubType;
        taskInfoMsg.senderRobotID = dataParts.at(0).toInt();
        taskInfoMsg.taskUUID = dataParts.at(1).toStdString();
        taskInfoMsg.encounteringTime = dataParts.at(2).toUInt();
        taskInfoMsg.handlingDuration = dataParts.at(3).toUInt();
        taskInfoMsg.timeOutDuration =  dataParts.at(4).toUInt();
        taskInfoMsg.requiredResources = dataParts.at(5).toStdString();
        taskInfoMsg.posX = dataParts.at(6).toDouble();
        taskInfoMsg.posY = dataParts.at(7).toDouble();
    }
    else if (infoMessageSubType == INFO_R2L_REACHED_TO_TASK)
    {
        taskInfoMsg.infoMessageType = infoMessageSubType;
        taskInfoMsg.senderRobotID = dataParts.at(0).toInt();
        taskInfoMsg.reachingTime = dataParts.at(1).toInt();
    }
    else if (infoMessageSubType == INFO_R2L_REACHED_TO_GOAL)
    {
        taskInfoMsg.infoMessageType = infoMessageSubType;
        taskInfoMsg.senderRobotID = dataParts.at(0).toInt();
        taskInfoMsg.reachingTime = dataParts.at(1).toInt();
    }


    messageTaskInfoFromRobotPub.publish(taskInfoMsg);
}

/*
 Incoming command message from the leader
*/
void RosThread::pubCmdFromLeader(ISLH_msgs::inMessage msg)
{
    QString package = QString::fromStdString(msg.message);

    // package = AA * messageType * messageSubType * datasize * data
    // data = sendingTime & message


    QStringList packageParts = package.split("*",QString::SkipEmptyParts);

    int cmdMessageSubType = packageParts.at(2).toInt();

    qDebug()<<" Incoming command message from the leader -> MessageType: "<<cmdMessageSubType<<" Message: "<<package;

    if ( (cmdMessageSubType == CMD_L2R_START_HANDLING) || (cmdMessageSubType == CMD_L2R_MOVE_TO_TASK_SITE) || (cmdMessageSubType == CMD_L2R_MOVE_TO_GOAL_POSE) )
    {        

        ISLH_msgs::cmdFromLeaderMessage msgCmd;

        msgCmd.cmdTypeID = cmdMessageSubType;

        QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

        msgCmd.sendingTime = dataParts.at(0).toUInt();
        msgCmd.cmdMessage = dataParts.at(1).toStdString();

        messageCmdFromLeaderPub.publish(msgCmd);
    }
    else if (cmdMessageSubType == CMD_L2R_SPLIT_FROM_COALITION)
    {

        ISLH_msgs::cmdFromLeaderMessage msgCmd;

        msgCmd.cmdTypeID = cmdMessageSubType;

        QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

        msgCmd.sendingTime = dataParts.at(0).toUInt();
        msgCmd.cmdMessage = dataParts.at(1).toStdString();

        messageCmdFromLeaderPub.publish(msgCmd);
    }
    else if (cmdMessageSubType == CMD_L2R_LEADER_CHANGED)
    {

        QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

        QStringList messageParts = dataParts.at(1).split(":",QString::SkipEmptyParts);

        QStringList messageSubParts = messageParts.at(0).split("&",QString::SkipEmptyParts);

        if (messageParts.size() > 1) // I am new coalition leader
        {
            ISLH_msgs::newLeaderMessage msgNewLeader;

            msgNewLeader.sendingTime = dataParts.at(0).toUInt();

            msgNewLeader.infoTypeID = cmdMessageSubType;

            msgNewLeader.infoMessage = dataParts.at(1).toStdString();

            messageNewLeaderPub.publish(msgNewLeader);

            myLeaderRobotID = ownRobotID;
          }
          else
          {
              QString tmpStr = QString(messageSubParts.at(1)).remove("NewLeaderID");
              myLeaderRobotID = tmpStr.toInt();
          }

        // informs the coalition member of the new leader

        ISLH_msgs::cmdFromLeaderMessage msgCmd;

        msgCmd.cmdTypeID = cmdMessageSubType;

        msgCmd.sendingTime = messageSubParts.at(0).toUInt();

        msgCmd.cmdMessage = messageSubParts.at(1).toStdString();

        messageCmdFromLeaderPub.publish(msgCmd);
    }
    else if (cmdMessageSubType == CMD_L2R_I_AM_LEADER)
    {
        QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

        QString tmpStr = QString(dataParts.at(1)).remove("NewLeaderID");
        myLeaderRobotID = tmpStr.toInt();

        // informs the coalition member's taskHandlerISLH of the new leader
        ISLH_msgs::cmdFromLeaderMessage msgCmd;

        msgCmd.cmdTypeID = cmdMessageSubType;

        msgCmd.sendingTime = dataParts.at(0).toUInt();

        msgCmd.cmdMessage = dataParts.at(1).toStdString();

        messageCmdFromLeaderPub.publish(msgCmd);
    }
    else if (cmdMessageSubType == CMD_L2R_NEW_ALL_TARGET_POSES)
    {
        ISLH_msgs::targetPoseListMessage targetPoseListMsg;

        QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

        QStringList messageParts = dataParts.at(1).split(";",QString::SkipEmptyParts);

          for(int mpIndx=0; mpIndx < messageParts.size(); mpIndx++)
          {
              QStringList messageSubParts = messageParts.at(mpIndx).split(",", QString::SkipEmptyParts);

              targetPoseListMsg.robotIDs.push_back(messageSubParts.at(0).toInt());

              geometry_msgs::Pose2D poseTmp;
              poseTmp.x = messageSubParts.at(1).toDouble();
              poseTmp.y = messageSubParts.at(2).toDouble();

              targetPoseListMsg.targetPoses.push_back(poseTmp);
          }

          messageTargetPoseListPub.publish(targetPoseListMsg);
    }

}

/*
 Incoming command message received from the task coordinator
*/
void RosThread::pubCmdFromCoordinator(ISLH_msgs::inMessage msg)
{
    ISLH_msgs::cmdFromCoordinatorMessage msgCmd;

    QString package = QString::fromStdString(msg.message);

    // package = AA * messageType * messageSubType * datasize * data
    // data = sendingTime & message

    QStringList packageParts = package.split("*",QString::SkipEmptyParts);

    msgCmd.messageTypeID = packageParts.at(2).toInt();

    QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

    msgCmd.sendingTime = dataParts.at(0).toUInt();

    msgCmd.message = dataParts.at(1).toStdString();


    messageCmdFromCoordinatorPub.publish(msgCmd);

}

/*
 Incoming task info message from the leader to the task coordinator
*/
void RosThread::pubTaskInfoFromLeader(ISLH_msgs::inMessage msg)
{
   ISLH_msgs::taskInfoFromLeaderMessage taskInfoMsg;


   QString package = QString::fromStdString(msg.message);

   // package = AA * messageType * messageSubType * datasize * data
   // data = sendingTime & message

   QStringList packageParts = package.split("*",QString::SkipEmptyParts);

   taskInfoMsg.infoTypeID = packageParts.at(2).toInt();

   QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

   taskInfoMsg.sendingTime = dataParts.at(0).toUInt();

   QStringList messageParts = dataParts.at(1).split(";",QString::SkipEmptyParts);

   taskInfoMsg.senderRobotID = messageParts.at(0).toUInt();

   if (taskInfoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE)
   {

       taskInfoMsg.taskUUID = messageParts.at(1).toStdString();

       taskInfoMsg.posX = messageParts.at(2).toDouble();

       taskInfoMsg.posY = messageParts.at(3).toDouble();

       taskInfoMsg.encounteringTime = messageParts.at(4).toUInt();

       taskInfoMsg.requiredResources = messageParts.at(5).toStdString();

       taskInfoMsg.encounteringRobotID = messageParts.at(6).toUInt();

       taskInfoMsg.timeOutDuration = messageParts.at(7).toUInt();

       taskInfoMsg.handlingDuration = messageParts.at(8).toUInt();

   }
   else if (taskInfoMsg.infoTypeID == INFO_L2C_START_HANDLING_WITH_TASK_INFO)
   {
       taskInfoMsg.taskUUID = messageParts.at(1).toStdString();

       taskInfoMsg.posX = messageParts.at(2).toDouble();

       taskInfoMsg.posY = messageParts.at(3).toDouble();

       taskInfoMsg.encounteringTime = messageParts.at(4).toUInt();

       taskInfoMsg.requiredResources = messageParts.at(5).toStdString();

       taskInfoMsg.encounteringRobotID = messageParts.at(6).toUInt();

       taskInfoMsg.startHandlingTime = messageParts.at(7).toUInt();

       taskInfoMsg.timeOutDuration = messageParts.at(8).toUInt();

       taskInfoMsg.handlingDuration = messageParts.at(9).toUInt();
   }
   else if ( (taskInfoMsg.infoTypeID == INFO_L2C_START_HANDLING) || (taskInfoMsg.infoTypeID == INFO_L2C_TASK_COMPLETED) )
   {
       taskInfoMsg.taskUUID = messageParts.at(1).toStdString();
   }
   else if ( (taskInfoMsg.infoTypeID == INFO_L2C_SPLITTING) || (taskInfoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED) )
   {   
       taskInfoMsg.extraMsg = messageParts.at(1).toStdString();
   }
   else if  ( (taskInfoMsg.infoTypeID == INFO_L2C_WAITING_GOAL_POSE) || (taskInfoMsg.infoTypeID == INFO_L2C_WAITING_TASK_SITE_POSE) )
   {
       taskInfoMsg.extraMsg = messageParts.at(1).toStdString();
   }

   messageTaskInfoFromLeaderPub.publish(taskInfoMsg);

}


/*
 Incoming coalition info from the old leader to the new leader

void RosThread::sendNewLeaderInfoFromOldLeader(communicationISLH::inMessage msg)
{

}
*/

/*
 Outgoing command message from the leader to the robot(s)
*/
void RosThread::sendCmd2Robots(ISLH_msgs::cmd2RobotsFromLeaderMessage msg)
{

    ISLH_msgs::outMessage outmsg;

    QString data;
    QString temp;

    temp = QString::number(msg.sendingTime);
    data.append(temp);

    if  ( (msg.cmdTypeID == CMD_L2R_START_HANDLING) || (msg.cmdTypeID == CMD_L2R_START_OR_STOP_MISSION) )
    {
        data.append("&");

        temp = QString::fromStdString(msg.cmdMessage);
        data.append(temp);

        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            if (ownRobotID == msg.receiverRobotID[i]) // If the receiver robot is itself, send the message to taskHandlerISLH directly
            {
                ISLH_msgs::cmdFromLeaderMessage directMsg;

                directMsg.cmdTypeID = msg.cmdTypeID;
                directMsg.sendingTime = msg.sendingTime;
                directMsg.cmdMessage = msg.cmdMessage;

                messageCmdFromLeaderPub.publish(directMsg);
            }
            else
            {
                outmsg.messageIndx.push_back(0);

                outmsg.robotid.push_back(msg.receiverRobotID[i]);
            }
        }
        outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_LEADER_TO_ROBOT);
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, data));
    }
    else if ( (msg.cmdTypeID == CMD_L2R_MOVE_TO_TASK_SITE) || (msg.cmdTypeID == CMD_L2R_MOVE_TO_GOAL_POSE) ){
        data.append("&");

        temp = QString::fromStdString(msg.cmdMessage);

        QStringList messages = temp.split(";", QString::SkipEmptyParts);
        int idx = 0;

        for(int i=0; i < messages.size(); i++)
        {
            QStringList messageParts = messages.at(i).split(",", QString::SkipEmptyParts);
            int robotIdx = messageParts.at(0).toInt();
            QString message = QString("%1,%2").arg(messageParts.at(1)).arg(messageParts.at(2));
            if (ownRobotID == robotIdx) // If the receiver robot is itself, send the message to taskHandlerISLH directly
            {
                ISLH_msgs::cmdFromLeaderMessage directMsg;

                directMsg.cmdTypeID = msg.cmdTypeID;
                directMsg.sendingTime = msg.sendingTime;
                directMsg.cmdMessage = message.toStdString();

                messageCmdFromLeaderPub.publish(directMsg);
            }
            else
            {
                outmsg.messageIndx.push_back(idx);

                outmsg.robotid.push_back(robotIdx);

                outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_LEADER_TO_ROBOT);
                outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, QString("%1%2").arg(data).arg(message)));

                idx++;
            }
        }
    }
    else if (msg.cmdTypeID == CMD_L2R_SPLIT_FROM_COALITION)
    {
        data.append("&");
        data.append(QString::fromStdString(msg.cmdMessage));

        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            outmsg.messageIndx.push_back(0);

            outmsg.robotid.push_back(msg.receiverRobotID[i]);
        }

        outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_LEADER_TO_ROBOT);
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, data));
    }
    else if (msg.cmdTypeID == CMD_L2R_LEADER_CHANGED)
    {
        QString cmdMessageStr = QString::fromStdString(msg.cmdMessage);
        QStringList cmdMessageParts = cmdMessageStr.split(":",QString::SkipEmptyParts);

        temp = cmdMessageParts.at(0);
        temp = temp.remove("NewLeaderID");
        int leaderID = temp.toInt();

        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            if (msg.receiverRobotID.at(i) != leaderID)
                outmsg.messageIndx.push_back(0);
            else
                outmsg.messageIndx.push_back(1);

            outmsg.robotid.push_back(msg.receiverRobotID[i]);

        }
        outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_LEADER_TO_ROBOT);

        // this message is sent to member robots
        temp = QString::number(msg.sendingTime);
        temp = temp.append("&");
        temp = temp.append(cmdMessageParts.at(0));
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, temp));

        // this message is sent to the new leader
        temp = QString::number(msg.sendingTime);
        temp = temp.append("&");
        temp = temp.append(cmdMessageStr);
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, temp));
    }
    else if (msg.cmdTypeID == CMD_L2R_I_AM_LEADER)
    {             
        QString cmdMessageStr = QString::fromStdString(msg.cmdMessage);
        QStringList cmdMessageParts = cmdMessageStr.split(":",QString::SkipEmptyParts);

        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            outmsg.messageIndx.push_back(0);

            outmsg.robotid.push_back(msg.receiverRobotID[i]);
        }
        outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_LEADER_TO_ROBOT);

        temp = QString::number(msg.sendingTime);
        temp = temp.append("&");
        temp = temp.append(cmdMessageParts.at(0));
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, temp));


        // I am the leader
        myLeaderRobotID = ownRobotID;

        // informs  taskHandlerISLH of the new leader
        ISLH_msgs::cmdFromLeaderMessage msgCmd;
        msgCmd.cmdTypeID = msg.cmdTypeID;
        msgCmd.sendingTime = msg.sendingTime;
        msgCmd.cmdMessage = cmdMessageParts.at(0).toStdString();
        messageCmdFromLeaderPub.publish(msgCmd);
    }
    else if (msg.cmdTypeID == CMD_L2R_NEW_ALL_TARGET_POSES)
    {
        data.append("&");

        temp = QString::fromStdString(msg.cmdMessage);
        data.append(temp);

        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            if (msg.receiverRobotID.at(i) == ownRobotID)
            {
                ISLH_msgs::targetPoseListMessage targetPoseListMsg;

                QString msgStr = QString::fromStdString(msg.cmdMessage);

                  QStringList messageParts = msgStr.split(";", QString::SkipEmptyParts);
                  for(int mpIndx=0; mpIndx < messageParts.size(); mpIndx++)
                  {
                      QStringList messageSubParts = messageParts.at(mpIndx).split(",", QString::SkipEmptyParts);

                      targetPoseListMsg.robotIDs.push_back(messageSubParts.at(0).toInt());

                      geometry_msgs::Pose2D poseTmp;
                      poseTmp.x = messageSubParts.at(1).toDouble();
                      poseTmp.y = messageSubParts.at(2).toDouble();

                      targetPoseListMsg.targetPoses.push_back(poseTmp);
                  }

                  messageTargetPoseListPub.publish(targetPoseListMsg);
            }
            else
            {
                outmsg.robotid.push_back(msg.receiverRobotID.at(i));

                outmsg.messageIndx.push_back(0);
            }
        }
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, data));
    }

    messageOutPub.publish(outmsg);
}

// Outgoing command message from the task coordinator to the leader(s)
void RosThread::sendCmd2Leaders(ISLH_msgs::cmd2LeadersMessage msg)
{
    ISLH_msgs::outMessage outmsg;

    int robotCnt = 0;
    for(int i=0; i<msg.leaderRobotID.size(); i++)
    {
        // if the robot is also coalition leader,
        // publish the message directly to the coalitionLeaderISL node
        if (msg.leaderRobotID[i] == ownRobotID)
        {
            ISLH_msgs::cmdFromCoordinatorMessage directMsg;

            directMsg.sendingTime = msg.sendingTime;
            directMsg.messageTypeID = msg.messageTypeID[i];
            directMsg.message = msg.message[i];

            messageCmdFromCoordinatorPub.publish(directMsg);
        }
        else
        {
            QString data;

            data.append(QString::number(msg.sendingTime));

            data.append('&');

            data.append(QString::fromStdString(msg.message[i]));

            outmsg.robotid.push_back(msg.leaderRobotID[i]);
            outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_COORDINATOR_TO_LEADER, msg.messageTypeID[i], data));//msg.message[i];
            outmsg.messageIndx.push_back(robotCnt);
            outmsg.messageTypeID.push_back(msg.messageTypeID[i]);
            robotCnt++;
        }
    }

    if (outmsg.robotid.size()>0)
    {
        messageOutPub.publish(outmsg);
    }
}

//Outgoing task info message from the member robot to its leader
void RosThread::sendTaskInfo2Leader(ISLH_msgs::taskInfo2LeaderMessage msg)
{
    // If the robot is coaltion leader,
    // publish the message to the coalitionLeaderISLH node
    if (ownRobotID==myLeaderRobotID)
    {
        ISLH_msgs::taskInfoFromRobotMessage directMsg;

        if (msg.infoMessageType == INFO_R2L_NEW_TASK_INFO)
        {
            directMsg.infoMessageType = INFO_R2L_NEW_TASK_INFO;
            directMsg.taskUUID = msg.taskUUID;
            directMsg.posX = msg.posX;
            directMsg.posY = msg.posY;
            directMsg.senderRobotID = msg.senderRobotID;
            directMsg.handlingDuration = msg.handlingDuration;
            directMsg.timeOutDuration = msg.timeOutDuration;
            directMsg.requiredResources = msg.requiredResources;
            directMsg.encounteringTime = msg.encounteringTime;
        }
        else if (msg.infoMessageType == INFO_R2L_REACHED_TO_TASK)
        {          
            directMsg.infoMessageType = INFO_R2L_REACHED_TO_TASK;
            directMsg.taskUUID = msg.taskUUID;
            directMsg.senderRobotID = msg.senderRobotID;
            directMsg.reachingTime = msg.reachingTime;
        }
        else if (msg.infoMessageType == INFO_R2L_REACHED_TO_GOAL)
        {
            directMsg.infoMessageType = INFO_R2L_REACHED_TO_GOAL;
            directMsg.senderRobotID = msg.senderRobotID;
            directMsg.reachingTime = msg.reachingTime;
        }

        messageTaskInfoFromRobotPub.publish(directMsg);
    }
    // if the robot is not coalition leader,
    // send the message to the leader through wireless communication
    else
    {
        ISLH_msgs::outMessage outmsg;

        QString data;
        QString temp;

        if (msg.infoMessageType == INFO_R2L_NEW_TASK_INFO)
        {
            temp = QString::number(msg.senderRobotID);
            data.append(temp);

            data.append(";");

            temp = QString::fromStdString(msg.taskUUID);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg.encounteringTime);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg.handlingDuration);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg. timeOutDuration);
            data.append(temp);

            data.append(";");

            temp = QString::fromStdString(msg.requiredResources);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg.posX);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg.posY);
            data.append(temp);
        }
        else if (msg.infoMessageType == INFO_R2L_REACHED_TO_TASK)
        {
            temp = QString::number(msg.senderRobotID);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg.reachingTime);
            data.append(temp);
        }
        else if (msg.infoMessageType == INFO_R2L_REACHED_TO_GOAL)
        {
            temp = QString::number(msg.senderRobotID);
            data.append(temp);

            data.append(";");

            temp = QString::number(msg.reachingTime);
            data.append(temp);
        }

        outmsg.robotid.push_back(msg.receiverRobotID);
        outmsg.messageIndx.push_back(0);
        outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_ROBOT_TO_LEADER);
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_ROBOT_TO_LEADER, msg.infoMessageType, data));

        messageOutPub.publish(outmsg);
    }
}


//Outgoing task info message from the coalition leader to the task coordinator
void RosThread::sendTaskInfo2Coordinator(ISLH_msgs::taskInfo2CoordinatorMessage taskInfoMsg)
{

    // If the robot is coodinator,
    // publish the message to the taskCoordinatorISLH node
    if (ownRobotID==taskInfoMsg.receiverRobotID)
    {
        ISLH_msgs::taskInfoFromLeaderMessage directMsg;

        directMsg.encounteringRobotID = taskInfoMsg.encounteringRobotID;
        directMsg.encounteringTime = taskInfoMsg.encounteringTime;
        directMsg.extraMsg = taskInfoMsg.extraMsg;
        directMsg.handlingDuration =  taskInfoMsg.handlingDuration;
        directMsg.infoTypeID = taskInfoMsg.infoTypeID;
        directMsg.posX = taskInfoMsg.posX;
        directMsg.posY = taskInfoMsg.posY;
        directMsg.requiredResources = taskInfoMsg.taskResource;
        directMsg.senderRobotID = taskInfoMsg.senderRobotID;
        directMsg.sendingTime = taskInfoMsg.sendingTime;
        directMsg.startHandlingTime = taskInfoMsg.startHandlingTime;
        directMsg.taskUUID = taskInfoMsg.taskUUID;
        directMsg.timeOutDuration = taskInfoMsg.timeOutDuration;

        messageTaskInfoFromLeaderPub.publish(directMsg);
    }
    else
    {
        ISLH_msgs::outMessage outmsg;

        QString data;
        QString temp;

        temp = QString::number(taskInfoMsg.sendingTime);
        data.append(temp);

        if ( (taskInfoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE) || (taskInfoMsg.infoTypeID == INFO_L2C_WAITING_TASK_SITE_POSE) )
        {
            data.append("&");

            data.append(QString::number(taskInfoMsg.senderRobotID));

            data.append(";");

            data.append(QString::fromStdString(taskInfoMsg.taskUUID));

            data.append(";");

            data.append(QString::number(taskInfoMsg.posX));

            data.append(";");

            data.append(QString::number(taskInfoMsg.posY));

            data.append(";");

            data.append(QString::number(taskInfoMsg.encounteringTime));

            data.append(";");

            data.append(QString::fromStdString(taskInfoMsg.taskResource));

            data.append(";");

            data.append(QString::number(taskInfoMsg.encounteringRobotID));

            data.append(";");

            data.append(QString::number(taskInfoMsg.timeOutDuration));

            data.append(";");

            data.append(QString::number(taskInfoMsg.handlingDuration));

        }
        else if (taskInfoMsg.infoTypeID == INFO_L2C_START_HANDLING_WITH_TASK_INFO)
        {
            data.append("&");

            data.append(QString::number(taskInfoMsg.senderRobotID));

            data.append(";");

            data.append(QString::fromStdString(taskInfoMsg.taskUUID));

            data.append(";");

            data.append(QString::number(taskInfoMsg.posX));

            data.append(";");

            data.append(QString::number(taskInfoMsg.posY));

            data.append(";");

            data.append(QString::number(taskInfoMsg.encounteringTime));

            data.append(";");

            data.append(QString::fromStdString(taskInfoMsg.taskResource));

            data.append(";");

            data.append(QString::number(taskInfoMsg.encounteringRobotID));

            data.append(";");

            data.append(QString::number(taskInfoMsg.startHandlingTime));

            data.append(";");

            data.append(QString::number(taskInfoMsg.timeOutDuration));

            data.append(";");

            data.append(QString::number(taskInfoMsg.handlingDuration));

        }
        else if ( (taskInfoMsg.infoTypeID == INFO_L2C_START_HANDLING) || (taskInfoMsg.infoTypeID == INFO_L2C_TASK_COMPLETED) )
        {

            data.append("&");

            data.append(QString::number(taskInfoMsg.senderRobotID));

            data.append(";");

            data.append(QString::fromStdString(taskInfoMsg.taskUUID));
        }
        else if ( (taskInfoMsg.infoTypeID == INFO_L2C_SPLITTING) || (taskInfoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED) || (taskInfoMsg.infoTypeID == INFO_L2C_WAITING_GOAL_POSE) )
        {
            data.append("&");

            data.append(QString::number(taskInfoMsg.senderRobotID));

            data.append(";");

            data.append(QString::fromStdString(taskInfoMsg.extraMsg));
        }


        outmsg.robotid.push_back(taskInfoMsg.receiverRobotID);
        outmsg.messageIndx.push_back(0);
        outmsg.messageTypeID.push_back(MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR);
        outmsg.message.push_back(makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR, taskInfoMsg.infoTypeID, data));

        messageOutPub.publish(outmsg);
    }

}

//void RosThread::sendTaskInfo2Leader(messageDecoderISLH::taskInfo2Leader taskInfoMsg)
//{
//    QString data;
/*
    QString temp = QString::number(info.neighbors.size());
    //qDebug()<<info.neighbors.size();
    data.append(temp);

   // data.append(info.neighbors.size());

    data.append(";");

    for(int i = 0; i < info.neighbors.size();i++){

        QString temp = QString::fromStdString(info.neighbors[i]);

        data.append(temp);

        data.append(";");


    }
    temp = QString::number(info.posX);

    data.append(temp);

    data.append(";");

    temp = QString::number(info.posY);

    data.append(temp);

    data.append(";");

    temp = QString::number(info.radius);

    data.append(temp);

    data.append(";");

    temp = QString::number(info.targetX);

    data.append(temp);

    data.append(";");

    temp = QString::number(info.targetY);

    data.append(temp);

    //data.append(";");

*/
    //int dataSize = data.size();


    //QString dat = makeDataPackage(RECV_ROBOT_INFO,dataSize,data);


//}

std::string RosThread::makeDataPackage(int messageType, int messageSubType, QString data)
{

    QString package;

    QString str;

    str = "AA";

    package.append(str); // control byte

    package.append("*");

    str.setNum(messageType);
    package.append(str); //message type

    package.append("*");

    str.setNum(messageSubType);
    package.append(str); //command type

    package.append("*");

    str.setNum(data.size()); // data size
    package.append(str);

    package.append("*");

    package.append(data); //data

    return package.toStdString();


}



// Reads the config file
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {

        coordinatorRobotID = result["taskCoordinatorRobotID"].toInt();
        qDebug()<< " coordinatorRobotID " << coordinatorRobotID;

        queueSize = result["queueSize"].toInt();
        qDebug()<<result["queueSize"].toString();

        ownRobotID = result["robotID"].toInt();

        myLeaderRobotID = ownRobotID;

    }
    file.close();
    return true;



}

