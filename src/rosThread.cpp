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
    messageOutPub = n.advertise<communicationISLH::outMessage>("messageDecoderISLH/messageOut",5);


    messageCmdFromCoordinatorPub= n.advertise<messageDecoderISLH::cmdFromCoordinatorMessage>("messageDecoderISLH/cmdFromCoordinator",5);

    messageCmdFromLeaderPub = n.advertise<messageDecoderISLH::cmdFromLeaderMessage>("messageDecoderISLH/cmdFromLeader",5);

    messageTaskInfoFromLeaderPub = n.advertise<messageDecoderISLH::taskInfoFromLeaderMessage>("messageDecoderISLH/taskInfoFromLeader", 5);

    messageTaskInfoFromRobotPub = n.advertise<messageDecoderISLH::taskInfoFromRobotMessage>("messageDecoderISLH/taskInfoFromRobot",5);

    messageNewLeaderPub = n.advertise<messageDecoderISLH::newLeaderMessage>("messageDecoderISLH/newLeader",5);



    messageTaskInfo2LeaderSub = n.subscribe("taskHandlerISLH/taskInfo2Leader",5,&RosThread::sendTaskInfo2Leader,this);

    messageCmd2RobotsSub = n.subscribe("coalitionLeaderISLH/cmd2Robots",5,&RosThread::sendCmd2Robots,this);

    messageCmd2LeadersSub = n.subscribe("taskCoordinatorISLH/cmd2Leaders",5,&RosThread::sendCmd2Leaders,this);

    mesageTaskInfo2CoordinatorSub = n.subscribe("coalitionLeaderISLH/taskInfo2Coordinator", 5,&RosThread::sendTaskInfo2Coordinator, this);



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
void RosThread::handleIncomingMessage(communicationISLH::inMessage msg)
{

    switch(msg.messageid)
    {
    case MT_TASK_INFO_FROM_LEADER_TO_ROBOT:
        sendCmdFromLeader(msg);
        break;
    case MT_TASK_INFO_FROM_ROBOT_TO_LEADER:
        sendTaskInfoFromRobot(msg);
        break;
    case MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR:
        sendTaskInfoFromLeader(msg);
        break;
    case MT_TASK_INFO_FROM_COORDINATOR_TO_LEADER:
        sendCmdFromCoordinator(msg);
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
void RosThread::sendTaskInfoFromRobot(communicationISLH::inMessage msg)
{
    messageDecoderISLH::taskInfoFromRobotMessage taskInfoMsg;

    QString package = QString::fromStdString(msg.message);

    QStringList packageParts = package.split("*",QString::SkipEmptyParts);

    int infoMessageSubType = packageParts.at(2).toInt();

    QStringList dataParts = packageParts.at(4).split(",",QString::SkipEmptyParts);

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
        taskInfoMsg.taskUUID = dataParts.at(1).toStdString();
        taskInfoMsg.reachingTime = dataParts.at(2).toInt();
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
 Incoming command message received from the leader
*/
void RosThread::sendCmdFromLeader(communicationISLH::inMessage msg)
{
    QString package = QString::fromStdString(msg.message);

    // package = AA * messageType * messageSubType * datasize * data
    // data = sendingTime & message


    QStringList packageParts = package.split("*",QString::SkipEmptyParts);

    int cmdMessageSubType = packageParts.at(2).toInt();


    if ( (cmdMessageSubType == CMD_L2R_START_HANDLING) || (cmdMessageSubType == CMD_L2R_MOVE_TO_TASK_SITE) || (cmdMessageSubType == CMD_L2R_MOVE_TO_GOAL_POSE) )
    {        

        messageDecoderISLH::cmdFromLeaderMessage msgCmd;

        msgCmd.cmdTypeID = cmdMessageSubType;

        QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

        msgCmd.sendingTime = dataParts.at(0).toUInt();
        msgCmd.cmdMessage = dataParts.at(1).toStdString();

        messageCmdFromLeaderPub.publish(msgCmd);
    }
    else if (cmdMessageSubType == CMD_L2R_SPLIT_FROM_COALITION)
    {

        messageDecoderISLH::cmdFromLeaderMessage msgCmd;

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

        if (messageParts.size() > 0) // I am new coalition leader
        {
            messageDecoderISLH::newLeaderMessage msgNewLeader;

            msgNewLeader.sendingTime = dataParts.at(0).toUInt();

            msgNewLeader.infoTypeID = cmdMessageSubType;

            msgNewLeader.infoMessage = dataParts.at(1).toStdString();

            messageNewLeaderPub.publish(msgNewLeader);


            /*
            numOfCoalMmbrs = dataParts.at(1).toInt();

            //coalition members' info
            //robotID1;res1,res2,..,resn;posex,posey: ...
            // dataParts.at(2) ... dataParts.at(numOfCoalMmbrs-1)
            for(int i=2; i<numOfCoalMmbrs+1; i++)
            {
                //member robot's info
                QStringList mmbrRobotData = dataParts.at(i).split(";",QString::SkipEmptyParts);
            }

            //the waiting tasks' info
            //dataParts.at(2)

*/
        }
        else // informs the coalition member of the new leader
        {
            messageDecoderISLH::cmdFromLeaderMessage msgCmd;

            msgCmd.cmdTypeID = cmdMessageSubType;

            QStringList dataParts = packageParts.at(4).split("&",QString::SkipEmptyParts);

            msgCmd.sendingTime = dataParts.at(0).toUInt();

            msgCmd.cmdMessage = dataParts.at(1).toStdString();

            messageCmdFromLeaderPub.publish(msgCmd);

        }


    }



    //
}

/*
 Incoming command message received from the task coordinator
*/
void RosThread::sendCmdFromCoordinator(communicationISLH::inMessage msg)
{
// messageDecoderISLH::cmdFromCoordinatorMessage msgCmd;
// ...
// messageCmdFromCoordinatorPub.publish(msgCmd);
}

/*
 Incoming task info message from the leader to the task coordinator
*/
void RosThread::sendTaskInfoFromLeader(communicationISLH::inMessage msg)
{
   // messageDecoderISLH::taskInfoFromLeaderMessage inmsg;


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
void RosThread::sendCmd2Robots(coalitionLeaderISLH::cmd2RobotsFromLeaderMessage msg)
{

    communicationISLH::outMessage outmsg;

    QString data;
    QString temp;

    temp = QString::number(msg.sendingTime);
    data.append(temp);

    if ( (msg.cmdTypeID == CMD_L2R_START_HANDLING) || (msg.cmdTypeID == CMD_L2R_MOVE_TO_TASK_SITE) || (msg.cmdTypeID == CMD_L2R_MOVE_TO_GOAL_POSE) )
    {
        data.append("&");

        temp = QString::fromStdString(msg.cmdMessage);
        data.append(temp);

        int robotCnt = 0;
        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            if (ownRobotID == msg.receiverRobotID[i]) // If the receiver robot is itself, send the message to taskHandlerISLH directly
            {
                messageDecoderISLH::cmdFromLeaderMessage directMsg;

                directMsg.cmdTypeID = msg.cmdTypeID;
                directMsg.sendingTime = msg.sendingTime;
                directMsg.cmdMessage = msg.cmdMessage;

                messageCmdFromLeaderPub.publish(directMsg);
            }
            else
            {
                outmsg.messageIndx[robotCnt] = 0;

                outmsg.robotid[robotCnt] = msg.receiverRobotID[i];

                robotCnt = robotCnt + 1;
            }
        }
        outmsg.messageTypeID[0] = MT_TASK_INFO_FROM_LEADER_TO_ROBOT;
        outmsg.message[0] = makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, data);
    }
    else if (msg.cmdTypeID == CMD_L2R_SPLIT_FROM_COALITION)
    {
        data.append("&");
        data.append(QString::fromStdString(msg.cmdMessage));

        for(int i=0; i<msg.receiverRobotID.size(); i++)
        {
            outmsg.messageIndx[i] = 0;

            outmsg.robotid[i] = msg.receiverRobotID[i];
        }

        outmsg.messageTypeID[0] = MT_TASK_INFO_FROM_LEADER_TO_ROBOT;
        outmsg.message[0] = makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, data);
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
                outmsg.messageIndx[i] = 0;
            else
                outmsg.messageIndx[i] = 1;

            outmsg.robotid[i] = msg.receiverRobotID[i];

        }
        outmsg.messageTypeID[0] = MT_TASK_INFO_FROM_LEADER_TO_ROBOT;

        temp = QString::number(msg.sendingTime);
        temp = temp.append("&");
        temp = temp.append(cmdMessageParts.at(0));
        outmsg.message[0] = makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, temp);

        temp = QString::number(msg.sendingTime);
        temp = temp.append("&");
        temp = temp.append(cmdMessageStr);
        outmsg.message[1] = makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_ROBOT, msg.cmdTypeID, temp);
    }

    messageOutPub.publish(outmsg);
}

// Outgoing command message from the task coordinator to the leader(s)
void RosThread::sendCmd2Leaders(taskCoordinatorISLH::cmd2LeadersMessage msg)
{

}

//Outgoing task info message from the member robot to its leader
void RosThread::sendTaskInfo2Leader(taskHandlerISLH::taskInfo2LeaderMessage msg)
{

    // If the robot is coaltion leader,
    // publish the message to the coalitionLeaderISLH node
    if (ownRobotID==leaderRobotID)
    {
        messageDecoderISLH::taskInfoFromRobotMessage directMsg;

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
        communicationISLH::outMessage outmsg;

        QString data;
        QString temp;

        if (msg.infoMessageType == INFO_R2L_NEW_TASK_INFO)
        {

            temp = QString::number(msg.senderRobotID);
            data.append(temp);

            data.append(",");
/*
            temp = QString::number(msg.infoMessageType);
            data.append(temp);

            data.append(",");
*/
            temp = QString::fromStdString(msg.taskUUID);
            data.append(temp);

            data.append(",");

            temp = QString::number(msg.encounteringTime);
            data.append(temp);

            data.append(",");

            temp = QString::number(msg.handlingDuration);
            data.append(temp);

            data.append(",");

            temp = QString::number(msg. timeOutDuration);
            data.append(temp);

            data.append(",");

            temp = QString::fromStdString(msg.requiredResources);
            data.append(temp);

            data.append(",");

            temp = QString::number(msg.posX);
            data.append(temp);

            data.append(",");

            temp = QString::number(msg.posY);
            data.append(temp);

        }
        else if (msg.infoMessageType == INFO_R2L_REACHED_TO_TASK)
        {
            temp = QString::number(msg.senderRobotID);
            data.append(temp);

            data.append(",");
/*
            temp = QString::number(msg.infoMessageType);
            data.append(temp);

            data.append(",");
*/
            temp = QString::fromStdString(msg.taskUUID);
            data.append(temp);

            data.append(",");

            temp = QString::number(msg.reachingTime);
            data.append(temp);
        }
        else if (msg.infoMessageType == INFO_R2L_REACHED_TO_GOAL)
        {
            temp = QString::number(msg.senderRobotID);
            data.append(temp);

            data.append(",");
/*
            temp = QString::number(msg.infoMessageType);
            data.append(temp);

            data.append(",");
*/
            temp = QString::number(msg.reachingTime);
            data.append(temp);
        }

        outmsg.robotid[0] = msg.receiverRobotID;
        outmsg.messageIndx[0] = 0;
        outmsg.messageTypeID[0] = MT_TASK_INFO_FROM_ROBOT_TO_LEADER;
        outmsg.message[0] = makeDataPackage(MT_TASK_INFO_FROM_ROBOT_TO_LEADER, msg.infoMessageType, data);

        messageOutPub.publish(outmsg);
    }
}


//
void RosThread::sendTaskInfo2Coordinator(coalitionLeaderISLH::taskInfo2CoordinatorMessage taskInfoMsg)
{
    communicationISLH::outMessage outmsg;

    QString data;
    QString temp;

    temp = QString::number(taskInfoMsg.sendingTime);
    data.append(temp);

    if (taskInfoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE)
    {
        data.append("&");

        data.append(QString::fromStdString(taskInfoMsg.taskUUID));

        data.append(";");

        data.append(QString::number(taskInfoMsg.posX));

        data.append(";");

        data.append(QString::number(taskInfoMsg.posY));

        data.append(";");

        data.append(QString::number(taskInfoMsg.encounteringTime));

        data.append(";");

        data.append(QString::fromStdString(taskInfoMsg.taskResource));

    }
    else if ( (taskInfoMsg.infoTypeID == INFO_L2C_START_HANDLING) || (taskInfoMsg.infoTypeID == INFO_L2C_TASK_COMPLETED) )
    {

        data.append("&");

        data.append(QString::fromStdString(taskInfoMsg.taskUUID));
    }
    else if (taskInfoMsg.infoTypeID == INFO_L2C_SPLITTING)
    {
        data.append("&");

        data.append(QString::fromStdString(taskInfoMsg.extraMsg));
    }


    outmsg.robotid[0] = taskInfoMsg.receiverRobotID;
    outmsg.messageIndx[0] = 0;
    outmsg.messageTypeID[0] = MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR;
    outmsg.message[0] = makeDataPackage(MT_TASK_INFO_FROM_LEADER_TO_COORDINATOR, taskInfoMsg.infoTypeID, data);

    messageOutPub.publish(outmsg);


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
        ownRobotID = result["robotID"].toInt();

        leaderRobotID = ownRobotID;

    }
    file.close();
    return true;



}

