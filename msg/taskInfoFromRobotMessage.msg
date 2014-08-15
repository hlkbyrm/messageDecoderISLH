# The task's information  from member robot to coalition leader

uint16 senderRobotID #robot id which encounters the task

uint16 infoMessageType

string taskUUID # the task's UUID

uint64 encounteringTime #The encountering time in timestamp form

uint64 reachingTime # The time when the robot reached to the task site

uint16 handlingDuration

uint16 timeOutDuration

string requiredResources # Required resources for this task

float32 posX # The position of the task at x direction 
float32 posY # The position of the task at y direction 


