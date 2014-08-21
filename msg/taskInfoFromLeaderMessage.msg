# The task's information to the task coordinator

uint64 sendingTime #The sending time

uint16 infoTypeID

uint16 senderRobotID # robot id of coalition leader 

uint16 encounteringRobotID #robot id which encounters the task

uint64 encounteringTime #The encountering time in timestamp form

uint64 startHandlingTime #the time when the task starts being handled

uint16 handlingDuration

uint16 timeOutDuration

string requiredResources # Required resources for this task

float32 posX # The position of the task at x direction
float32 posY # The position of the task at y direction

string taskUUID # the task's UUID

string extraMsg

