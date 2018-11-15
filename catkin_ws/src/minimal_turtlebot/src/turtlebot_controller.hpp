/*struct turtlebotInputs
{
	// time
	uint64_t nanoSecs;

	//wheel drop states
	uint8_t leftWheelDropped;
	uint8_t rightWheelDropped;

	//bumper states
	uint8_t leftBumperPressed;
	uint8_t centerBumperPressed;
	uint8_t rightBumperPressed;

	//color and depth images
	sensor_msgs::Image colorImage;
	sensor_msgs::Image depthImage;

	//cliff states
	uint8_t sensor0State;
	uint8_t sensor1State;
	uint8_t sensor2State;

	//laserscan data
	std::vector<float> ranges;
	float minAngle;
	float maxAngle;
	float angleIncrement;
	int numPoints;

	//imu data
	float linearAccelX;
	float linearAccelY;
	float linearAccelZ;
	float angularVelocityX;
	float angularVelocityY;
	float angularVelocityZ;
	float orientationX;
	float orientationY;
	float orientationZ;
	float orientationW;

	//batt voltage
	float battVoltage;
};*/

//sound options
#define MAIL_RECIEVED 0
#define MAIL_DELIVERED 1
#define PRESENCE 2
#define PICKUP_REQUEST 3
#define ERROR 4
#define START_DELIVERY 5
#define COMPLETE_DELIVERY 6

#define NONE 0.0
#define FORWARD 0.1
#define BACKWARD -0.2
#define RIGHT -0.7
#define LEFT 0.7