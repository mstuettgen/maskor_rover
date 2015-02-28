#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Range.h>


#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_DMP.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#define LED_PIN 13
#define ANALOG_PIN 8
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07

//function prototypes
void oncmdvelreceived_cb( const geometry_msgs::Twist& twist_msg );  //cmd_vel callback
void onstringmsgreceived_cb( const std_msgs::String& msg );
void initMPU6050();
void readDMP();
void readRAW();
float get_voltage_in(int pin_num);

//instance of MPU6050 and servo handlers
MPU6050 mpu; 
Servo throttleServo;   
Servo steeringServo;

//servo parameters
int throttle_pwm_neutral;
int throttle_pwm_range;
int steering_pwm_neutral;
int steering_pwm_range;
int invert_throttle;
int invert_steering;

float throttle_pwm;
float steering_pwm;


//ROS Stuff
std_msgs::Float32MultiArray floatData_;
std_msgs::Int16MultiArray intData_;
std_msgs::String response_;
sensor_msgs::Range range_msg_;

char ir_range_frameid[] = "/ir_ranger";


ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &oncmdvelreceived_cb );
ros::Subscriber<std_msgs::String> string_sub("/mpu6050/command" , &onstringmsgreceived_cb);
ros::Publisher string_pub("/mpu6050/response" , &response_);
ros::Publisher floatData_pub ("/mpu6050/floatdata", &floatData_);
ros::Publisher intData_pub ("/mpu6050/intdata", &intData_); 
ros::Publisher range_pub("range_data" , &range_msg_);

ros::Time last_cmd_vel_received;

float THROTTLE_SCALE_FACTOR = 0.6;
float STEERING_SCALE_FACTOR = 1.0;

float v_ref = 5.0;
float adc_max = 1024.0;
float voltage_slope (v_ref/adc_max);
float equation_offset = 0.1836092228;
float equation_slope = 2.5513414751;
float calibration_offset = 0.02;
float voltage, meter = 0.0;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           	// [w, x, y, z]         quaternion container
VectorInt16 dmp_aa;     	// [x, y, z]            accel sensor measurements
VectorInt16 dmp_aaReal; 	// [x, y, z]            gravity-free accel sensor measurements
VectorInt16 dmp_aaWorld;	// [x, y, z]            world-frame accel sensor measurements	
int16_t dmp_gyro[3];		// [x, y, z]            gyro sensor measurements
int16_t dmp_mag[3];		// [x, y, z]            magnetometer sensor measurements
VectorFloat gravity;    	// [x, y, z]            gravity vector
float euler[3];         	// [psi, theta, phi]    Euler angle container
float ypr[3];           	// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool AccelGyroCalibrated = true;	// wether to enter the calibration process or not


//Accelerometer + Gyroscope calibration stuff
int buffersize=2000; //amount of readings used to average
int acel_deadzone=8; //allowed error for accelerometer calibration
int giro_deadzone=1; //allowed error for gyroscope calibration
int state = 0;       //state of the calibration state machine
int ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;



//float data array for ros will have following content:
enum floatArrayContent {
    quaternion_x,	// DMP quaternion
    quaternion_y,
    quaternion_z,
    quaternion_w,
    //accel_x,		// DMP acceleration
    //accel_y,
    //accel_z,
    //freeaccel_x,	// DMP free acceleration
    //freeaccel_y,
    //freeaccel_z,
    //worldaccel_x,	// DMP world frame acceleration
    //worldaccel_y,
    //worldaccel_z,
    //gyro_x,		// DMP gyro
    //gyro_y,
    //gyro_z,
    //mag_x,		// DMP magnetometer
    //mag_y,
    //mag_z,
    //yaw,		// yaw pitch roll
    //pitch,
    //roll,
    //gravity_x,		// gravity orientation
    //gravity_y,
    //gravity_z,
    N_FLOATS		// array size
};

float floatDataForROS[N_FLOATS]; //initialize array

//integer data array for ros will have the following content:
enum intArrayContent {
    raw_accel_x,	// RAW acceleration
    raw_accel_y,
    raw_accel_z,
    raw_gyro_x,		// RAW gyro
    raw_gyro_y,
    raw_gyro_z,
    mag_x,		// DMP magnetometer
    mag_y,
    mag_z,
    N_INTS		// array size
};

int16_t intDataForROS[N_INTS];  //initialize array



//Interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


//init function of controller
void setup()
{
    //init ros node
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(string_sub);
    nh.advertise(floatData_pub);
    nh.advertise(intData_pub);
    nh.advertise(string_pub);
    nh.advertise(range_pub);

    //give the ros node some time to register with the master
    //before loading params from ros parameter server
    delay(2000);

    //read parameters from ROS launch file
    if(!nh.getParam("/MPU6050_node/XAccelOffset", &ax_offset))
        nh.logerror("could not get param XAccelOffset");
    
    if(!nh.getParam("/MPU6050_node/YAccelOffset", &ay_offset))
        nh.logerror("could not get param YAccelOffset");

    if(!nh.getParam("/MPU6050_node/ZAccelOffset", &az_offset))
        nh.logerror("could not get param ZAccelOffset");

    if(!nh.getParam("/MPU6050_node/XGyroOffset",  &gx_offset))
        nh.logerror("could not get param ZAccelOffset");

    if(!nh.getParam("/MPU6050_node/YGyroOffset",  &gy_offset))
        nh.logerror("could not get param ZAccelOffset");

    if(!nh.getParam("/MPU6050_node/ZGyroOffset",  &gz_offset))
        nh.logerror("could not get param ZAccelOffset");

    if(!nh.getParam("/MPU6050_node/throttle_pwm_neutral",  &throttle_pwm_neutral))
        nh.logerror("could not get param throttle_pwm_neutral");

    if(!nh.getParam("/MPU6050_node/throttle_pwm_range",  &throttle_pwm_range))
        nh.logerror("could not get param throttle_pwm_range");

    if(!nh.getParam("/MPU6050_node/steering_pwm_neutral",  &steering_pwm_neutral))
        nh.logerror("could not get param steering_pwm_neutral");

    if(!nh.getParam("/MPU6050_node/steering_pwm_range",  &steering_pwm_range))
        nh.logerror("could not get param steering_pwm_neutral");

    if(!nh.getParam("/MPU6050_node/invert_steering",  &invert_steering))
        nh.logerror("could not get param invert_steering");

    if(!nh.getParam("/MPU6050_node/invert_throttle",  &invert_throttle))
        nh.logerror("could not get param invert_throttle");


    //bind servos to the connection pins on arduino board
    throttleServo.attach(2);
    steeringServo.attach(5);

    throttle_pwm = throttle_pwm_neutral;
    steering_pwm = steering_pwm_neutral;


    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    //init MPU6050
    mpu.initialize();

    //init DMP
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        ////Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        ////Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }


    // set offsets (after(!) call of mpu.initialize())
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
	
    // magnetic sensor
	mpu.setI2CMasterModeEnabled(0);
	mpu.setI2CBypassEnabled(1);

	Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
    Wire.write((uint8_t)0x02);
    Wire.write((uint8_t)0x00); // Continuos mode
	Wire.endTransmission();
    delay(5);

	Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)B00011000); // 75 Hertz
	Wire.endTransmission();
	delay(5);

    mpu.setI2CBypassEnabled(0);

    // magnetic sensor X axis word
	mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
	mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
	mpu.setSlaveEnabled(0, true);
	mpu.setSlaveWordByteSwap(0, false);
	mpu.setSlaveWriteMode(0, false);
	mpu.setSlaveWordGroupOffset(0, false);
	mpu.setSlaveDataLength(0, 2);

    // magnetic sensor Y axis word
	mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
	mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
	mpu.setSlaveEnabled(1, true);
	mpu.setSlaveWordByteSwap(1, false);
	mpu.setSlaveWriteMode(1, false);
	mpu.setSlaveWordGroupOffset(1, false);
	mpu.setSlaveDataLength(1, 2);

    // magnetic sensor Z axis word
	mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
	mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
	mpu.setSlaveEnabled(2, true);
	mpu.setSlaveWordByteSwap(2, false);
	mpu.setSlaveWriteMode(2, false);
	mpu.setSlaveWordGroupOffset(2, false);
	mpu.setSlaveDataLength(2, 2);

	mpu.setI2CMasterModeEnabled(1);


	// distance sensor 








 /*
	//if you want to supply your own offsets hard-coded, do it here
        mpu.setXAccelOffset(-3650);
        mpu.setYAccelOffset(1716);
        mpu.setZAccelOffset(1553);
        mpu.setXGyroOffset(-16);
        mpu.setYGyroOffset(34);
        mpu.setZGyroOffset(-40);
*/

}

//main loop
void loop()
{
    if (!AccelGyroCalibrated)
    {
        _calibrateAccelGyro();
    }


    ros::Time now = nh.now();
    if(now.toSec() - last_cmd_vel_received.toSec() > 0.1)
    {
       throttleServo.write(throttle_pwm_neutral);
       steeringServo.write(steering_pwm_neutral);
    }
    else
    {
        //write pwm values to servos
        throttleServo.write(throttle_pwm);
        steeringServo.write(steering_pwm);
    }

    //read data
    readRAW();    
    readDMP();

    //spin ros node
    nh.spinOnce();
}



//additional functions
//the callback for the cmd_vel subscriber
void oncmdvelreceived_cb( const geometry_msgs::Twist& twist_msg ){

    //update time
    last_cmd_vel_received = nh.now();


    //apply steering
    float steering_threshold = 0.1;
    float angular_z = twist_msg.angular.z * STEERING_SCALE_FACTOR;

    if (angular_z < -steering_threshold || angular_z > steering_threshold)
    {
        //[-1 : 1] -> [(steering_pwm_neutral-steering_pwm_range) : steering_pwm_neutral : (steering_pwm_neutral+steering_pwm_range]
        steering_pwm = (angular_z * steering_pwm_range) + steering_pwm_neutral;
        if (invert_steering != 0)
	{
		steering_pwm = -1.0 * steering_pwm;
	}
    }
    else //no steering
    {
        steering_pwm = steering_pwm_neutral;
    }

    //apply throttle
    float throttle_threshold = 0.1;
    float linear_x = twist_msg.linear.x * THROTTLE_SCALE_FACTOR;
    if (linear_x > THROTTLE_SCALE_FACTOR) { linear_x = THROTTLE_SCALE_FACTOR; }   
    if (linear_x < -THROTTLE_SCALE_FACTOR) { linear_x = -THROTTLE_SCALE_FACTOR; }

    if (linear_x > throttle_threshold || linear_x < -throttle_threshold)
    {
        //[-1 : 1] -> [(throttle_pwm_neutral-throttle_pwm_range) : throttle_pwm_neutral : (throttle_pwm_neutral+throttle_pwm_range]
        throttle_pwm = (linear_x * throttle_pwm_range) + throttle_pwm_neutral;
        if (invert_throttle != 0)
	{
		throttle_pwm = -1.0 * throttle_pwm;
	}
    }
    else
    {
        throttle_pwm = throttle_pwm_neutral;
    }
}


void onstringmsgreceived_cb( const std_msgs::String& msg ){

    char* s = msg.data;

    if (strcmp(s,"calibrate")==0)
    {
        nh.loginfo("mpu6050: calibration command received!");
        AccelGyroCalibrated = false;
    }

    if (strcmp(s, "getoffsets")==0)
    {	
	_getOffsets();
    }

    if (strcmp(s,"test")==0)
    {
        nh.loginfo("mpu6050: test command received!");
    }

}



//read data from the DMP
void readDMP()
{
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    unsigned long start = millis();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    }
    else if (mpuIntStatus & 0x02) // otherwise, check for DMP data ready interrupt (this should happen frequently)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here //in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //read data from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        //mpu.dmpGetAccel(&dmp_aa, fifoBuffer);
        //mpu.dmpGetGyro(dmp_gyro, fifoBuffer);
        //mpu.dmpGetMag(dmp_mag, fifoBuffer);
	//mpu.dmpGetGravity(&gravity, &q);
        

        //process data
        //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //mpu.dmpGetLinearAccel(&dmp_aaReal, &dmp_aa, &gravity);
        //mpu.dmpGetLinearAccelInWorld(&dmp_aaWorld, &dmp_aaReal, &q);

        //fill array with data for ROS
        floatDataForROS[quaternion_x] = q.x;
        floatDataForROS[quaternion_y] = q.y;
        floatDataForROS[quaternion_z] = q.z;
        floatDataForROS[quaternion_w] = q.w;
        //floatDataForROS[accel_x] = dmp_aa.x;
        //floatDataForROS[accel_y] = dmp_aa.y;
        //floatDataForROS[accel_z] = dmp_aa.z;
        //floatDataForROS[freeaccel_x] = dmp_aaReal.x;
        //floatDataForROS[freeaccel_y] = dmp_aaReal.y;
        //floatDataForROS[freeaccel_z] = dmp_aaReal.z;
        //floatDataForROS[gyro_x] = dmp_gyro[0];
        //floatDataForROS[gyro_y] = dmp_gyro[1];
        //floatDataForROS[gyro_z] = dmp_gyro[2];
        //floatDataForROS[mag_x] = dmp_mag[0];
        //floatDataForROS[mag_y] = dmp_mag[1];
        //floatDataForROS[mag_z] = dmp_mag[2];
        //floatDataForROS[yaw] = ypr[0];
        //floatDataForROS[pitch] = ypr[1];
        //floatDataForROS[roll] = ypr[2];
        //floatDataForROS[gravity_x] = gravity.x;
        //floatDataForROS[gravity_y] = gravity.y;
        //floatDataForROS[gravity_z] = gravity.z;

        //fill ros msg with array data
        floatData_.data = floatDataForROS;
        floatData_.data_length = N_FLOATS;

        //publish to ROS
        floatData_pub.publish( &floatData_ );
    }
}

//read raw data
void readRAW()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t magx, magy, magz;

    //read raw accel and gyro
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //read magnetometer
    magx = mpu.getExternalSensorWord(0);
    magy = mpu.getExternalSensorWord(2);
    magz = mpu.getExternalSensorWord(4);

    //fill array with data for ROS
    intDataForROS[raw_accel_x] = ax;
    intDataForROS[raw_accel_y] = ay;
    intDataForROS[raw_accel_z] = az;
    intDataForROS[raw_gyro_x] = gx;
    intDataForROS[raw_gyro_y] = gy;
    intDataForROS[raw_gyro_z] = gz;
    intDataForROS[mag_x] = magx;
    intDataForROS[mag_y] = magy;
    intDataForROS[mag_z] = magz;

    //fill ros msg with array data
    intData_.data = intDataForROS;
    intData_.data_length = N_INTS;

    //publish to ROS
    intData_pub.publish( &intData_ );


    //read ir range sensor and publish msg
    range_msg_.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg_.header.frame_id =  ir_range_frameid;
    range_msg_.field_of_view = 0.01;
    range_msg_.min_range = 0.1;
    range_msg_.max_range = 0.8;
    range_msg_.range=get_voltage_in(ANALOG_PIN);
    range_msg_.header.stamp = nh.now();
    range_pub.publish( &range_msg_ );

}



void _calibrateAccelGyro()
{
    nh.loginfo("mpu6050: starting accelerometer and gyroscope calibration...");
    nh.loginfo("mpu6050: Place device horizontally and do not touch or move it until FINISHED message!");

    // reset offsets
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    if (state==0){
        nh.loginfo("mpu6050: Reading sensors for first time...");
        _meansensors();
        state++;
        delay(1000);
    }

    if (state==1) {
        nh.loginfo("mpu6050: Calculating offsets...");
        _calibrate();
        state++;
        delay(1000);
    }

    if (state==2) {
        _meansensors();
        nh.loginfo("mpu6050: Offset calibration finished!");
	state++;
	delay(1000);
    }

    if (state==3) {
	nh.loginfo("averaging quaternion...");
	_averageQuaternion();
	delay(1000);
    }

    AccelGyroCalibrated = true;
    _getOffsets();
}


void _meansensors(){
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

    while (i<(buffersize+101)){
        // read raw accel/gyro measurements from device
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
            buff_ax=buff_ax+ax;
            buff_ay=buff_ay+ay;
            buff_az=buff_az+az;
            buff_gx=buff_gx+gx;
            buff_gy=buff_gy+gy;
            buff_gz=buff_gz+gz;
        }
        if (i==(buffersize+100)){
            mean_ax=buff_ax/buffersize;
            mean_ay=buff_ay/buffersize;
            mean_az=buff_az/buffersize;
            mean_gx=buff_gx/buffersize;
            mean_gy=buff_gy/buffersize;
            mean_gz=buff_gz/buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}


void _calibrate(){
    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;

    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;
    while (1){
        int ready=0;
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);

        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);

        _meansensors();
	nh.loginfo("calibrating...");
        
        if (abs(mean_ax)<=acel_deadzone) { ready++; nh.loginfo("x acc ready"); }
        else ax_offset=ax_offset-mean_ax/acel_deadzone;

        if (abs(mean_ay)<=acel_deadzone) { ready++; nh.loginfo("y acc ready"); }
        else ay_offset=ay_offset-mean_ay/acel_deadzone;

        if (abs(16384-mean_az)<=acel_deadzone) { ready++; nh.loginfo("z acc ready"); }
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

        if (abs(mean_gx)<=giro_deadzone) { ready++; nh.loginfo("x gyro ready"); }
        else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

        if (abs(mean_gy)<=giro_deadzone) { ready++; nh.loginfo("y gyro ready"); }
        else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

        if (abs(mean_gz)<=giro_deadzone) { ready++; nh.loginfo("z gyro ready"); }
        else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

        if (ready==6) break;
        else { nh.loginfo("could not calibrate all 6 axis, re-trying...");}
    }
}


void _getOffsets() {
	int base = 10;
  	char buffer [15];
	
	itoa (mpu.getXAccelOffset(), buffer, base);
	nh.loginfo("xacc offset:");        
	nh.loginfo(buffer);

	itoa (mpu.getYAccelOffset(), buffer, base);
	nh.loginfo("yacc offset:");        
	nh.loginfo(buffer);

	itoa (mpu.getZAccelOffset(), buffer, base);
	nh.loginfo("zacc offset:");        
	nh.loginfo(buffer);

        itoa (mpu.getXGyroOffset(), buffer, base);
	nh.loginfo("xgyro offset:");        
	nh.loginfo(buffer);

	itoa (mpu.getYGyroOffset(), buffer, base);
	nh.loginfo("ygyro offset:");        
	nh.loginfo(buffer);

	itoa (mpu.getZGyroOffset(), buffer, base);
	nh.loginfo("zgyro offset:");        
	nh.loginfo(buffer);	
}

void _averageQuaternion() {

	int base = 10;
  	char buffer [15];

	float average_qw;
	float average_qx;
	float average_qy;
	float average_qz;

	int numberOfIterations = 10000;

	for (int i=0; i<numberOfIterations; i++)
	{	
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		average_qw += q.w;
		average_qx += q.x;		
		average_qy += q.y;
		average_qz += q.z;
	}

	average_qw /= numberOfIterations;
	average_qx /= numberOfIterations;
	average_qy /= numberOfIterations;
	average_qz /= numberOfIterations;
	
	itoa (average_qw*1e5, buffer, base);
	nh.loginfo("qw1e5:");
	nh.loginfo(buffer);

	itoa (average_qx*1e5, buffer, base);
	nh.loginfo("qx1e5:");
	nh.loginfo(buffer);

	itoa (average_qy*1e5, buffer, base);
	nh.loginfo("qy1e5:");
	nh.loginfo(buffer);

	itoa (average_qz*1e5, buffer, base);
	nh.loginfo("qz1e5:");
	nh.loginfo(buffer);
}


float get_voltage_in(int pin_num)
{
    int sample = analogRead(pin_num);
    voltage = sample * voltage_slope;
    // equation: a(x) = mx + b with a = 1/y
    // -> x(y) = (1/y - b) / m
    meter = ((1/voltage) - equation_offset) / equation_slope;
    return meter;
}
