#include	<Arduino.h>
#include	<Wire.h>
#include	<I2Cdev.h>
#include	<MPU6050_6Axis_MotionApps20.h>

MPU6050			mpu;
HardwareSerial	rxtx(PA_10, PA_9);
uint8_t			erty = 255;

//#define	OUTPUT_READABLE_QUATERNION
#define		OUTPUT_READABLE_EULER
//#define	OUTPUT_READABLE_YAWPITCHROLL
//#define	OUTPUT_READABLE_REALACCEL
//#define	OUTPUT_READABLE_WORLDACCEL

#define		INTERRUPT_PIN PB5	//	use pin PB5 on STM32F103C8T6
#define		LED_PIN PC13		//	use pin PC13 on STM32F103C8T6

bool blinkState = false;

// MPU control/status vars
bool		dmpReady = false;	//	set true if DMP init was successful
uint8_t		mpuIntStatus;		//	holds actual interrupt status byte from MPU
uint8_t		devStatus;			//	return status after each device operation (0 = success, !0 = error)
uint16_t	packetSize;			//	expected DMP packet size (default is 42 bytes)
uint16_t	fifoCount;			//	count of all bytes currently in FIFO
uint8_t		fifoBuffer[64];		//	FIFO storage buffer

//	orientation/motion vars
Quaternion	q;			//	[w, x, y, z]         quaternion container
VectorInt16	aa;			//	[x, y, z]            accel sensor measurements
VectorInt16	aaReal;		//	[x, y, z]            gravity-free accel sensor measurements
VectorInt16	aaWorld;	//	[x, y, z]            world-frame accel sensor measurements
VectorFloat	gravity;	//	[x, y, z]            gravity vector
float euler[3];			//	[psi, theta, phi]    Euler angle container
float euler_f[3];		//	[psi, theta, phi]    Euler angle container
float ypr[3];			//	[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool	mpuInterrupt = false;	//	indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
	mpuInterrupt = true;
}

void setup() {
	//	oin I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		Wire.setClock(400000);	//	400kHz I2C clock. Comment this line if having compilation difficulties
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	rxtx.begin(115200);
//	delay(500);
	//	initialize device
	rxtx.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);
//	delay(500);

	//	verify connection
	rxtx.println(F("Testing device connections..."));
//	delay(500);
	rxtx.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//	delay(500);

	// load and configure the DMP
	rxtx.println(F("Initializing DMP..."));
//	delay(500);
	devStatus = mpu.dmpInitialize();
//	delay(500);

	// make sure it worked (returns 0 if so)
	rxtx.println(devStatus);
//	delay(500);
	if (devStatus == 0) {
		//	Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
//		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		rxtx.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
	rxtx.println(F("Enabling DMP PASSED..."));
//	delay(500);

		//	enable Arduino interrupt detection
		rxtx.print(F("Enabling interrupt detection (Arduino external interrupt "));
		rxtx.print(digitalPinToInterrupt(INTERRUPT_PIN));
		rxtx.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		rxtx.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		rxtx.print(F("DMP Initialization failed (code "));
		rxtx.print(devStatus);
		rxtx.println(F(")"));
	}

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);

//	euler_f[0] = 1.1;
//	euler_f[1] = 1.1;
//	euler_f[2] = 1.1;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady) return;
	// read a packet from FIFO
	if (mpuInterrupt && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
//	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

	#ifdef OUTPUT_READABLE_QUATERNION
			//	display quaternion values in easy matrix form: w x y z
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			Serial.print("quat\t");
			Serial.print(q.w);
			Serial.print("\t");
			Serial.print(q.x);
			Serial.print("\t");
			Serial.print(q.y);
			Serial.print("\t");
			Serial.println(q.z);
		#endif

		#ifdef OUTPUT_READABLE_EULER
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetEuler(euler, &q);
			//	Float - Little Endian (DCBA)
			rxtx.write((byte*)&erty, 1);
//			rxtx.write((byte*)&euler_f[0], 4);
//			rxtx.write((byte*)&euler_f[1], 4);
//			rxtx.write((byte*)&euler_f[2], 4);
			euler[0] = euler[0] * 180/M_PI;
			euler[1] = euler[1] * 180/M_PI;
			euler[2] = euler[2] * 180/M_PI;
			rxtx.write((byte*)&euler[0], 4);
			rxtx.write((byte*)&euler[1], 4);
			rxtx.write((byte*)&euler[2], 4);
		#endif

		#ifdef OUTPUT_READABLE_YAWPITCHROLL
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			_rs485.print("ypr\t");
			_rs485.print(ypr[0] * 180/M_PI);
			_rs485.print("\t");
			_rs485.print(ypr[1] * 180/M_PI);
			_rs485.print("\t");
			_rs485.println(ypr[2] * 180/M_PI);
		#endif

		#ifdef OUTPUT_READABLE_REALACCEL
			// display real acceleration, adjusted to remove gravity
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			Serial.print("areal\t");
			Serial.print(aaReal.x);
			Serial.print("\t");
			Serial.print(aaReal.y);
			Serial.print("\t");
			Serial.println(aaReal.z);
		#endif

		#ifdef OUTPUT_READABLE_WORLDACCEL
			// display initial world-frame acceleration, adjusted to remove gravity
			// and rotated based on known orientation from quaternion
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			Serial.print("aworld\t");
			Serial.print(aaWorld.x);
			Serial.print("\t");
			Serial.print(aaWorld.y);
			Serial.print("\t");
			Serial.println(aaWorld.z);
		#endif

		#ifdef OUTPUT_TEAPOT
			// display quaternion values in InvenSense Teapot demo format:
			teapotPacket[2] = fifoBuffer[0];
			teapotPacket[3] = fifoBuffer[1];
			teapotPacket[4] = fifoBuffer[4];
			teapotPacket[5] = fifoBuffer[5];
			teapotPacket[6] = fifoBuffer[8];
			teapotPacket[7] = fifoBuffer[9];
			teapotPacket[8] = fifoBuffer[12];
			teapotPacket[9] = fifoBuffer[13];
			Serial.write(teapotPacket, 14);
			teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		#endif

		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
		mpuInterrupt = false;
	}
}
