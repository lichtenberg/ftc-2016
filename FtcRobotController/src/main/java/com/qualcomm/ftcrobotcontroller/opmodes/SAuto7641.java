
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class SAuto7641 extends OpMode {

	FtcConfig ftcConfig = new FtcConfig();

	//
    // The member variables below represent the hardware features of the robot.
    // For each of these objects, we will retrieve information from the
    // hardwareMap.   The hardwareMap is the list of devices that you
    // named on the robot controller using the "settings" and "scan" functions
    // of the robot controller app.

	DcMotor motorRight;
	DcMotor motorLeft;
	ColorSensor colorSensor1;
	ColorSensor colorSensor2;
	OpticalDistanceSensor distanceSensor;
	I2cDevice gyro;
    TouchSensor touchSensor;

    //
	// The member variables below are used for when we are executing a turn.
	//
	int destHeading;
	Boolean gyroTurnIsRunning = false;
	ModernGyroReader gyroReader;

    //
    // The member variables below are used when we are following a line.
    //

    int blackBaseLine = 0;
	Boolean lineDetected = false;
	Boolean lineFollowerIsRunning = false;


    //
    // The member variables below describe the drive train of the motor
    //
    double wheelDiameter = 2.75;     // Wheel diameter in inches
    double gearReduction = 1.0;     // Amount of gear reduction in drive train
    double encoderCountsPerRevolution = 1120; // This is the number of encoder counts per turn of the output shaft

	Boolean moveDistanceIsRunning = false;

    //
    // The member variables below are for holding the current "state" of our program.
    //


	enum autoStep {
		IDLE,
		FORWARD_1,
		TURN_1,
		TURN_2,
		FORWARD_2,
		STOP
	};

	autoStep currentStep = autoStep.IDLE;



	/**
	 * Constructor
	 */
	public SAuto7641() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		ftcConfig.init(hardwareMap.appContext, this);
		
		/*
		 *   Gather motors out of the hardware map.
		 */
		motorRight = hardwareMap.dcMotor.get("motor_2");
		motorLeft = hardwareMap.dcMotor.get("motor_1");
		motorRight.setDirection(DcMotor.Direction.REVERSE);


        /*
         * Gather the sensors out of the hardware map.
         */

		colorSensor1 = hardwareMap.colorSensor.get("color1");
		colorSensor2 = hardwareMap.colorSensor.get("color2");
		distanceSensor = hardwareMap.opticalDistanceSensor.get("dist1");

        /*
         * Set up our gyro.   To help with the gyro we take the I2C device
         * and give it to our special ModernGyroReader class, which takes care
         * of the dirty work of reading the heading out of the gyro.
         */

		gyro = hardwareMap.i2cDevice.get("gyro1");
		gyroReader = new ModernGyroReader(gyro);


        touchSensor = hardwareMap.touchSensor.get("touch1");


		colorSensor2.setI2cAddress(0x3e);
        /*
         * Set up the color sensor.
         */
		colorSensor1.enableLed(false);
		colorSensor2.enableLed(true);



	}


    //
    // normalizeHeading(heading)
    //
    // Force a heading to fit within the range 0..359.   For example,
    // if the heading is 370 degrees, that's really the same as 10 degrees.
    // If the heading is -10 degrees, that's really 350.
    //
	private int normalizeHeading(int heading)
	{
		while (heading < 0) heading += 360;
		while (heading >= 360) heading -= 360;
		return heading;
	}


	//
	// subtractHeadings(a,b)
	//
	// Subtract two compass headings, returning the difference between them in degrees.
	//
	// This routine tries to be smart about the direction, so it will pick the smaller of
	// the two differences (turning left 10 vs right 350).
	//
	private int subtractHeadings(int a,int b)
	{
		int difference;

		difference = a - b;

		if (difference > 180) return -(360 - difference);
		if (difference <= -180) return (360 + difference);

		return difference;
	}
    //
    // inchesToEncoder(inches)
    //
    // Given some distance in inches, compute the amount that the encoders will change after the robot
    // has moved that distance.
    //
    // To do this, we need the following info:
    //    - The diameter of the wheel
    //    - The ratio of the gears between the motor's output shaft and the wheel
    //    - The number of encoder counts for each revolution of the output shaft.
    //
    // For example, suppose the wheel is 2" in diameter and the encoder counts 1120 pulses per revolution.
    // Also, imagine that you have a 2:1 reduction using a chain or gears.
    // This means that you need 1120 * 2 = 2240 pulses to rotate the wheel one turn.
    // Since the wheel's diameter is 2", this means that the circumference is 2*pi = 6.28".
    // If you want to move 12 inches, you would need the wheels to turn 1.91 times.
    // 1.91 rotations is 1.91 * 1120 * 2 encoder counts = 4274.
    //
    private int inchesToEncoder(double inches)
    {
        double wheelCircumference = Math.PI * wheelDiameter;
        double encoderCounts;

        encoderCounts = (inches / wheelCircumference) * (encoderCountsPerRevolution * gearReduction);

        return (int) encoderCounts;
    }


	//
	// The init_loop routine is called repeatedly in between the time you press the INIT
	// button and the time you press the START button.
	//
	// For our example, we will read the optical distance (line follow) sensor
	// and remember its maximum value.  This should be the color of the black
	// carpet tiles, so that we can easily tell the difference between the black
	// tile and the white line.
	//


	@Override
	public void init_loop()
	{
		int rawSensorValue;

		// After the INIT button is pressed, this routine gets called over and over
        // in a loop.

		ftcConfig.init_loop(hardwareMap.appContext, this);


		rawSensorValue = distanceSensor.getLightDetectedRaw();
		if (rawSensorValue > blackBaseLine) {
			blackBaseLine = rawSensorValue;
		}

		//
		// Add some telemetry.   The FTC forum says you should be able to read
		// the joystick values here, but I have not been able to do so.
		//
		telemetry.addData("baseline", blackBaseLine );
		telemetry.addData("buttons", (gamepad1.x ? "X" : "") + (gamepad1.y ? "Y" : ""));
        telemetry.addData("touch", touchSensor.isPressed() ? "Touch" : "no_touch");



		//
		// Keep reading the gyro all the time.
		//

		gyroReader.checkGyro();

	}

	//
	// The start() method below is called when you press the triangle start button on the
	// driver station.   This routine is called only once.
	//

	@Override
	public void start()
	{
		// Erase all the telemetry data.

		telemetry.clearData();


		// Set the current autonomous step to our first step, FORWARD_1.

		currentStep = autoStep.FORWARD_1;
	}


	//
	// Although this example is for autonomous mode, when we reach the end this particular
	// program lets you continue with teleop.  In a real competition, this would not be allowed.
	//
	private void teleopMode()
	{
		float left = -gamepad1.left_stick_y;
		float right = -gamepad1.right_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float) scaleInput(right);
		left = (float) scaleInput(left);

		// write the values to the motors
		motorRight.setPower(right);
		motorLeft.setPower(left);
	}


	//
	// This is the line sensor follower step.
	//

	private Boolean lineFollower()
	{
		//
		// If we have not detected the line yet, keep going
		// at 0.25 power.   Once we see the white line,
		// set lineDetected true and start following.

		if (lineFollowerIsRunning) {

			int lineSensor = distanceSensor.getLightDetectedRaw();

			if (lineSensor > (blackBaseLine + 50)) {
				if (!lineDetected) {
					DbgLog.msg("LINE_FOLLOW:  Detected the line");

					lineDetected = true;
				}
			}

			if (!lineDetected) {
				// Line not detected yet.   Keep going straight.
				motorLeft.setPower(0.20);
				motorRight.setPower(0.20);
			} else {

				// Line has been detected.   If we see the line, turn one way.
				// If we do not see the line, turn the other way.
				// The robot will oscillate as it follows the line.

				if (lineSensor > (blackBaseLine + 50)) { // light detected
					lineDetected = true;
					motorLeft.setPower(0);
					motorRight.setPower(0.15);
				} else {
					motorLeft.setPower(0.15);
					motorRight.setPower(0);
				}

			}
			//
			// If we hit the wall, we will use our touch sensor to stop this step.
			// We will monitor the touch sensor even if we haven't found the line yet,
			// just in case we miss the line entirely and slam into the wall.
			//

			if (touchSensor.isPressed()) {
				DbgLog.msg("LINE_FOLLOW:  touch sensor activated, stopping");

				motorLeft.setPower(0);
				motorRight.setPower(0);
				lineFollowerIsRunning = false;
			}

		} else {
			lineFollowerIsRunning = true;
		}

		return lineFollowerIsRunning;

	}


	//
	// gyroTurn(howMuch)
	//
	// Start a gyro-based turn.   howMuch is the number of degrees you want to turn,
	// which can be negative for right turns.
	//
	// It returns 'true' if the turn has just started or is still in progress
	// 'false' is returned when the turn is complete.
	//
	private Boolean gyroTurn(int howMuch)
	{
		if (gyroTurnIsRunning) {
			int curHeading = gyroReader.getHeading();

			int degreesToTurn = subtractHeadings(destHeading, curHeading);
			Boolean shouldTurnLeft;
			//double turnSpeed = 0.10;
			//double slowTurnSpeed = 0.04;

			double turnSpeed = 0.20;
			double slowTurnSpeed = 0.10;

			if (degreesToTurn > 0) {
				shouldTurnLeft = true;
			} else {
				shouldTurnLeft = false;
			}


			// If we need to turn

			DbgLog.msg("TURN: myHeading:" + curHeading + " dest " + (destHeading) + " left:" + degreesToTurn + (shouldTurnLeft ? " LEFT" : " RIGHT"));

			if (Math.abs(degreesToTurn) <= 20) {
				turnSpeed = slowTurnSpeed;
			}

			if (Math.abs(degreesToTurn) <= 1) {
				motorLeft.setPower(0);
				motorRight.setPower(0);
				gyroTurnIsRunning = false;

			} else if (shouldTurnLeft) {
				motorLeft.setPower(-turnSpeed);
				motorRight.setPower(turnSpeed);

			} else {
				motorLeft.setPower(turnSpeed);
				motorRight.setPower(-turnSpeed);

			}
		} else {

			destHeading = normalizeHeading(gyroReader.getHeading() + howMuch);

			DbgLog.msg("STARTING TURN: amount " + howMuch + " destHeading " + destHeading);

			gyroTurnIsRunning = true;
		}

		return gyroTurnIsRunning;
	}


	public Boolean moveDistance(double distanceInInches, double motorPower)
	{
		// If the "move distance" routine is running, test to see if we should
		// be stopped.

		// If the "move distance" routine is not yet running, set it to run.

		// The return value of this routine is 'true' if we are still busy.

		if (moveDistanceIsRunning) {
			if (!motorLeft.isBusy() && motorRight.isBusy()) {
				motorLeft.setPower(0);
				motorRight.setPower(0);
				motorLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
				motorRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);

				DbgLog.msg("MOVE_DISTANCE: finished");

				moveDistanceIsRunning = false;
			}
		} else {
			int distanceInEncoderCounts = inchesToEncoder(distanceInInches);

			DbgLog.msg("MOVE_DISTANCE: distance " + distanceInInches + " power " + motorPower);


			motorLeft.setChannelMode(RunMode.RUN_TO_POSITION);
			motorRight.setChannelMode(RunMode.RUN_TO_POSITION);

			motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + distanceInEncoderCounts);
			motorRight.setTargetPosition(motorRight.getCurrentPosition() + distanceInEncoderCounts);

			motorLeft.setPower(motorPower);
			motorRight.setPower(motorPower);

			moveDistanceIsRunning = true;
		}

		return moveDistanceIsRunning;

	}

	//
	// This is the main loop method.  Note that it's not really a loop,
	// the loop() routine is called over and over by the Robot Controller main class
	// You are supposed to decide what to do based on what you were doing before, and
	// any inputs from the joysticks or sensors.
	//
	// Do not try to wait for things to happen in this routine, that will just make
	// the runtime angry.
	//

	@Override
	public void loop() {


		//
		// We update the gyro's currentHeading each time the loop is called.
		// It runs on its own, we query the current compass position by reading the
		// latest value that checkGyro gets from the compass.
		//

		gyroReader.checkGyro();

		//
		// Based on the current step in our autonomous program, decide what to do.
		//
		// Remember, each of the "steps" below is a routine that returns a Boolean result.
		// a 'true' return means that the step has just started or is still running (so don't move onto the next step)
		// a 'false' return means that the step has finished, and we can move onto the next step.
		//
		// This is a state machine, written (hopefully) simply enough to understand.
		//

		switch (currentStep) {
			case IDLE:
				break;
			case FORWARD_1:
				if (moveDistance(12.0, 0.3) == false) {
					currentStep = autoStep.TURN_1;
				}
				break;
			case TURN_1:
				if (gyroTurn(90) == false) {
					currentStep = autoStep.TURN_2;
				}
				break;
			case TURN_2:
				if (gyroTurn(-90) == false) {
					currentStep = autoStep.FORWARD_2;
				}
				break;
			case FORWARD_2:
				if (moveDistance(12.0, 0.3) == false) {
					currentStep = autoStep.STOP;
				}
				break;
			case STOP:
				teleopMode();
				break;

		}



      /*
       * Send telemetry data back to driver station.
       */

		//telemetry.addData("Text", "*** Robot Data***");
		//telemetry.addData("Time", String.format("%f", this.time));

		telemetry.addData("Color1:", "A:" + colorSensor1.alpha() + " R:" + colorSensor1.red() + " G:" + colorSensor1.green() + " B:" + colorSensor1.blue());
		telemetry.addData("Color2:", "A:" + colorSensor2.alpha() + " R:" + colorSensor2.red() + " G:" + colorSensor2.green() + " B:" + colorSensor2.blue());
		telemetry.addData("OptDist", distanceSensor.getLightDetectedRaw());
		telemetry.addData("baseline", blackBaseLine + (lineDetected ? " line" : " no line"));
		telemetry.addData("heading", gyroReader.getHeading());
		telemetry.addData("encoders","Left:"+motorLeft.getCurrentPosition() + " right:"+motorRight.getCurrentPosition());
		telemetry.addData("step#",currentStep);

		telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
		telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
		telemetry.addData("AutonType", ftcConfig.param.autonType);

		telemetry.addData("BeaconValue",detectBeacon());


	}
	public int detectBeacon() {
		// lower limit below red and blue are meaningful
		int threshold = 0;

		telemetry.addData("Color:", "A:" + colorSensor1.alpha() + " R:" + colorSensor1.red() + " G:" + colorSensor1.green() + " B:" + colorSensor1.blue());
		DbgLog.msg("Color: A: " + colorSensor1.alpha() + " R: " + colorSensor1.red() + " G: " + colorSensor1.green() + " B: " + colorSensor1.blue());

		if (colorSensor1.red() < threshold || colorSensor1.blue() < threshold){
			return 0;
		}
		else if (colorSensor1.red() > colorSensor1.blue() ){
			return 1;

		}
		else if (colorSensor1.blue() > colorSensor1.red() ){
			return -1;
		}
		else
			return 0;

	}

	//
	// This method is called when the OpMode is stopped.
	//

	@Override
	public void stop() {

	}

	/*
        * This method scales the joystick input so for low joystick values, the
        * scaled value is less than linear.  This is to make it easier to drive
        * the robot more precisely at slower speeds.
        */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}