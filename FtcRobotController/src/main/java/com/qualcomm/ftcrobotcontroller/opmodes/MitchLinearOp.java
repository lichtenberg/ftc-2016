/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


public class MitchLinearOp extends LinearOpMode {


	//
    // The member variables below represent the hardware features of the robot.
    // For each of these objects, we will retrieve information from the
    // hardwareMap.   The hardwareMap is the list of devices that you
    // named on the robot controller using the "settings" and "scan" functions
    // of the robot controller app.

	DcMotor motorRight;
	DcMotor motorLeft;
	ColorSensor colorSensor;
	OpticalDistanceSensor distanceSensor;
	I2cDevice gyro;
    TouchSensor touchSensor;

    //
    // The member variables below are used for when we are executing a turn.
    //
	int destHeading;
    ModernGyroReader gyroReader;

    //
    // The member variables below are used when we are following a line.
    //

    int blackBaseLine = 0;
	Boolean lineDetected = false;


    //
    // The member variables below describe the drive train of the motor
    //
    double wheelDiameter = 2.0;     // Wheel diameter in inches
    double gearReduction = 1.0;     // Amount of gear reduction in driver train
    double encoderCountsPerRevolution = 1120; // This is the number of encoder counts per turn of the output shaft

    //
    // The member variables below are for holding the current "state" of our program.
    //

    int currentMode = 0;


    /**
	 * Constructor
	 */
	public MitchLinearOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */



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


	private void startTurning(int howMuch)
	{
		destHeading = normalizeHeading(gyroReader.getHeading() + howMuch);
		currentMode = 2;
	}




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


	private void lineFollowMode()
	{

		// In automatic mode, look out for detecting the line
		// for the first time.
		//
		// If we have not detected the line yet, keep going
		// at 0.25 power.   Once we see the white line,
		// set lineDetected true and start following.

		int lineSensor = distanceSensor.getLightDetectedRaw();

		if (lineSensor > (blackBaseLine+50)) {
			if (!lineDetected) {
				colorSensor.enableLed(true);
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

	}

	private int subtractHeadings(int a,int b)
	{
		int difference;

		difference = a - b;

		if (difference > 180) return -(360 - difference);
		if (difference <= -180) return (360 + difference);

		return difference;
	}

	private void gyroTurnMode()
	{
		int curHeading = gyroReader.getHeading();

		int degreesToTurn = subtractHeadings(destHeading,curHeading);
		Boolean shouldTurnLeft;
		double turnSpeed = 0.10;

		if (degreesToTurn > 0) {
			shouldTurnLeft = true;
		} else {
			shouldTurnLeft = false;
		}


		// If we need to turn

		DbgLog.msg("TURN: myHeading:" + curHeading + " dest " + (destHeading) + " left:" + degreesToTurn + (shouldTurnLeft ? " LEFT" : " RIGHT"));

		if (Math.abs(degreesToTurn) <= 20) {
			turnSpeed = 0.04;
		}

		if (Math.abs(degreesToTurn) <= 1) {
			motorLeft.setPower(0);
			motorRight.setPower(0);
			currentMode = 0;

		} else if (shouldTurnLeft) {
			motorLeft.setPower(-turnSpeed);
			motorRight.setPower(turnSpeed);

		} else {
			motorLeft.setPower(turnSpeed);
			motorRight.setPower(-turnSpeed);

		}

	}


	public void moveDistance(double distanceInInches, double motorPower)
	{
        int distanceInEncoderCounts = inchesToEncoder(distanceInInches);

		motorLeft.setChannelMode(RunMode.RUN_TO_POSITION);
		motorRight.setChannelMode(RunMode.RUN_TO_POSITION);

		motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + distanceInEncoderCounts);
		motorRight.setTargetPosition(motorRight.getCurrentPosition() + distanceInEncoderCounts);

		motorLeft.setPower(motorPower);
		motorRight.setPower(motorPower);

		currentMode = 4;
	}
	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */


	@Override
	public void runOpMode() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 *   Gather motors out of the hardware map.
		 */
		motorRight = hardwareMap.dcMotor.get("moto2");
		motorLeft = hardwareMap.dcMotor.get("moto1");
		motorRight.setDirection(DcMotor.Direction.REVERSE);


        /*
         * Gather the sensors out of the hardware map.
         */

		colorSensor = hardwareMap.colorSensor.get("color1");
		distanceSensor = hardwareMap.opticalDistanceSensor.get("dist1");

        /*
         * Set up our gyro.   To help with the gyro we take the I2C device
         * and give it to our special ModernGyroReader class, which takes care
         * of the dirty work of reading the heading out of the gyro.
         */

		gyro = hardwareMap.i2cDevice.get("gyro1");
		gyroReader = new ModernGyroReader(gyro);


		touchSensor = hardwareMap.touchSensor.get("touch1");

        /*
         * Set up the color sensor.
         */

		colorSensor.enableLed(true);


		int rawSensorValue;

		// After the INIT button is pressed, this routine gets called over and over
		// in a loop.

		rawSensorValue = distanceSensor.getLightDetectedRaw();
		if (rawSensorValue > blackBaseLine) {
			blackBaseLine = rawSensorValue;
		}

		telemetry.addData("baseline", blackBaseLine );
		telemetry.addData("buttons", (gamepad1.x ? "X" : "") + (gamepad1.y ? "Y" : ""));
		telemetry.addData("touch", touchSensor.isPressed() ? "Touch" : "no_touch");

		gyroReader.checkGyro();


		try {
			waitForStart();
		} catch (InterruptedException e) {
			return;
		}

		// If you press the game pad 'A' button, switch to automatic mode.
		// Drive forward at 0.25 until we hit the line, then start following the
		// line.
		//
		// If you press the game pad 'B' button, cancel auto mode and go back
		// to standard driving.

		while (true) {

			try {
				waitForNextHardwareCycle();
			} catch (InterruptedException e) {
				return;
			}

			gyroReader.checkGyro();

			switch (currentMode) {
				default:
				case 0:
					teleopMode();
					break;
				case 1:
					lineFollowMode();
					break;
				case 2:
					gyroTurnMode();
					break;
				case 3:
					moveDistance(12, 0.2);
					break;
				case 5:
					moveDistance(-12, 0.2);
					break;
				case 4:
					if (!motorLeft.isBusy() && !motorRight.isBusy()) {
						motorLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
						motorRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
						motorLeft.setPower(0);
						motorRight.setPower(0);
						currentMode = 0;
						DbgLog.msg("DONE WITH ENCODER RUN");
					} else {
						DbgLog.msg("WAITING FOR ENCODER RUN");
					}
			}

			if (gamepad1.a && (currentMode != 0)) {
				lineDetected = false;
				motorLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
				motorRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);

				currentMode = 0;
			} else if (gamepad1.b && (currentMode != 1)) {
				lineDetected = false;
				currentMode = 1;
			} else if (gamepad1.x && (currentMode != 2)) {
				startTurning(90);
				DbgLog.msg("STARTING TURN");
			} else if (gamepad1.y && (currentMode != 2)) {
				startTurning(-90);
				DbgLog.msg("STARTING TURN");
			} else if (gamepad1.left_bumper) {
				currentMode = 3;
			} else if (gamepad1.right_bumper) {
				currentMode = 5;
			}


		/*
		 * Send telemetry data back to driver station.
		 */

			//telemetry.addData("Text", "*** Robot Data***");
			telemetry.addData("Time", String.format("%f", this.time));
			telemetry.addData("Color:", "A:" + colorSensor.alpha() + " R:" + colorSensor.red() + " G:" + colorSensor.green() + " B:" + colorSensor.blue());
			telemetry.addData("OptDist", distanceSensor.getLightDetectedRaw());
			telemetry.addData("baseline", blackBaseLine + (lineDetected ? " line" : " no line"));
			telemetry.addData("heading", gyroReader.getHeading());
			telemetry.addData("encoders", "Left:" + motorLeft.getCurrentPosition() + " right:" + motorRight.getCurrentPosition());

		}

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
