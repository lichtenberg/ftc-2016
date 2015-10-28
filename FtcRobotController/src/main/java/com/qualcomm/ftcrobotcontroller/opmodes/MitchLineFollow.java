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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.ftccommon.DbgLog;

import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

class I2CDeviceReader {

	private final I2cDevice device;
	private boolean transaction_complete;
	private boolean buffer_read_complete;
	private byte[] device_data;

	public I2CDeviceReader(I2cDevice i2cDevice, int i2cAddress, int memAddress, int num_bytes) {
		this.device = i2cDevice;
		device_data = null;
		transaction_complete = false;
		buffer_read_complete = false;
		i2cDevice.enableI2cReadMode(i2cAddress, memAddress, num_bytes);
		i2cDevice.setI2cPortActionFlag();
		i2cDevice.writeI2cCacheToController();
		i2cDevice.registerForI2cPortReadyCallback(new I2cController.I2cPortReadyCallback() {
			public void portIsReady(int port) {
				I2CDeviceReader.this.portDone();
			}
		});
	}

	public boolean isDone() {
		return this.transaction_complete && buffer_read_complete;
	}

	private void portDone() {
		if (!transaction_complete && device.isI2cPortReady()) {
			transaction_complete = true;
			device.readI2cCacheFromController();
		}
		else if (transaction_complete) {
			device_data = this.device.getCopyOfReadBuffer();
			device.deregisterForPortReadyCallback();
			buffer_read_complete = true;
		}
	}

	public byte[] getReadBuffer() {
		return device_data;
	}
}


public class MitchLineFollow extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */

	DcMotor motorRight;
	DcMotor motorLeft;
	ColorSensor colorSensor;
	OpticalDistanceSensor distanceSensor;
	I2cDevice gyro;
	int curHeading;
	int destHeading;

	I2CDeviceReader gyroReader = null;

	int blackBaseLine = 0;

	Boolean lineDetected = false;

	int currentMode = 0;

	/**
	 * Constructor
	 */
	public MitchLineFollow() {

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
		
		/*
		 *   Gather motors out of the hardware map.
		 */
		motorRight = hardwareMap.dcMotor.get("moto2");
		motorLeft = hardwareMap.dcMotor.get("moto1");
		motorRight.setDirection(DcMotor.Direction.REVERSE);
		

		colorSensor = hardwareMap.colorSensor.get("color1");
		distanceSensor = hardwareMap.opticalDistanceSensor.get("dist1");

		gyro = hardwareMap.i2cDevice.get("gyro1");

		colorSensor.enableLed(true);
        


	}


	private int normalizeHeading(int heading)
	{
		while (heading < 0) heading += 360;
		while (heading > 360) heading -= 360;
		return heading;
	}

	private void checkGyro()
	{
		if (gyroReader == null) {
			gyroReader = new I2CDeviceReader(gyro, 0x20, 0x04, 2);
		} else {
			if (gyroReader.isDone()) {
				byte[] buffer = gyroReader.getReadBuffer();
				int msb = (buffer[1] & 0xFF);
				int lsb = (buffer[0] & 0xFF);
				curHeading = (msb << 8) | lsb;
				telemetry.addData("rawgyro", String.format("%02x %02x %d", buffer[1],buffer[0],curHeading));
				gyroReader = null;
			}
		}
	}

	private void startTurning(int howMuch)
	{
		destHeading = normalizeHeading(curHeading + howMuch);
		currentMode = 2;
	}

	@Override
	public void init_loop()
	{
		int rawSensorValue;

		// While the INIT button is being held down, sample the light sensor
		// and remember the highest value we found.   This will be our
		// baseline for 'black'.    Numbers significantly above
		// this value will be 'white' (and represent the line).

		rawSensorValue = distanceSensor.getLightDetectedRaw();
		if (rawSensorValue > blackBaseLine) {
			blackBaseLine = rawSensorValue;
		}

		colorSensor.enableLed(true);

		telemetry.addData("baseline", blackBaseLine );
		telemetry.addData("buttons", (gamepad1.a ? "A" : "") + (gamepad1.b ? "B" : "") +
				(gamepad1.x ? "X" : "") + (gamepad1.y ? "Y" : ""));

		checkGyro();

	}

	@Override
	public void start()
	{
		telemetry.clearData();
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
		int degreesToTurn = subtractHeadings(destHeading,curHeading);
		Boolean shouldTurnLeft;
		double turnSpeed = 0.30;

		if (degreesToTurn > 0) {
			shouldTurnLeft = true;
		} else {
			shouldTurnLeft = false;
		}


		// If we need to turn

		DbgLog.msg("TURN: myHeading:" + curHeading + " dest " + (destHeading) + " left:" + degreesToTurn + (shouldTurnLeft ? " LEFT" : " RIGHT"));

		if (Math.abs(degreesToTurn) <= 5) {
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


	public void moveALittle(Boolean backwards)
	{
		motorLeft.setChannelMode(RunMode.RUN_TO_POSITION);
		motorRight.setChannelMode(RunMode.RUN_TO_POSITION);

		motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + (backwards ? -2400 : 2400));
		motorRight.setTargetPosition(motorRight.getCurrentPosition() + (backwards ? -2400 : 2400));

		motorLeft.setPower(0.2);
		motorRight.setPower(0.2);

		currentMode = 4;
	}
	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {



		// If you press the game pad 'A' button, switch to automatic mode.
		// Drive forward at 0.25 until we hit the line, then start following the
		// line.
		//
		// If you press the game pad 'B' button, cancel auto mode and go back
		// to standard driving.

		checkGyro();

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
				moveALittle(false);
				break;
			case 5:
				moveALittle(true);
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
		}
		else if (gamepad1.b && (currentMode != 1)) {
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
		telemetry.addData("heading", curHeading);
		telemetry.addData("encoders","Left:"+motorLeft.getCurrentPosition() + " right:"+motorRight.getCurrentPosition());

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
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
