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

package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import org.firstinspires.ftc.teamcode.PIDController;


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

enum ButtonEvent {
		BTN_NONE,
		BTN_A_PRESSED,
		BTN_B_PRESSED,
		BTN_X_PRESSED,
		BTN_Y_PRESSED,
		BTN_DPUP_PRESSED,
		BTN_DPDN_PRESSED,
		BTN_DPLEFT_PRESSED,
		BTN_DPRIGHT_PRESSED,
		BTN_BLEFT_PRESSED,
		BTN_BRIGHT_PRESSED
		};



class GamepadButtonEventMonitor {

	boolean prevA,prevB,prevX,prevY;
	boolean prevDPUP,prevDPDN,prevDPLEFT,prevDPRIGHT;
	boolean prevBLEFT,prevBRIGHT;
	ButtonEvent event = ButtonEvent.BTN_NONE;

	public GamepadButtonEventMonitor()
	{

	}

	public void updateState(Gamepad g)
	{
		if (g.a && !prevA) {
			event = ButtonEvent.BTN_A_PRESSED;
		} else if (g.b && !prevB) {
			event = ButtonEvent.BTN_B_PRESSED;
		} else if (g.x && !prevX) {
			event = ButtonEvent.BTN_X_PRESSED;
		} else if (g.y && !prevY) {
			event = ButtonEvent.BTN_Y_PRESSED;
		} else if (g.dpad_up && !prevDPUP) {
			event = ButtonEvent.BTN_DPUP_PRESSED;
		} else if (g.dpad_down && !prevDPDN) {
			event = ButtonEvent.BTN_DPDN_PRESSED;
		} else if (g.dpad_left && !prevDPLEFT) {
			event = ButtonEvent.BTN_DPLEFT_PRESSED;
		} else if (g.dpad_right && !prevDPRIGHT) {
			event = ButtonEvent.BTN_DPRIGHT_PRESSED;
		}else if (g.left_bumper && !prevBLEFT) {
			event = ButtonEvent.BTN_BLEFT_PRESSED;
		} else if (g.right_bumper && !prevBRIGHT) {
			event = ButtonEvent.BTN_BRIGHT_PRESSED;
		}
		prevA = g.a;
		prevB = g.b;
		prevX = g.x;
		prevY = g.y;
		prevDPUP = g.dpad_up;
		prevDPDN = g.dpad_down;
		prevDPLEFT = g.dpad_left;
		prevDPRIGHT = g.dpad_right;
		prevBLEFT = g.left_bumper;
		prevBRIGHT = g.right_bumper;

	}

	public ButtonEvent getButtonEvent()
	{
		ButtonEvent ret = event;
		event = ButtonEvent.BTN_NONE;
		return ret;
	}
}

class GyroWatcher {

	GyroSensor gyro;
	double nullHeading;
	double targetHeading;

	GyroWatcher(GyroSensor g)
	{
		gyro = g;
	}

	private double normalizeAngle(double angle)
	{
		while (angle > 360.0) {
			angle = angle - 360.0;
		}
		while (angle < -360.0) {
			angle = angle + 360.0;
		}

		if (angle >= 180.0) {
			return angle - 360.0;
		}

		return angle;

	}

	public void gyroZero()
	{
		nullHeading = gyro.getHeading();
	}

	public void setTargetHeading(double heading)
	{
		targetHeading = heading;
	}

	public double relativeHeading()
	{
		return normalizeAngle(gyro.getHeading() - nullHeading);
	}

	public double angleTo(double heading)
	{
		return normalizeAngle(heading - relativeHeading());
	}

	public double targetError()
	{
		// Return an "error value" from 0 to 1 representing how far off we are.
		return (normalizeAngle(targetHeading - relativeHeading())) / 180.0;
	}
}

@TeleOp(name="HolomicPlayground", group="MentorBot")
public class HolomicPlayground extends LinearOpMode {


	DcMotor motorFrontRight;
	DcMotor motorFrontLeft;
	DcMotor motorRearRight;
	DcMotor motorRearLeft;

	GyroSensor gyroSensor;

	byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
	int odsSensorValue;
	int ultrasonicSensorValue;

	I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
	public static final int RANGE1_REG_START = 0x04; //Register to start reading
	public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

	public I2cDevice RANGE1;
	public I2cDeviceSynch RANGE1Reader;

	public final boolean usePIDMode = false;

	GamepadButtonEventMonitor gbA;

	double pidP = 1.1;
	double pidI = 0.0;
	double pidD = 0.3;


	/**
	 * Constructor
	 */
	public HolomicPlayground() {
		gbA = new GamepadButtonEventMonitor();
	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	//@Override
	public void initRobot()  throws InterruptedException {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


		motorFrontRight = hardwareMap.dcMotor.get("fr");
		motorFrontLeft = hardwareMap.dcMotor.get("fl");
		motorRearRight = hardwareMap.dcMotor.get("rr");
		motorRearLeft = hardwareMap.dcMotor.get("rl");

		gyroSensor = hardwareMap.gyroSensor.get("gyro");

		// Try using the PID modes to control the motors.

		if (usePIDMode) {
			motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
			motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
			motorRearLeft.setMode(RunMode.RUN_USING_ENCODER);
			motorRearRight.setMode(RunMode.RUN_USING_ENCODER);

			motorFrontLeft.setMaxSpeed(606);
			motorFrontRight.setMaxSpeed(606);
			motorRearLeft.setMaxSpeed(606);
			motorRearRight.setMaxSpeed(606);
		}


		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorRearRight.setDirection(DcMotor.Direction.REVERSE);

		RANGE1 = hardwareMap.i2cDevice.get("range");
		RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
		RANGE1Reader.engage();

		gyroSensor.calibrate();
		while (gyroSensor.isCalibrating()) {
			Thread.sleep(100);
		}

	}

	public double normalizeAngle(double angle)
	{
		while (angle > 360.0) {
			angle = angle - 360.0;
		}
		while (angle < -360.0) {
			angle = angle + 360.0;
		}

		if (angle >= 180.0) {
			return angle - 360.0;
		}

		return angle;

	}

	// On carpet, 1 full rotation takes about 2 seconds.  That's 180 degrees/sec.

	private void setAllMotors(double val)
	{
		double deadBand = 0.06;

		val = Range.clip(val, -1.0, 1.0);

		val = -val;

		if (val != 0) {
			if ((val > 0) && (val < deadBand)) val = deadBand;
			if ((val < 0) && (val > -deadBand)) val = -deadBand;
		}


		// Rotates clockwise with positive 'val' or counter-clockwise with negative 'val'
		motorFrontRight.setPower(-val);
		motorRearLeft.setPower(val);
		motorFrontLeft.setPower(val);
		motorRearRight.setPower(-val);
	}

	public void gyroTurn(double ndeg) throws InterruptedException
	{
		GyroWatcher gw = new GyroWatcher(gyroSensor);

		PIDController pid = new PIDController(0, 0, 0);
		double timeDelta = 0.005;
		double currentTime = 0.0;

		double timeAtTarget = 0.0;
		pid.pidSetParams(pidP, pidI, pidD);
		pid.windupGuard = 1.0;

		gw.gyroZero();
		gw.setTargetHeading(ndeg);

		while (currentTime < 5.0) {

			telemetry.addData("Gyro", String.format("%d ", gyroSensor.getHeading()));
			telemetry.update();

			double error = gw.targetError();

			// If we've been at our target (within a degree) for half a second, we're done.
			if (Math.abs(gw.angleTo(ndeg)) <= 1.0) {
				timeAtTarget += timeDelta;
				if (timeAtTarget >= 0.5) {
					break;
				}
			} else {
				timeAtTarget = 0.0;
			}

			double control = pid.pidUpdate(error, 1.0);

			setAllMotors(control);	// XXX don't we need to scale this somehow?

			Thread.sleep((int) (timeDelta * 1000));

			DbgLog.msg("PID Loop:  Time=%f error=%f control=%f\n", currentTime, gw.angleTo(ndeg), control);

			currentTime += timeDelta;
		}

		setAllMotors(0);

	}
	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void runOpMode() throws InterruptedException {

		boolean holdHeading = false;
		double robotFaceDirection = 0.0;


		initRobot();

		telemetry.addData("init","Gyro Calibrated");
		telemetry.update();

		waitForStart();

		while (opModeIsActive()) {

			double wheelPowerA = 0.0;
			double wheelPowerB = 0.0;
			double wheelA = 0.0;
			double wheelB = 0.0;

			double wheelRL = 0.0;
			double wheelRR = 0.0;
			double wheelFL = 0.0;
			double wheelFR = 0.0;

			double rotateRL = 0.0;
			double rotateRR = 0.0;
			double rotateFL = 0.0;
			double rotateFR = 0.0;

			// Get our current heading and the angle of the omniwheels.
			double omniWheelAngle = 45.0;
			double direction = 0.0;
			double heading = gyroSensor.getHeading();

			// Look for button edge transitions.
			gbA.updateState(gamepad1);

			// Get the current state of the joystick controls.
			double yaxis = -gamepad1.left_stick_y;
			double xaxis = gamepad1.left_stick_x;

			double leftRotate = gamepad1.left_trigger;
			double rightRotate = gamepad1.right_trigger;


			//
			// The range sensor is not part of the FTC SDK yet.  We read
			// the range directly from the sensor using I2C
			//
			// Try to filter out a little noise by rejecting
			// -1 (out of range) readings.

			range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

			odsSensorValue = (int) range1Cache[1];
			if (range1Cache[0] != -1) {
				ultrasonicSensorValue = (int) range1Cache[0];
			}


			//
			// Our button watcher class returns "events" that correspond
			// to actions on the regular joystick buttons.   This way we do not
			// need to look for edges here or deal with the button being
			// down as we run through the loop multiple times.
			//
			switch (gbA.getButtonEvent()) {
				case BTN_A_PRESSED:
					holdHeading = !holdHeading;
					if (holdHeading) {
						robotFaceDirection = gyroSensor.getHeading();
					}
					break;
				case BTN_B_PRESSED:
					gyroTurn(90.0);
					break;
				case BTN_X_PRESSED:
					gyroTurn(-90.0);
					break;
				case BTN_DPUP_PRESSED:
					pidP = pidP + 0.1;
					break;
				case BTN_DPDN_PRESSED:
					pidP = pidP - 0.1;
					break;
				case BTN_DPLEFT_PRESSED:
					pidI = pidI - 0.1;
					break;
				case BTN_DPRIGHT_PRESSED:
					pidI = pidI + 0.1;
					break;
				case BTN_BLEFT_PRESSED:
					pidD = pidD - 0.1;
					break;
				case BTN_BRIGHT_PRESSED:
					pidD = pidD + 0.1;
					break;
				default:
					break;
			}

			// The right joystick is for rotating in place.  We use the bumpers on
			// the back of the joystick for rotation.

			rotateFR = 0.0;
			rotateFL = 0.0;
			rotateRL = 0.0;
			rotateRR = 0.0;


			// Let's keep a dead zone when we're close to zero on the joystick bumper
			// buttons - they're usually zero, but as joysticks get abused they might
			// float a little above zero.
			//
			// For rotation in place, run all the wheels in the same direction.
			// The negatives below are a little confusing; above in the motor setup
			// we set two of the wheels to 'reverse' for being on the opposite side
			// of the robot as you face it.  Holonomic robots don't really have
			// front vs back.
			//
			if (leftRotate > 0.01) {
				rotateFR = leftRotate;
				rotateRL = -leftRotate;
				rotateFL = -leftRotate;
				rotateRR = leftRotate;

			} else if (rightRotate > 0.01) {
				rotateFR = -rightRotate;
				rotateRL = rightRotate;
				rotateFL = rightRotate;
				rotateRR = -rightRotate;

			}



			// The joystick is not supposed to return values beyond +/- 1,
			// but just in case...
			xaxis = Range.clip(xaxis, -1, 1);
			yaxis = Range.clip(yaxis, -1, 1);

			// Compute our desired heading given the Y and X joystick values.
			// We could work in radians, but degrees looks better on telemetry.
			// You don't really need to bother with the trigonometry for a simple
			// control, but doing it this way lets you do additional transformations
			// like the "hold heading" feature below.

			direction = Math.toDegrees(Math.atan2(yaxis, xaxis));

			// Add in the offset that our omniwheels are mounted at (45 degrees) and
			// normalize to +/- 180 degrees.
			direction = normalizeAngle(direction - omniWheelAngle);

			// "Hold Heading" mode uses the gyro to keep the "front" of the robot facing
			// the same direction.  The joystick therefore moves in the same direction
			// no matter where the front of the robot is.

			if (holdHeading) {
				direction = normalizeAngle(direction - (heading - robotFaceDirection));
			}

			// Compute wheel amounts.  We only need two for normal driving,
			// since we do the same thing to opposite wheels.
			// Compute the ratio first, then the magnitude is
			// done by the hypotenuse distance.

			// None of this stuff is really necessary for joystick driving,
			// since we have the ratios of the joysticks, but we can use
			// this to build autonomous code.

			wheelB = Math.cos(Math.toRadians(direction));
			wheelA = Math.sin(Math.toRadians(direction));

			double magnitude = Math.sqrt((xaxis * xaxis) + (yaxis * yaxis));

			wheelA = wheelA * magnitude;
			wheelB = wheelB * magnitude;


			// Combine everything together for all four wheels.

			wheelFR = wheelA + rotateFR;
			wheelRL = wheelA + rotateRL;
			wheelFL = wheelB + rotateFL;
			wheelRR = wheelB + rotateRR;

			// OK, we don't want to pass more than 1.0 into the motor controllers, otherwise
			// the proportions will all be incorrect.   Compute the max value
			// for all our motor controllers and if more than 1.0, scale everything.

			double maxFront = Math.max(Math.abs(wheelFR), Math.abs(wheelFL));
			double maxRear = Math.max(Math.abs(wheelRL), Math.abs(wheelRR));
			double maxAll = Math.max(maxFront,maxRear);

			if (maxAll > 1.0) {
				double scaleFactor = 1.0/maxAll;

				wheelFR *= scaleFactor;
				wheelFL *= scaleFactor;
				wheelRL *= scaleFactor;
				wheelRR *= scaleFactor;

			}



			// write the values to the motors.

			motorFrontRight.setPower(wheelFR);
			motorRearLeft.setPower(wheelRL);

			motorFrontLeft.setPower(wheelFL);
			motorRearRight.setPower(wheelRR);


			//telemetry.addData("Text", "*** Robot Data***");
			telemetry.addData("Time", String.format("%f", this.time));
			telemetry.addData("Wheel", String.format("%f %f %f %f", wheelFL, wheelFR, wheelRL, wheelRR));
			telemetry.addData("Left", String.format("%d/%d", motorFrontLeft.getCurrentPosition(), motorRearLeft.getCurrentPosition()));
			telemetry.addData("Right", String.format("%d/%d", motorFrontRight.getCurrentPosition(), motorRearRight.getCurrentPosition()));
			telemetry.addData("Gyro", String.format("%d ", gyroSensor.getHeading()));
			telemetry.addData("Ultra Sonic", ultrasonicSensorValue);
			telemetry.addData("ODS", odsSensorValue);
			telemetry.addData("HoldHeading",String.format("%s %f", holdHeading ? "yes" : "no",robotFaceDirection));
			telemetry.addData("PID","P=%f  I=%f  D=%f", pidP, pidI, pidD);
			telemetry.update();

			idle();
		}


	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	//@Override
	//public void stop() {

//	}


}
