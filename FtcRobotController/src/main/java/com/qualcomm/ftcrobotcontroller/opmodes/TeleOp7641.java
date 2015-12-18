/**
 * 7641 Main TeleOperation Code
 *
 *
 *"
**/
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp7641 extends OpMode{

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
    // TETRIX VALUES.
    final static double ARM_MIN_RANGE  = 0.20;
    final static double ARM_MAX_RANGE  = 0.90;
    final static double CLAW_MIN_RANGE  = 0.20;
    final static double CLAW_MAX_RANGE  = 0.7;
    final static double ZIP_OPEN = 0.5;
    final static double ZIP_CLOSED = 1.0;

	// position of the arm servo.
	double armPosition;

	// amount to change the arm servo position.
	double armDelta = 0.1;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.1;
    Boolean oldX;

	DcMotor motorRightPrimary;
	DcMotor motorRightSecondary;
	DcMotor motorLeftPrimary;
	DcMotor motorLeftSecondary;
    Servo zipServo;
	Servo personDropperServo;

    double zipPosition;
    double personPosition;
	/**
	 * Constructor
	 */
	public TeleOp7641() {

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
		motorRightPrimary = hardwareMap.dcMotor.get("motor-1");
		motorLeftPrimary = hardwareMap.dcMotor.get("motor-2");
		motorRightSecondary = hardwareMap.dcMotor.get("motor-3");
		motorLeftSecondary = hardwareMap.dcMotor.get("motor-4");
        zipServo = hardwareMap.servo.get("servo-zip");
        personDropperServo = hardwareMap.servo.get("servo-person");
        oldX = false;
// WARNING: Ugly motor direction fix ahead
        motorRightPrimary.setDirection(DcMotor.Direction.REVERSE);
        motorLeftSecondary.setDirection(DcMotor.Direction.REVERSE);

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
    public void init_loop() {

        zipServo.setPosition(ZIP_CLOSED);
        zipPosition = ZIP_CLOSED;
        personDropperServo.setPosition(0.8);
        personPosition =  0.8;

    }
	@Override
	public void loop() {
		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */
        zipServo.setPosition(zipPosition);
        personDropperServo.setPosition(personPosition);
        if (gamepad2.x && !oldX) {
            zipPosition = (zipPosition != ZIP_CLOSED) ? ZIP_CLOSED : ZIP_OPEN;
        }

        oldX = gamepad2.x;
        if (gamepad2.right_trigger > 0) {
            personPosition += gamepad2.right_trigger;
        }
        else {
            personDropperServo.setPosition(0.8);
        }
		
		if (personPosition < 0.0) {
			personPosition = 0.0;
		}

        if (personPosition > 0.8) {
            personPosition = 0.8;
        }
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
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
        motorRightPrimary.setPower(right);
        motorLeftPrimary.setPower(left);
        motorRightSecondary.setPower(right);
        motorLeftSecondary.setPower(left);

		//--------------------------------------------------------------------
		// Driver Control Features
		//--------------------------------------------------------------------



        // On left trigger push, turn left
        if (gamepad1.left_trigger > 0) {
            motorRightPrimary.setPower(gamepad1.left_trigger);
            motorRightSecondary.setPower(gamepad1.left_trigger);
			motorLeftPrimary.setPower(-gamepad1.left_trigger);
			motorLeftSecondary.setPower(-gamepad1.left_trigger);
        }

		// On right trigger push, turn right
		if (gamepad1.right_trigger > 0) {
			motorRightPrimary.setPower(-gamepad1.right_trigger);
			motorRightSecondary.setPower(-gamepad1.right_trigger);
			motorLeftPrimary.setPower(gamepad1.right_trigger);
			motorLeftSecondary.setPower(gamepad1.right_trigger);
		}


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        // telemetry.addData("distance", "Distance Travelled : " + String.format("%.2f", encoder*2240*6*3.14) + "inches");
        telemetry.addData("zip","Zip position " + zipServo.getPosition());
        telemetry.addData("person","dropper position " + personDropperServo.getPosition());

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
