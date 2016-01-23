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

    // TETRIX VALUES.
    final static double ZIP_OPEN = 0.5;
    final static double ZIP_CLOSED = 1.0;

    boolean oldB;
	boolean oldX;
	DcMotor motorRightPrimary;
	DcMotor motorRightSecondary;
	DcMotor motorLeftPrimary;
	DcMotor motorLeftSecondary;
    Servo zipServoRight;
	Servo zipServoLeft;
	Servo personDropperServo;
    Servo blade;


    double zipPositionRight;
    double zipPositionLeft;
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
		motorRightPrimary = hardwareMap.dcMotor.get("motor-fr");
		motorLeftPrimary = hardwareMap.dcMotor.get("motor-fl");

		motorRightSecondary = hardwareMap.dcMotor.get("motor-br");
		motorLeftSecondary = hardwareMap.dcMotor.get("motor-bl");
        zipServoRight = hardwareMap.servo.get("servo-rzip");
		zipServoLeft = hardwareMap.servo.get("servo-lzip");
        personDropperServo = hardwareMap.servo.get("servo-person");
        blade = hardwareMap.servo.get("blade");
        blade.setPosition(0.9);

        oldB = false;
		oldX = false;
        motorLeftPrimary.setDirection(DcMotor.Direction.REVERSE);
        motorLeftSecondary.setDirection(DcMotor.Direction.REVERSE);
	}

    public void init_loop() {


    }
	@Override
	public void loop() {


        if (gamepad2.x) {
                blade.setPosition(0.0);
        }
        else {
            blade.setPosition(0.9);
        }

        if (gamepad2.right_trigger > 0){
            zipServoRight.setPosition(-gamepad2.right_trigger);
        }
        else {
            zipServoRight.setPosition(1.0);
        }

        if (gamepad2.left_trigger > 0){
            zipServoLeft.setPosition(gamepad2.left_trigger);
        }
        else {
            zipServoLeft.setPosition(0.0);
        }

        // Person Dropper
        if (gamepad2.y) {
            personDropperServo.setPosition(0.0);
        }
        else {
            personDropperServo.setPosition(0.8);
        }


        // tank drive
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

        if (gamepad1.a){
            motorRightPrimary.setPower(1.0);
            motorRightSecondary.setPower(1.0);
            motorLeftPrimary.setPower(1.0);
            motorLeftSecondary.setPower(1.0);
        }

        if (gamepad1.b){
            motorRightPrimary.setPower(0.5);
            motorRightSecondary.setPower(0.5);
            motorLeftPrimary.setPower(0.5);
            motorLeftSecondary.setPower(0.5);
        }

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

        // telemetry.addData("distance", "Distance Travelled : " + String.format("%.2f", encoder*2240*6*3.14) + "inches");
        telemetry.addData("rzip","Right Zip Position " + zipServoRight.getPosition());
		telemetry.addData("lzip","Left Zip Position " + zipServoLeft.getPosition());
        telemetry.addData("person","Dropper Position " + personDropperServo.getPosition());

    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
        personDropperServo.setPosition(0.8);
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