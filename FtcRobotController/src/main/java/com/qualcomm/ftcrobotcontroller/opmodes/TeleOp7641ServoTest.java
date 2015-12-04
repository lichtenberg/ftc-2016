/**
 * 7641 Main TeleOperation Code
 *
 *
 *"
**/
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOp7641ServoTest extends OpMode{

	final static double ZIP_OPEN = 0.5;
	final static double ZIP_CLOSED = 1.0;
	final static double BUTTON_PRESSED = 0.0;
	final static double BUTTON_NOTPRESSED = 0.6;

	// Drive Motors

	DcMotor motorRightPrimary;
	DcMotor motorRightSecondary;
	DcMotor motorLeftPrimary;
	DcMotor motorLeftSecondary;

	// Color Sensors

	ColorSensor colorSensorFront;
	ColorSensor colorSensorBottom;

	// Servos

	Servo leftServo;
	Servo rightServo;
	Servo zipServo;

	// Button status

	Boolean oldLeft,oldRight,oldX;

	// current servo position

	double zipPosition;
	double leftPosition;
	double rightPosition;


	/**
	 * Constructor
	 */
	public TeleOp7641ServoTest() {

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
// WARNING: Ugly motor direction fix ahead
        motorRightPrimary.setDirection(DcMotor.Direction.REVERSE);
        motorLeftSecondary.setDirection(DcMotor.Direction.REVERSE);

		colorSensorBottom = hardwareMap.colorSensor.get("color-bottom");
		colorSensorBottom.setI2cAddress(0x3E);
		colorSensorBottom.enableLed(true);
		colorSensorFront = hardwareMap.colorSensor.get("color-front");
		colorSensorFront.enableLed(false);

		leftServo = hardwareMap.servo.get("servo-left");
		leftServo.setDirection(Servo.Direction.REVERSE);
		rightServo = hardwareMap.servo.get("servo-right");
		zipServo = hardwareMap.servo.get("servo-zip");

		oldLeft = false;
		oldRight = false;
		oldX = false;



	}

	@Override
	public void init_loop() {

		// To enable the LED, you need to turn it off and back on again.
		colorSensorBottom.enableLed(true);
		colorSensorBottom.enableLed(false);
		colorSensorBottom.enableLed(true);

		colorSensorFront.enableLed(false);
		zipServo.setPosition(ZIP_CLOSED);
		zipPosition = ZIP_CLOSED;
		leftPosition = BUTTON_PRESSED;
		rightPosition = BUTTON_PRESSED;


	}
	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 */

		// If the joystick button states change and go from not pressed to pressed,
		// take action.   These buttons are all toggles - press once to open, press again to close.
		if (gamepad1.x && !oldX) {
			zipPosition = (zipPosition != ZIP_CLOSED) ? ZIP_CLOSED : ZIP_OPEN;
		}

		if (gamepad1.left_bumper && !oldLeft) {
			leftPosition = (leftPosition != BUTTON_NOTPRESSED) ? BUTTON_NOTPRESSED : BUTTON_PRESSED;

		}
		if (gamepad1.right_bumper && !oldRight) {
			rightPosition = (rightPosition != BUTTON_NOTPRESSED) ? BUTTON_NOTPRESSED : BUTTON_PRESSED;

		}

		// Remember the current state of the buttons for the next time we loop.
		oldLeft = gamepad1.left_bumper;
		oldRight = gamepad1.right_bumper;
		oldX = gamepad1.x;

		// Set the servos to the positions we want them at.
		zipServo.setPosition(zipPosition);
		leftServo.setPosition(leftPosition);
		rightServo.setPosition(rightPosition);

		// tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        boolean ybutton = gamepad1.y;

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
        float encoder = motorRightPrimary.getCurrentPosition();


        // On y button push, start back wheel motors at 100% speed to get over churro bars
        if (ybutton == true) {
            motorRightPrimary.setPower(1.00);
            motorLeftPrimary.setPower(1.00);
        }



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("distance", "Revolutions: " + encoder*2240);
		telemetry.addData("zip","Zip position " + zipServo.getPosition());
		telemetry.addData("lflip","lflip position " + leftServo.getPosition());
		telemetry.addData("rflip","rflip position " + rightServo.getPosition());


		telemetry.addData("FrontColor","Front: R:"+colorSensorFront.red() + " G:"+colorSensorFront.green() + " B:" + colorSensorFront.blue() + " A:"+colorSensorFront.alpha());
		telemetry.addData("BottomColor","Bottom: R:"+colorSensorBottom.red() + " G:"+colorSensorBottom.green() + " B:" + colorSensorBottom.blue() + " A:"+colorSensorBottom.alpha());


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
