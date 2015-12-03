
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;




public class BetaAuto extends OpMode {

    FtcConfig ftcConfig = new FtcConfig();

    //
    // The member variables below represent the hardware features of the robot.
    // For each of these objects, we will retrieve information from the
    // hardwareMap.   The hardwareMap is the list of devices that you
    // named on the robot controller using the "settings" and "scan" functions
    // of the robot controller app.

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    ColorSensor colorSensorB;
    ColorSensor colorSensorL;
    //OpticalDistanceSensor distanceSensor;
    I2cDevice gyro;
    TouchSensor touchSensor;
    Servo fingerLeft;
    Servo fingerRight;

    double fingerLeftDefaultPos = 0.05;
    double fingerRightDefaultPos = 0.05;

    //
    // The member variables below are used for when we are executing a turn.
    //
    int destHeading;
    int degreesToTurn;
    boolean shouldTurnLeft;
    Boolean gyroTurnIsRunning = false;
    ModernGyroReader gyroReader;

    //
    // The member variables below are used when we are following a line.
    //

    int blackBaseLine = 0;
    Boolean lineDetected = false;
    Boolean lineFollowerIsRunning = false;

    //
    // The member variables below are used when we are pushing the beacon button.
    //
    Boolean buttonIsPushed = false;

    //
    // The member variable used for delaying
    //
    Boolean delayIsRunning;
    double startDelayTime;
    double stopDelayTime;

    //
    // The member variables below describe the drive train of the motor
    //
    double wheelDiameter = 6;     // Wheel diameter in inches. //7641 Bot 6, Mentor Bot 3, was 2.75
    double gearReduction = 2;     // Amount of gear reduction in drive train //7641 Bot 2, Mentor Bot 1, was 1.0
    double encoderCountsPerRevolution = 1120; // This is the number of encoder counts per turn of the output shaft. Do not change.

    Boolean moveDistanceIsRunning = false;
    int moveDistanceDelay = 0;



    //
    // The member variables below are for holding the current "state" of our program.
    //

    /*
    //enum has been moved to bottom of code for easy editing
    enum autoStep {
        IDLE,
        FORWARD_1,
        TURN_1,
        TURN_2,
        FORWARD_2,
        STOP
    };


    autoStep currentStep = autoStep.IDLE;
    */


    /**
     * Constructor
     */
    public BetaAuto() {

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

        ftcConfig.init(hardwareMap.appContext, this); // Do not comment out
		
		/*
		 *   Gather motors out of the hardware map.
		 */
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_2");
        motorBackRight = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Servos
         */
        fingerRight = hardwareMap.servo.get("servo_1");
        fingerLeft = hardwareMap.servo.get("servo_2");
        fingerRight.setDirection(Servo.Direction.FORWARD);
        fingerLeft.setDirection(Servo.Direction.REVERSE);

        /*
         * Gather the sensors out of the hardware map.
         */
        // TODO
        colorSensorB = hardwareMap.colorSensor.get("color1");
        //colorSensorL = hardwareMap.colorSensor.get("color2");
        //distanceSensor = hardwareMap.opticalDistanceSensor.get("dist1");

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

        colorSensorB.enableLed(true);
        //colorSensorL.enableLed(true);


    }


    //
    // normalizeHeading(heading)
    //
    // Force a heading to fit within the range 0..359.   For example,
    // if the heading is 370 degrees, that's really the same as 10 degrees.
    // If the heading is -10 degrees, that's really 350.
    //
    private int normalizeHeading(int heading) {
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
    private int subtractHeadings(int a, int b) {
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
    private int inchesToEncoder(double inches) {
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
    public void init_loop() {
        int rawSensorValue;

        // After the INIT button is pressed, this routine gets called over and over
        // in a loop.

        ftcConfig.init_loop(hardwareMap.appContext, this);

        // TODO
        rawSensorValue = 0;
        //rawSensorValue = distanceSensor.getLightDetectedRaw();
        //rawSensorValue = colorSensorL.alpha();
        if (rawSensorValue > blackBaseLine) {
            blackBaseLine = rawSensorValue;
        }

        //
        // Add some telemetry.   The FTC forum says you should be able to read
        // the joystick values here, but I have not been able to do so.
        //
        telemetry.addData("baseline", blackBaseLine);
        telemetry.addData("buttons", (gamepad1.x ? "X" : "") + (gamepad1.y ? "Y" : ""));
        telemetry.addData("touch", touchSensor.isPressed() ? "Touch" : "no_touch");


        //
        // Keep reading the gyro all the time.
        //

        gyroReader.checkGyro();

    }


    //
    // Although this example is for autonomous mode, when we reach the end this particular
    // program lets you continue with teleop.  In a real competition, this would not be allowed.
    //
    private void teleopMode() {
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
        motorFrontRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackRight.setPower(right);
        motorBackLeft.setPower(left);
    }


    //
    // This is the line sensor follower step.
    //
    // TODO change for color sensor
    protected Boolean lineFollower() {
        //
        // If we have not detected the line yet, keep going
        // at 0.25 power.   Once we see the white line,
        // set lineDetected true and start following.

        motorFrontLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);

        if (lineFollowerIsRunning) {

            // TODO
            int lineSensor = 0;
            //int lineSensor = colorSensorL.alpha(); // Get the amount of light detected by the sensor as an int
            //int lineSensor = distanceSensor.getLightDetectedRaw(); // Get the amount of light detected by the sensor as an int

            if (lineSensor > (blackBaseLine + 50)) {
                if (!lineDetected) {
                    DbgLog.msg("LINE_FOLLOW:  Detected the line");

                    lineDetected = true;
                }
            }

            if (!lineDetected) {
                // Line not detected yet.   Keep going straight.
                motorFrontLeft.setPower(0.20);
                motorFrontRight.setPower(0.20);
                motorBackRight.setPower(0.20);
                motorBackLeft.setPower(0.20);
            } else {

                // Line has been detected.   If we see the line, turn one way.
                // If we do not see the line, turn the other way.
                // The robot will oscillate as it follows the line.

                if (lineSensor > (blackBaseLine + 50)) { // light detected
                    lineDetected = true;
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0.15);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0.15);
                } else {
                    motorFrontLeft.setPower(0.15);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0.15);
                    motorBackRight.setPower(0);
                }

            }
            //
            // If we hit the wall, we will use our touch sensor to stop this step.
            // We will monitor the touch sensor even if we haven't found the line yet,
            // just in case we miss the line entirely and slam into the wall.
            //

            if (touchSensor.isPressed()) {
                DbgLog.msg("LINE_FOLLOW:  touch sensor activated, stopping");

                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
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

    protected Boolean gyroTurn(int howMuch) {
        motorFrontLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);

        if (gyroTurnIsRunning) {
            int curHeading = gyroReader.getHeading();

            degreesToTurn = subtractHeadings(destHeading, curHeading);

            //double turnSpeed = 0.10;
            //double slowTurnSpeed = 0.04;

            double turnSpeed = 0.10;
            double slowTurnSpeed = 0.10;
            /*
            if (degreesToTurn > 0) {
                shouldTurnLeft = true;
            } else {
                shouldTurnLeft = false;
            }
            */
            //shouldTurnLeft = false;

            // If we need to turn

            DbgLog.msg("TURN: myHeading:" + curHeading + " dest " + (destHeading) + " left:" + degreesToTurn + (shouldTurnLeft ? " LEFT" : " RIGHT"));

            if (Math.abs(degreesToTurn) <= 20) {
                turnSpeed = slowTurnSpeed;
            }

            if (Math.abs(degreesToTurn) <= 1) {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                gyroTurnIsRunning = false;

            } else if (shouldTurnLeft) {
                motorFrontLeft.setPower(-turnSpeed);//-
                motorFrontRight.setPower(turnSpeed);
                motorBackLeft.setPower(-turnSpeed);
                motorBackRight.setPower(turnSpeed);

            } else {
                motorFrontLeft.setPower(turnSpeed);
                motorFrontRight.setPower(-turnSpeed);//-
                motorBackLeft.setPower(turnSpeed);
                motorBackRight.setPower(-turnSpeed);

            }
        } else {
            destHeading = normalizeHeading(gyroReader.getHeading() + howMuch);

            motorFrontLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
            motorFrontRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);

            DbgLog.msg("STARTING TURN: amount " + howMuch + " destHeading " + destHeading);

            shouldTurnLeft = !(degreesToTurn > 0); // (d... > 0) is a boolean value
            //if (degreesToTurn > 0) {
            //    shouldTurnLeft
            //}
            gyroTurnIsRunning = true;
        }

        return gyroTurnIsRunning;
    }

    public Boolean moveDistance(double distanceInInches, double motorPower) {
        // If the "move distance" routine is running, test to see if we should
        // be stopped.

        // If the "move distance" routine is not yet running, set it to run.

        // The return value of this routine is 'true' if we are still busy.
        motorFrontLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);

        if (moveDistanceIsRunning) {
            //moveDistanceDelay++; // added to fix isBusy() bug
            //DbgLog.msg("busy: " + moveDistanceDelay + ';' + motorFrontLeft.isBusy() + ';' + motorFrontRight.isBusy());
            //DbgLog.msg("enc " + motorFrontLeft.getCurrentPosition() + " " + motorFrontLeft.getCurrentPosition());

            // WARNING: HAPHAZARDLY SLAPPED-TOGETHER TEMPORARILY SKETCHY FIX
            // FOR RAGE QUIT-INDUCING BUGS AHEAD!!! PROCEED AT YOUR OWN RISK.

            // ***RESOLVED***
            // Seems like the motorFrontXxx.getCurrentPosition() values are +- inverted, so add a negative to getCurrentPosition()
            // Update: the RIGHT motors were reversed instead of the left ones, so that has been corrected
            // The reasoning: both left and right motors are displaying that they are moving backwards
            // so this is why they have been displaying negative values for currentPosition

            // ***RESOLVED***
            // Another bug: the motors still twitch and move a bit even after the motors have done their job
            // This has been observed in the motorFrontRight on the MentorBot, not the FrontLeft
            // Reading the telemetry data, the cause for this is the setChannelMode(RunMode.*) is not changing
            // Seems like we never needed the RunMode.RUN_TO_POSITION after all
            // Since there is already code that stops the robot, we don't need to mess with RunMode or isBusy()

            // ^^^ DUDE U WOT M8 ??!! ^^^
            // Turns out the problem is NOT ADDING "break;" to the end of each step in the switch statement of the loop code

            if ((motorFrontRight.getCurrentPosition() >= (motorFrontRight.getTargetPosition()) && distanceInInches > 0) || (motorFrontRight.getCurrentPosition() <= (motorFrontRight.getTargetPosition()) && distanceInInches < 0)){
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
                    motorFrontRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
                    //DbgLog.msg("enc " + motorFrontLeft.getCurrentPosition() + " " + motorFrontLeft.getTargetPosition());
                    //DbgLog.msg("busy: " + moveDistanceDelay + ';' + motorFrontLeft.isBusy() + ';' + motorFrontRight.isBusy());
                    //motorBackLeft.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
                    //motorBackRight.setChannelMode(RunMode.RUN_WITHOUT_ENCODERS);
                    telemetry.addData("MOVE_DISTANCE: finished", 0);
                    DbgLog.msg("MOVE_DISTANCE: finished");

                moveDistanceIsRunning = false;
            }
        } else {
            int distanceInEncoderCounts = inchesToEncoder(distanceInInches);

            //motorFrontLeft.setChannelMode(RunMode.RUN_TO_POSITION);
            //motorFrontRight.setChannelMode(RunMode.RUN_TO_POSITION);
            //motorFrontLeft.setChannelMode(RunMode.RUN_USING_ENCODERS);
            //motorFrontRight.setChannelMode(RunMode.RUN_USING_ENCODERS);

            DbgLog.msg("Move:" + distanceInEncoderCounts + ';' + motorFrontLeft.getCurrentPosition() + ';' + motorFrontLeft.getTargetPosition());

            motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + distanceInEncoderCounts);
            motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + distanceInEncoderCounts);

            // reverse motor power if moving backward
            if (distanceInInches > 0) {
                motorFrontLeft.setPower(motorPower);
                motorFrontRight.setPower(motorPower);
                motorBackLeft.setPower(motorPower);
                motorBackRight.setPower(motorPower);
            } else if (distanceInInches < 0) {
                motorFrontLeft.setPower(-motorPower);
                motorFrontRight.setPower(-motorPower);
                motorBackLeft.setPower(-motorPower);
                motorBackRight.setPower(-motorPower);
            }
            //DbgLog.msg("MOVE_DISTANCE: distance " + distanceInInches + " power " + motorPower + ';' + distanceInEncoderCounts + ';' + motorFrontLeft.getTargetPosition());
            //DbgLog.msg("Move:" + distanceInEncoderCounts + ';' + motorFrontLeft.getCurrentPosition() + ';' + motorFrontLeft.getTargetPosition());
            //DbgLog.msg("busy xxx: " + motorFrontLeft.isBusy() + ';' + motorFrontRight.isBusy());

            //moveDistanceDelay = 0;
            moveDistanceIsRunning = true;
        }

        return moveDistanceIsRunning;

    }

    Boolean beaconLeftIsRed;

    //TODO wait for the hardware people and the girls to complete
    public Boolean pushButton() {
        if (true) {
            buttonIsPushed = false;
        } else {
            buttonIsPushed = true;
        }
        return buttonIsPushed;
    }

    protected Boolean delay(double secs) {
        if (delayIsRunning) {
            if (startDelayTime >= stopDelayTime) {
                delayIsRunning = false;
            }
        } else {
            startDelayTime = time;
            stopDelayTime = time + secs;
            delayIsRunning = true;
        }
        return delayIsRunning;
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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

    /*
    Boolean moveServoIsRunning = false;
    public Boolean moveServoLeft (double posLeft) {
        if (moveServoIsRunning) {
            if (fingerLeft.getPosition() < (posLeft + 0.03) || fingerLeft.getPosition() > (posLeft - 0.03)) {
                moveServoIsRunning = false;
            } else {

            }
        } else {
            fingerLeft.setPosition(posLeft);
            moveServoIsRunning = true;
        }
        return moveServoIsRunning;
    }

    public Boolean moveServoRight (double posRight) {
        if (moveServoIsRunning) {
            if (fingerRight.getPosition() < (posRight + 0.03) || fingerRight.getPosition() > (posRight - 0.03)) {
                moveServoIsRunning = false;
            } else {

            }
        } else {
            fingerRight.setPosition(posRight);
            moveServoIsRunning = true;
        }
        return moveServoIsRunning;
    }
    */

    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    // **********
    // CHANGE ANYTHING BELOW THIS DEPENDING ON THE OBJECTIVES.
    // ANYTHING ABOVE THIS COMMENT SHOULD NOT BE CHANGED UNLESS NECESSARY
    // OR PROGRAM WILL NOT FUNCTION PROPERLY.
    // **********
    //

    //change enum based on steps in loop
    enum autoStep {
        IDLE,
        DELAY,
        FORWARD_1,
        TURN_1,
        TURN_2,
        FORWARD_2,
        FORWARD_3,
        FOLLOW_LINE,
        PUSH_BUTTON,
        STOP
    }

    autoStep currentStep = autoStep.IDLE;

    boolean isRed;
    boolean startNearMountain;
    int delayTime;
    FtcConfig.AutonType autonType;

    //
    // The start() method below is called when you press the triangle start button on the
    // driver station.   This routine is called only once.
    //

    public void start() {
        // Erase all the telemetry data.

        telemetry.clearData();


        // Set the current autonomous step to our first step, DELAY.
        currentStep = autoStep.DELAY;

        // Get config settings
        // the object ftcConfig's subclass variables hold the config
        isRed = ftcConfig.param.colorIsRed;
        autonType = ftcConfig.param.autonType;
        startNearMountain = ftcConfig.param.startNearMountain;
        delayTime = ftcConfig.param.delayInSec;
        //fingerRight.setPosition(0.1);
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
    //@Override
    public void loop() {

        // We update the gyro's currentHeading each time the loop is called.
        // It runs on its own, we query the current compass position by reading the
        // latest value that checkGyro gets from the compass.
        gyroReader.checkGyro();

        // Do not change to switch statement
        // Or "case ftcConfig.param.autonType.* " will be red lines

        if (autonType == FtcConfig.AutonType.GO_FOR_BEACON) {
            beacon();
        } else if (autonType == FtcConfig.AutonType.GO_FOR_MOUNTAIN) {
            mountain();
        } else if (autonType == FtcConfig.AutonType.GO_FOR_BOTH) {
            beacon();
            beaconToMountain();
        } else if (autonType == FtcConfig.AutonType.TEST) { //ftcConfig.param.autonType.TEST will yield access via static message
            //test();
        }
        testServo();
        //fingerRight.setDirection(Servo.Direction.REVERSE);
        //fingerLeft.setDirection(Servo.Direction.FORWARD);
        fingerRight.setDirection(Servo.Direction.FORWARD);
        fingerLeft.setDirection(Servo.Direction.REVERSE);
        //fingerRight.setPosition(fingerRightDefaultPos);
        //fingerLeft.setPosition(fingerLeftDefaultPos);
        //fingerRight.setPosition(0.2);
        //fingerLeft.setPosition(0.2);
        //fingerRight.setDirection(Servo.Direction.FORWARD);
        //fingerLeft.setDirection(Servo.Direction.REVERSE);
        //fingerRight.setPosition(0.1);
        //fingerLeft.setPosition(0.1);

        telemetry.addData("enc: ", motorFrontLeft.getCurrentPosition() + " " + motorFrontRight.getCurrentPosition() + " " + motorFrontLeft.getTargetPosition());
        telemetry.addData("gyro: ", " cur " + gyroReader.curHeading + " dest " + destHeading + " degsToTurn " + degreesToTurn);
        telemetry.addData("channelMode: ", motorFrontLeft.getChannelMode());
        telemetry.addData("currentStep: ", currentStep);
        telemetry.addData("autonType", autonType);
        telemetry.addData("random stuff", fingerLeft.getPosition() + " " + fingerRight.getPosition());
    }

    public void beacon() {
        //put everything that should have been in the loop method (the switch statement)
        //TODO the values in the parentheses still have to be changed as the robot is tested
        switch (currentStep) {
            case IDLE:
                break;
            case DELAY:
                if (!delay((double)delayTime)) {
                    currentStep = autoStep.FORWARD_1;
                }
                break;
            case FORWARD_1: //leave spawn
                if (!moveDistance(12, 0.3)) {  //12 in per sec
                    currentStep = autoStep.TURN_1;
                }
                break;
            case TURN_1:
                if (isRed) {
                    if (!gyroTurn(45)) { //spins left 45 degs
                        currentStep = autoStep.FORWARD_2;
                    }
                } else if (!isRed) {
                    if (!gyroTurn(-45)) {
                        currentStep = autoStep.FORWARD_2;
                    }
                }
                break;
            case FORWARD_2:
                if (!moveDistance(33.6, 0.3)) {
                    currentStep = autoStep.TURN_2;
                }
                break;
            case TURN_2: //turns towards beacon, prepare for line follow
                //depends on the initial place of the robot
                if (!startNearMountain) {
                    if (isRed) {
                        if (!gyroTurn(25)) {
                            currentStep = autoStep.FORWARD_3;
                        }
                    } else if (!isRed) {
                        if (!gyroTurn(-25)) {
                            currentStep = autoStep.FORWARD_3;
                        }
                    }
                }
                break;
            case FORWARD_3: //proceed to beacon
                if (!moveDistance(12.0, 0.3)) {
                    currentStep = autoStep.FOLLOW_LINE;
                }
                break;
            case FOLLOW_LINE: //follows line to the beacon
                if (!lineFollower()) {
                    currentStep = autoStep.PUSH_BUTTON;
                }
                break;
            case PUSH_BUTTON:
                if (!pushButton()) { //TODO pushButton has not been coded
                    currentStep = autoStep.STOP;
                }
                break;
            case STOP:
                //teleopMode();
                break;
        }
    }

    public void beaconToMountain() {
        switch (currentStep) {
            case IDLE:
                break;
            case DELAY:
                if (!delay((double)delayTime)) {
                    currentStep = autoStep.FORWARD_1;
                }
                break;
            case FORWARD_1: //back up from beacon
                if (!moveDistance(-3.0, 0.3)) {
                    currentStep = autoStep.TURN_1;
                }
                break;
            case TURN_1: //turn 180 degs
                if (!gyroTurn(180)) {
                    currentStep = autoStep.FORWARD_2;
                }
                break;
            case FORWARD_2: //continue moving away from beacon
                if (!moveDistance(12.0, 0.3)) { //TODO move how much?
                    currentStep = autoStep.TURN_2;
                }
                break;
            case TURN_2: //turn towards the mountain
                if (isRed) {
                    if (!gyroTurn(135)) { //TODO may have to be changed
                        currentStep = autoStep.FORWARD_3;
                    }
                } else if (!isRed) {
                    if (!gyroTurn(-135)) {
                        currentStep = autoStep.FORWARD_3;
                    }
                }
                break;
            case FORWARD_3: //move up
                if (!moveDistance(12.0, 0.3)) {
                    currentStep = autoStep.STOP;
                }
                break;
            case STOP:
                //teleopMode();
                break;
        }
    }
    //TODO work still in progress
    public void mountain() {
        switch (currentStep) {
            case IDLE:
                break;
            case DELAY:
                if (!delay((double)delayTime)) {
                    currentStep = autoStep.FORWARD_1;
                }
                break;
            case FORWARD_1: //leave spawn
                if (!moveDistance(12.0, 0.3)) {
                    currentStep = autoStep.TURN_1;
                }
                break;
            case TURN_1:
                if (isRed) {
                    if (!gyroTurn(45)) {
                        currentStep = autoStep.FORWARD_2;
                    }
                } else if (!isRed) {
                    if (!gyroTurn(-45)) {
                        currentStep = autoStep.FORWARD_2;
                    }
                }
                break;
            case FORWARD_2:
                if (startNearMountain) {
                    if (!moveDistance(22.32, 0.3)) {
                        currentStep = autoStep.TURN_2;
                    }
                } else if (!startNearMountain) {
                    if (!moveDistance(33.6, 0.3)) {
                        currentStep = autoStep.TURN_2;
                    }
                }
                break;
            case TURN_2: //turn towards the mountain
                if (isRed) {
                    if (!gyroTurn(90)) {
                        currentStep = autoStep.FORWARD_3;
                    }
                } else if (!isRed) {
                    if (!gyroTurn(-90)) {
                        currentStep = autoStep.FORWARD_3;
                    }
                }
                break;
            case FORWARD_3: //move up the mountain
                if (startNearMountain) {
                    if (!moveDistance(24, 0.3)) {
                        currentStep = autoStep.STOP;
                    }
                } else if (!startNearMountain) {
                    if (!moveDistance(36, 0.3)) {
                        currentStep = autoStep.STOP;
                    }
                }
                break;
            case STOP:
                //teleopMode();
                break;
        }
    }

    public void test() {
        switch (currentStep) {
            case IDLE:
                break;
            case DELAY:
                if (true) {
                    currentStep = autoStep.FORWARD_1;
                }
                break;
            case FORWARD_1:
                if (!moveDistance(12, 0.3)) { //moves 2ft
                    currentStep = autoStep.FORWARD_2;
                }
                break;
            case FORWARD_2:
                if (!moveDistance(-12, 0.3)) {
                    currentStep = autoStep.TURN_1;
                }
                break;
            case TURN_1:
                if(!gyroTurn(90)) {
                    currentStep = autoStep.TURN_2;
                }
                break;
            case TURN_2:
                if(!gyroTurn(-90)) {
                    currentStep = autoStep.STOP;
                }
                break;
            case STOP:
                //teleopMode();
                break;
        }
    }

    public void testServo() {
        switch (currentStep) {
            case DELAY:
                currentStep = autoStep.FORWARD_1;
                break;
            case FORWARD_1:
                fingerLeft.setPosition(0);
                currentStep = autoStep.FORWARD_2;

                break;
            case FORWARD_2:
                fingerLeft.setPosition(0.3);
                currentStep = autoStep.STOP;

                break;
            case TURN_1:
                fingerLeft.setPosition(0);
                currentStep = autoStep.TURN_2;

                break;
            case TURN_2:
                fingerLeft.setPosition(0.7);
                currentStep = autoStep.STOP;

                break;
            case STOP:
                break;
        }
    }
    //
    // This method is called when the OpMode is stopped.
    //

    public void stop() {
        //well it means stop.
    }
}