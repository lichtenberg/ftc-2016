
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class BetaAuto extends OpMode {

    FtcConfig ftcConfig = new FtcConfig();

    //
    // The member variables below represent the hardware features of the robot.
    // For each of these objects, we will retrieve information from the
    // hardwareMap.   The hardwareMap is the list of devices that you
    // named on the robot controller using the "settings" and "scan" functions
    // of the robot controller app.
    //
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    ColorSensor colorSensorB;
    ColorSensor colorSensorL;
    I2cDevice gyro;
    Servo fingerLeft;
    Servo fingerRight;
    Servo personDropper;
    Servo zipServoLeft;
    Servo zipServoRight;
    Servo blade;

    //
    // The member variables used in servos
    //
    final static double BUTTON_PRESSED = 0;
    final static double BUTTON_NOTPRESSED = 0.6;
    final static double L_ZIP_CLOSED = 0.8;
    final static double R_ZIP_CLOSED = 0.8;
    final static double PERSON_DROPPED = 0;
    final static double PERSON_NOT_DROPPED = 0.8;
    final static double BLADE_DOWN = 0;
    final static double BLADE_UP = 0.9;

    //
    // The member variables below are used for when we are executing a turn.
    //
    int destHeading;
    int degreesToTurn;
    boolean shouldTurnLeft;
    boolean gyroTurnIsRunning = false;
    ModernGyroReader gyroReader;

    //
    // The member variables below are used when we are following a line.
    //
    final static double LINE_FOLLOW_SPEED = 0.20;
    final static double OTHER_WHEEL_SPEED = -0.1;
    int blackBaseLine = 0;
    final static int LINE_ALPHA = 1;
    double foundLineTime;
    boolean lineDetected = false;
    boolean lineFollowerIsRunning = false;

    //
    // The member variables below are used when we are detecting the beacon and pushing the button.
    //
    boolean beaconLeftIsBlue = false;
    int detectBlueStreak = 0;
    int detectRedStreak = 0;

    //
    // The member variables used for delaying
    //
    boolean delayIsRunning = false;
    long sysCurTime;
    long sysStartTime;

    //
    // The member variables below describe the drive train of the motor
    //
    double wheelDiameter = 6;     // Wheel diameter in inches. //7641 Bot 6, Mentor Bot 3, was 2.75
    double gearReduction = 2;     // Amount of gear reduction in drive train //7641 Bot 2, Mentor Bot 1, was 1.0
    double encoderCountsPerRevolution = 1120; // This is the number of encoder counts per turn of the output shaft. Do not change.

    boolean droppingPerson = false;
    boolean moveDistanceIsRunning = false;
    boolean pushDebrisIsRunning = false;

    boolean isRed;
    boolean startNearMountain;
    double delayTime;
    boolean pushButton;
    FtcConfig.AutonType autonType;

    int targetPosition;
    long startOpModeTime;
    long finishedTime;

    // enum has been moved to bottom of code for easy editing

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
        motorFrontRight = hardwareMap.dcMotor.get("motor-fr");
        motorFrontLeft = hardwareMap.dcMotor.get("motor-fl");
        motorBackRight = hardwareMap.dcMotor.get("motor-br");
        motorBackLeft = hardwareMap.dcMotor.get("motor-bl");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        /*
         * Servos
         */
        fingerRight = hardwareMap.servo.get("servo-right");
        fingerLeft = hardwareMap.servo.get("servo-left");
        personDropper = hardwareMap.servo.get("servo-person");
        fingerRight.setDirection(Servo.Direction.FORWARD);
        fingerLeft.setDirection(Servo.Direction.REVERSE);
        zipServoRight = hardwareMap.servo.get("servo-rzip");
        zipServoLeft = hardwareMap.servo.get("servo-lzip");
        blade = hardwareMap.servo.get("blade");


        /*
         * Gather the sensors out of the hardware map.
         */

        colorSensorB = hardwareMap.colorSensor.get("color-front");
        colorSensorL = hardwareMap.colorSensor.get("color-bottom");
        colorSensorL.setI2cAddress(0x3E);
        colorSensorB.enableLed(false); // nope. don't enable the LED
        colorSensorL.enableLed(true);


        /*
         * Set up our gyro.   To help with the gyro we take the I2C device
         * and give it to our special ModernGyroReader class, which takes care
         * of the dirty work of reading the heading out of the gyro.
         */
        gyro = hardwareMap.i2cDevice.get("gyro-1");
        gyroReader = new ModernGyroReader(gyro);
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

        rawSensorValue = colorSensorL.alpha();
        if (rawSensorValue > blackBaseLine) {
            blackBaseLine = rawSensorValue;
        }

        //
        // Add some telemetry.   The FTC forum says you should be able to read
        // the joystick values here, but I have not been able to do so.
        //
        telemetry.addData("baseline", blackBaseLine);
        telemetry.addData("buttons", (gamepad1.x ? "X" : "") + (gamepad1.y ? "Y" : ""));

        //
        // Keep reading the gyro all the time.
        //
        gyroReader.checkGyro();
    }

    //
    // This is the line sensor follower step.
    //

    public boolean lineFollower() {
        //
        // If we have not detected the line yet, keep going
        // at 0.25 power.   Once we see the white line,
        // set lineDetected true and start following.

        if (lineFollowerIsRunning) {

            int lineSensor = colorSensorL.alpha(); // Get the amount of light detected by the sensor as an int
            //int lineSensor = distanceSensor.getLightDetectedRaw(); // Get the amount of light detected by the sensor as an int

            if (lineSensor >= (blackBaseLine + LINE_ALPHA) && !lineDetected) { // TODO add noise ignoring code
                DbgLog.msg("LINE_FOLLOW:  Detected the line");
                foundLineTime = System.currentTimeMillis();
                lineDetected = true;
            }
            if (!lineDetected) {
                // Line not detected yet. Keep going straight.
                motorFrontLeft.setPower(0.10);
                motorFrontRight.setPower(0.10);
                motorBackRight.setPower(0.10);
                motorBackLeft.setPower(0.10);
            } else if (lineDetected && (System.currentTimeMillis() - foundLineTime) >= 7000) {
                // if the line is detected and 10 seconds has passed after the line has found
                // it goes straight to DROP_PERSON
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                lineFollowerIsRunning = false;
            } else {
                // Line has been detected.   If we see the line, turn one way. (left, in the case of red team)
                // If we do not see the line, turn the other way. (right)
                // The robot will oscillate as it follows the line.
                //if (pushButton) pushTheButton(); // pushButton is from ftcConfig settings
                if (pushButton) setPushButtonPosition();
                else {
                    fingerLeft.setPosition(BUTTON_NOTPRESSED);
                    fingerRight.setPosition(BUTTON_NOTPRESSED);
                }
                // it will turn right instead of left on detecting the line while on blue team
                // not isRed (t), detect line --> left (true)
                // not isRed (t), not detect line --> right (false)
                // isRed (f), detect line --> right (false)
                // isRed (f), not detect line --> left (true)
                if (!(!isRed ^ lineSensor >= (blackBaseLine + LINE_ALPHA))) { // light detected
                //if (lineSensor >= (blackBaseLine + LINE_ALPHA)) {
                    motorFrontLeft.setPower(OTHER_WHEEL_SPEED);
                    motorFrontRight.setPower(LINE_FOLLOW_SPEED);
                    motorBackLeft.setPower(OTHER_WHEEL_SPEED);
                    motorBackRight.setPower(LINE_FOLLOW_SPEED);
                } else {
                    motorFrontLeft.setPower(LINE_FOLLOW_SPEED);
                    motorFrontRight.setPower(OTHER_WHEEL_SPEED);
                    motorBackLeft.setPower(LINE_FOLLOW_SPEED);
                    motorBackRight.setPower(OTHER_WHEEL_SPEED);

                }
            }
        } else {
            lineDetected = false;
            lineFollowerIsRunning = true;
        }
        return lineFollowerIsRunning;
    }

    public boolean pushDebris() {
        if (pushDebrisIsRunning) {
            if (colorSensorL.alpha() >= (blackBaseLine + LINE_ALPHA) && !lineDetected) { // TODO add noise ignoring code
                DbgLog.msg("LINE_FOLLOW:  Detected the line");
                lineDetected = true;
            }
            if (lineDetected) {
                while (moveDistance(10)) { }
                while (moveDistance(-12)) { }
                pushDebrisIsRunning = false;
            }
        } else {
            lineDetected = false;
            pushDebrisIsRunning = true;
        }
        return pushDebrisIsRunning;
    }

    public void setPushButtonPosition() {
        // Determines if the robot should push the left button or the right one
        // and extends the corresponding finger
        if (ifPushLeftButton() && pushButton) {
            if (fingerLeft.getPosition() > 0.55) {
                fingerLeft.setPosition(BUTTON_PRESSED);
            }
            if (fingerRight.getPosition() < 0.05) {
                fingerRight.setPosition(BUTTON_NOTPRESSED);
            }
        } else if (!ifPushLeftButton() && pushButton){
            if (fingerLeft.getPosition() < 0.05) {
                fingerLeft.setPosition(BUTTON_NOTPRESSED);
            }
            if (fingerRight.getPosition() > 0.55) {
                fingerRight.setPosition(BUTTON_PRESSED);
            }
        } else {
            fingerLeft.setPosition(BUTTON_NOTPRESSED);
            fingerRight.setPosition(BUTTON_NOTPRESSED);
        }
    }

    public boolean ifPushLeftButton() {
        // this method has two sections
        // how it works is, if it detects a blue it will check blue for 50 times
        // and measure how many times it detected red. Kind of the "purity" of the signal
        // the two sections are for detecting blue and red respectively

        if (colorSensorB.blue() > 1) { // if a blue is detected
            // resets the red streak when a blue is initially detected
            if (detectBlueStreak == 0) detectRedStreak = 0;
            // u should know this...
            detectBlueStreak++;
            // if there is less than 10 red detections within the 25 blue detection streak
            // the code is sure the detection is really blue.
            if (detectBlueStreak > 50) {
                if (detectRedStreak < 20) {
                    // determines that the beaconLeft is really blue, and resets the red streak
                    detectRedStreak = 0;
                    beaconLeftIsBlue = true;
                }
                if (detectRedStreak >= 20) {
                    // resets both streaks and start over when there is too much red in the blue streak
                    detectBlueStreak = 0;
                    detectRedStreak = 0;
                }
            }
        } else {
            if (detectRedStreak == 0) detectBlueStreak = 0;
            detectRedStreak++;
            if (detectRedStreak > 50){
                if (detectBlueStreak < 20) {
                    detectBlueStreak = 0;
                    beaconLeftIsBlue = false;
                }
                if (detectBlueStreak >= 20) {
                    detectBlueStreak = 0;
                    detectRedStreak = 0;
                }
            }
        }
        return !(beaconLeftIsBlue ^ !isRed); // check if the left beacon matches the team color XNOR (NOT(XOR))
    }

    public boolean dropPerson() {
        if (droppingPerson) {
        // its fine if the loop is empty cause it will pause there until it turns false
            while (delay(1)) { }
            personDropper.setPosition(PERSON_NOT_DROPPED);
            droppingPerson = false;
        } else {
            personDropper.setPosition(PERSON_DROPPED);
            droppingPerson = true;
        }
        return droppingPerson;
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

    public boolean gyroTurn(int howMuch) {

        // We update the gyro's currentHeading each time the loop is called.
        // It runs on its own, we query the current compass position by reading the
        // latest value that checkGyro gets from the compass.
        gyroReader.checkGyro();

        if (gyroTurnIsRunning) {
            int curHeading = gyroReader.getHeading();

            degreesToTurn = subtractHeadings(destHeading, curHeading);

            //double turnSpeed = 0.10;
            //double slowTurnSpeed = 0.04;

            double turnSpeed = 0.10;
            double slowTurnSpeed = 0.10;

            // If we need to turn

            DbgLog.msg("TURN: myHeading:" + curHeading + " dest: " + (destHeading) +
                    degreesToTurn + (shouldTurnLeft ? " LEFT" : " RIGHT"));

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
                motorFrontLeft.setPower(-turnSpeed);
                motorFrontRight.setPower(turnSpeed);
                motorBackLeft.setPower(-turnSpeed);
                motorBackRight.setPower(turnSpeed);
            } else {
                motorFrontLeft.setPower(turnSpeed);
                motorFrontRight.setPower(-turnSpeed);
                motorBackLeft.setPower(turnSpeed);
                motorBackRight.setPower(-turnSpeed);
            }
        } else {
            destHeading = normalizeHeading(gyroReader.getHeading() + howMuch);

            DbgLog.msg("STARTING TURN: amount " + howMuch + " destHeading " + destHeading);

            shouldTurnLeft = (degreesToTurn > 0);
            gyroTurnIsRunning = true;
        }
        return gyroTurnIsRunning;
    }

    // ===== moveDistance =====

    public boolean moveDistance(double distanceInInches) {
        return moveDistance(distanceInInches, 0.3);
    }
    // ^^ overloaded method, if motorPower not provided then assumes 0.3 as default

    public boolean moveDistance(double distanceInInches, double motorPower) {
        // If the "move distance" routine is running, test to see if we should
        // be stopped.

        // If the "move distance" routine is not yet running, set it to run.

        // The return value of this routine is 'true' if we are still busy.

        if (moveDistanceIsRunning) {
            if ((motorFrontRight.getCurrentPosition() >= targetPosition && distanceInInches > 0)
                    || (motorFrontRight.getCurrentPosition() <= targetPosition && distanceInInches < 0)){
            //if (Math.abs(motorFrontRight.getCurrentPosition()) >= Math.abs(motorFrontRight.getTargetPosition())) {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                DbgLog.msg("MOVE_DISTANCE: finished");
                moveDistanceIsRunning = false;
            }
        } else {
            int distanceInEncoderCounts = inchesToEncoder(distanceInInches);

            DbgLog.msg("Move:" + distanceInEncoderCounts + ';' + motorFrontLeft.getCurrentPosition() + ';' + targetPosition);

            targetPosition = (motorFrontLeft.getCurrentPosition() + distanceInEncoderCounts);
            targetPosition = (motorFrontRight.getCurrentPosition() + distanceInEncoderCounts);

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
            moveDistanceIsRunning = true;
        }
        return moveDistanceIsRunning;
    }

    // does nothing for a certain time
    // the delay is based on an idea off
    // github.com/adendabeast/Java-Timer

    public boolean delay(double secs) {
        if (delayIsRunning) {
            sysCurTime = System.currentTimeMillis();
            if (sysCurTime - sysStartTime >= secs * 1000) {
                delayIsRunning = false;
            }
        } else {
            sysStartTime = System.currentTimeMillis();
            delayIsRunning = true;
        }
        return delayIsRunning;
    }

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
    // **********
    //

    //
    // The member variables below are for holding the current "state" of our program.
    // change enum based on steps in loop
    //
    enum autoStep {
        IDLE,
        DELAY,
        FORWARD_1,
        TURN_1,
        TURN_2,
        FORWARD_2,
        FORWARD_3,
        FOLLOW_LINE,
        DROP_PERSON,
        CLIMB_MOUNTAIN,
        GET_TIME,
        STOP
    }

    autoStep currentStep = autoStep.IDLE;

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
        // the object ftcConfig's inner class variables hold the config

        isRed = ftcConfig.param.colorIsRed;
        autonType = ftcConfig.param.autonType;
        startNearMountain = ftcConfig.param.startNearMountain;
        delayTime = ftcConfig.param.delayInSec;
        pushButton = ftcConfig.param.pushButton;

        fingerLeft.setPosition(BUTTON_NOTPRESSED);
        fingerRight.setPosition(BUTTON_NOTPRESSED);
        zipServoLeft.setPosition(L_ZIP_CLOSED);
        zipServoRight.setPosition(R_ZIP_CLOSED);
        personDropper.setPosition(PERSON_NOT_DROPPED);
        blade.setPosition(BLADE_UP);
        startOpModeTime = System.currentTimeMillis();
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

        // We update the gyro's curretHeading each time the loop is called.
        // It runs on its own, we query the current compass position by reading the
        // latest value that checkGyro gets from the compass.
        gyroReader.checkGyro();

        // Do not change to switch statement
        // Or "case ftcConfig.param.autonType.* " will be red lines

        // ftcConfig.param.autonType.TEST will yield access via static message
        if (autonType == FtcConfig.AutonType.GO_FOR_BEACON) {
            beacon();
        } else if (autonType == FtcConfig.AutonType.BEACON_FLOORZONE) {
            beaconFloorZone();
        } else if (autonType == FtcConfig.AutonType.BEACON_MOUNTAIN) {
            beaconMountain();
        } else if (autonType == FtcConfig.AutonType.TEST) {
            test();
        }

        fingerRight.setDirection(Servo.Direction.FORWARD);
        fingerLeft.setDirection(Servo.Direction.REVERSE);

        showTelemetry();
    }

    public void showTelemetry() {
        telemetry.addData("TIME ELAPSED", (System.currentTimeMillis() - startOpModeTime) / 1000 + "\n");
        telemetry.addData("TIME LEFT", 30 - ((System.currentTimeMillis() - startOpModeTime) / 1000) + "\n");
        telemetry.addData("currentStep", currentStep);
        telemetry.addData("autonType", autonType);
        telemetry.addData("team", (isRed ? "red" : "blue") + "\n");
        telemetry.addData("push the left button? ", ifPushLeftButton());
        telemetry.addData("detected the line? ", colorSensorL.alpha() + " " + lineDetected);
        telemetry.addData("turn left? ", (colorSensorL.alpha() >= (blackBaseLine + LINE_ALPHA)) + "\n");
        telemetry.addData("enc", motorFrontLeft.getCurrentPosition() + " " + motorFrontRight.getCurrentPosition());
        telemetry.addData("target pos", targetPosition);
        telemetry.addData("gyro", " cur " + gyroReader.curHeading + " dest " + destHeading + " degsToTurn " + degreesToTurn + "\n");
        telemetry.addData("l-finger pos", fingerLeft.getPosition());
        telemetry.addData("r-finger pos", fingerRight.getPosition());
        telemetry.addData("personDropper pos", personDropper.getPosition());
        telemetry.addData("l-zip pos", zipServoLeft.getPosition());
        telemetry.addData("r-zip pos", zipServoRight.getPosition());
    }

    // TODO the values in the parentheses still have to be changed as the robot is tested
    // put everything that should have been in the loop method (the switch statement)
    void beacon() {
        switch (currentStep) {
            case IDLE:
                break;
            case DELAY:
                if (!delay(delayTime)) {
                    currentStep = autoStep.FORWARD_1;
                }
                break;
            case FORWARD_1:
                if (!moveDistance(70)) {
                    currentStep = autoStep.FOLLOW_LINE;
                }
                break;
            case FOLLOW_LINE: //follows line to the beacon
                if (!lineFollower()) {
                    currentStep = autoStep.DROP_PERSON;
                }
                break;
            case DROP_PERSON:
                if (!dropPerson()) {
                    currentStep = autoStep.CLIMB_MOUNTAIN;
                }
                break;
            case GET_TIME:
                finishedTime = System.currentTimeMillis();
                currentStep = autoStep.STOP;
                break;
            case STOP:
                telemetry.addData("Total time: ", (finishedTime - startOpModeTime)/1000);
                break;
        }
    }
    void beaconFloorZone() {
        switch (currentStep) {
            case IDLE:
                break;
            case DELAY:
                if (!delay(delayTime)) {
                    currentStep = autoStep.FORWARD_1;
                }
                break;
            case FORWARD_1:
                if (!moveDistance(65)) {
                    currentStep = autoStep.FOLLOW_LINE;
                }
                break;
            case FOLLOW_LINE: //follows line to the beacon
                if (!lineFollower()) {
                    currentStep = autoStep.DROP_PERSON;
                }
                break;
            case DROP_PERSON:
                if (!dropPerson()) {
                    currentStep = autoStep.FORWARD_2;
                }
                break;
            case FORWARD_2:
                if (!moveDistance(-6)) {
                    currentStep = autoStep.TURN_2;
                }
                break;
            case TURN_2:
                if (!gyroTurn(isRed ? 80 : -80)) {
                    currentStep = autoStep.FORWARD_3;
                }
                break;
            case FORWARD_3:
                if (!moveDistance(18)) {
                    blade.setPosition(BLADE_DOWN);
                    currentStep = autoStep.GET_TIME;
                }
                break;
            case GET_TIME:
                finishedTime = System.currentTimeMillis();
                currentStep = autoStep.STOP;
                break;
            case STOP:
                telemetry.addData("Total time: ", (finishedTime - startOpModeTime)/1000);
                break;
        }
    }
    void beaconMountain() {
        switch (currentStep) {
            case FORWARD_2:
                if (!moveDistance(-36)) {
                    currentStep = autoStep.TURN_2;
                }
                break;
            case TURN_2:
                if (gyroTurn(45)) {
                    currentStep = autoStep.FORWARD_3;
                }
                break;
            case FORWARD_3:
                if (!moveDistance(36)) {
                    currentStep = autoStep.GET_TIME;
                }
            case GET_TIME:
                finishedTime = System.currentTimeMillis();
                currentStep = autoStep.STOP;
                break;
            case STOP:
                telemetry.addData("Total time: ", (finishedTime - startOpModeTime)/1000);
                break;
        }
    }
    void test() {
        switch (currentStep) {
            case DELAY:
                currentStep = autoStep.FORWARD_1;
                break;
            case FORWARD_1:
                if (!moveDistance(12)) currentStep = autoStep.STOP;
                break;
            case FOLLOW_LINE:
                if (!lineFollower()) currentStep = autoStep.STOP;
                break;
            case GET_TIME:
                finishedTime = System.currentTimeMillis();
                currentStep = autoStep.STOP;
                break;
            case STOP:
                telemetry.addData("", (finishedTime - startOpModeTime) / 1000);
                break;
        }
    }
    void beaconToMountain() {
        switch (currentStep) {
            case IDLE:
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
                if (!moveDistance(18.0, 0.3)) { //TODO move how much?
                    currentStep = autoStep.TURN_2;
                }
                break;
            case TURN_2: //turn towards the mountain
                if (!gyroTurn(isRed? -135 : 135)) {
                    currentStep = autoStep.FORWARD_3;
                }
                break;
            case FORWARD_3: //move up
                if (!moveDistance(18.0, 0.3)) {
                    currentStep = autoStep.STOP;
                }
                break;
            case STOP:
                break;
        }
    }
}