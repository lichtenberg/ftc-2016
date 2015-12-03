package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Object;

/**
 * Created by Aden on 11/29/2015.
 */

/*
// A program stripped down to its bare minimum
// to reduce clutter and simplify hardwareMap config
public class BetaTest extends OpMode {
    //@see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()

    public void init() {

    }

    public void init_loop() {

    }

    public void loop() {

    }
}
 */
// A program stripped down to its bare minimum
// to reduce clutter and simplify hardwareMap config
public class BetaTest extends OpMode {
    //@see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()

    Servo fingerLeft;

    Boolean delayIsRunning;
    double startDelayTime;
    double stopDelayTime;

    public void init() {
        fingerLeft = hardwareMap.servo.get("servo1");
    }

    public void init_loop() {

    }

    public void loop() {
        telemetry.addData("random stuff", fingerLeft.getPosition());
        test();
    }

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
    //com.qualcomm.robotcore.util.ElapsedTime();
    public void test(){
        fingerLeft.setPosition(0);
        delay(0.2);
        fingerLeft.setPosition(0.3);
        delay(0.2);
        fingerLeft.setPosition(0);
        delay(0.2);
        fingerLeft.setPosition(0.7);
        delay(0.2);
        fingerLeft.setPosition(0);
    }
    public void testServo() {

        switch (currentStep) {
            case IDLE:
                currentStep = autoStep.FORWARD_1;
                break;
            case DELAY:
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

}

/*
    /*
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     /

ColorSensor colorSensorL;

int blackBaseLine;
    //DcMotor motorFrontRight;
    //DcMotor motorFrontLeft;


    public void init() {
        //motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void init_loop() {

    }

    public void loop() {
        //move the robot
        //get the sensor values
        //whatever robot does
        //goes here
        telemetry.addData("TEST PROGRAM: ", "motor encoder testing");
        telemetry.addData("channel mode: ", motorFrontLeft.getChannelMode() + ";" + motorFrontRight.getChannelMode());
        telemetry.addData("enc: ", motorFrontLeft.getCurrentPosition() + ";" + motorFrontRight.getCurrentPosition());
        motorFrontLeft.setPower(0.3);
        motorFrontRight.setPower(0.3);

    }
*/

// COLOR SENSOR TESTING
/*
ColorSensor colorSensorL;

    int blackBaseLine;
    public void init() {
        colorSensorL = hardwareMap.colorSensor.get("color2");
        colorSensorL.enableLed(true);
    }
    public void init_loop() {
        int rawSensorValue;
        rawSensorValue = colorSensorL.alpha();
        if (rawSensorValue > blackBaseLine) {
            blackBaseLine = rawSensorValue;
        }
        telemetry.addData("TEST PROGRAM: ", "blackBaseLine and RawSensorValue");
        telemetry.addData("blackBaseLine: ", blackBaseLine);
    }
    public void loop() {
        int lineSensor = colorSensorL.alpha();
        telemetry.addData("TEST PROGRAM: ", "color sensor testing");
        telemetry.addData("alpha (amount of white light): ", lineSensor);
    }
*/
/*
ColorSensor colorSensorL;

int blackBaseLine;
DcMotor motorFrontRight;
DcMotor motorFrontLeft;

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     *
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
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
    public void init_loop() {

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
    public void loop() {
        //move the robot
        //get the sensor values
        //whatever robot does
        //goes here
        telemetry.addData("TEST PROGRAM: ", "motor encoder testing");
        telemetry.addData("channel mode: ", motorFrontLeft.getChannelMode() + ";" + motorFrontRight.getChannelMode());
        telemetry.addData("enc: ", motorFrontLeft.getCurrentPosition() + ";" + motorFrontRight.getCurrentPosition());
        motorFrontLeft.setPower(0.3);
        motorFrontRight.setPower(0.3);

    }
*/