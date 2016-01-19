package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Aden on 11/29/2015.
 */

/*
// A program stripped down to its bare minimum
// to reduce clutter and simplify hardwareMap config
// only uses what is actually needed
// Copy and paste the commented template
public class BetaTest extends OpMode {

    public void init() {

    }

    public void init_loop() {

    }

    public void start() {

    }

    public void loop() {

    }
}
 */
//@see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()

public class BetaTest extends BetaAuto {

    @Override
    public void loop() {
        //isRed = true;

        showTelemetry();
        //setPushButtonPosition();
        //test();
    }

    void test() {
        switch (currentStep) {
            case DELAY:
                currentStep = autoStep.FOLLOW_LINE;
                break;
            case FOLLOW_LINE:
                if (!lineFollower()) currentStep = autoStep.DROP_PERSON;
                break;
            case DROP_PERSON:
                if (!dropPerson()) currentStep = autoStep.STOP;
                break;
            case STOP:
                break;
        }
    }
}