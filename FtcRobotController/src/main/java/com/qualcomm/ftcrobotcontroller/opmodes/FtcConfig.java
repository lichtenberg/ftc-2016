package com.qualcomm.ftcrobotcontroller.opmodes;

// This awesome code was written by "cheer4ftc" -
// See github https://github.com/cheer4ftc/OpModeConfig

//
// It can do two things:
// - It can prompt the user for various configurations for the autonomous mode (or any OpMode for that matter)
// - it can save the current settings into a text file which will be remembered for the next match.
//

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class FtcConfig {

    private enum ConfigStep {
        TEST_GAMEPAD1,
        TEST_GAMEPAD2,
        COLOR,
        START_NEAR_MTN,
        DELAY,
        PUSH_BUTTON,
        AUTON_TYPE,
        READY;
        private static ConfigStep[] vals = values();
        public ConfigStep next() { return vals[(this.ordinal()+1) % vals.length];}
        public ConfigStep prev() { return vals[(this.ordinal()-1+vals.length) % vals.length];}
    }

    public class Param {
        boolean colorIsRed;
        int delayInSec;
        AutonType autonType;
        boolean startNearMountain;
        boolean pushButton;
    }

    public enum AutonType {
        GO_FOR_BEACON,
        TEST;
        //GO_FOR_MOUNTAIN,
        //GO_FOR_BOTH;
        private static AutonType[] vals = values();
        public AutonType next() { return vals[(this.ordinal()+1) % vals.length];}
        public AutonType prev() { return vals[(this.ordinal()-1+vals.length) % vals.length];}
    }

    // the parameters that need to be setup during configuration
    boolean gamepad1IsOK, gamepad2IsOK;

    public Param param;

    // variables used during the configuration process
    ConfigStep configStepState, currConfigStepCheck;
    boolean back1, a1, b2, y1, start1;
    boolean lastBack1, lastA1, lastB2, lastY1, lastStart1;
    private String configFileName="FtcRobotConfig.txt";

    public FtcConfig() {
        param = new Param();
    }

    public void init(Context context, OpMode opMode) {
        // setup initial configuration parameters here
        gamepad1IsOK=false;
        gamepad2IsOK=true;
        //context=hardwareMap.appContext;

        setDefaults();

        // setup initial toggle memory states for buttons used
        lastBack1=false; lastA1=false; lastB2=false; lastY1=false; lastStart1=false;
        configStepState = ConfigStep.TEST_GAMEPAD1;
    }

    public void setDefaults() {
        // declare defaults here
        param.colorIsRed=true;
        param.delayInSec=0;
        param.autonType=AutonType.GO_FOR_BEACON;
        param.pushButton=true;
        param.startNearMountain = false;
    }

    public void init_loop(Context context, OpMode opMode) {
        // read the gamepad state
        back1 = opMode.gamepad1.back || opMode.gamepad1.left_bumper;
        a1 = opMode.gamepad1.a;
        b2 = opMode.gamepad2.b;
        y1 = opMode.gamepad1.y;
        start1 = opMode.gamepad1.start || opMode.gamepad1.right_bumper;

        opMode.telemetry.clearData();

        currConfigStepCheck = ConfigStep.TEST_GAMEPAD1;
        // message to driver about state of this config parameter
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            if (!gamepad1IsOK) {
                opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "GAMEPAD1 NOT VERIFIED!!!");
            }
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            if (!gamepad1IsOK) {
                opMode.telemetry.addData("C" + currConfigStepCheck.ordinal() + "A", "Push A on Gamepad 1");
            }
            if (a1) {
                gamepad1IsOK = true;
            }
        }

        currConfigStepCheck = ConfigStep.TEST_GAMEPAD2;
        // message to driver about state of this config parameter
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            if (!gamepad2IsOK) {
                opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "GAMEPAD2 NOT VERIFIED!!!");
            }
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            if (!gamepad2IsOK) {
                opMode.telemetry.addData("C" + currConfigStepCheck.ordinal() + "A", "Push B on Gamepad 2");
            }
            if (b2) {
                gamepad2IsOK = true;
            }
        }

        currConfigStepCheck = ConfigStep.COLOR;
        // message to driver about state of this config parameter
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            if (param.colorIsRed) {
                opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "Team: Red");
            } else {
                opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "Team: Blue");
            }
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            opMode.telemetry.addData("C" + currConfigStepCheck.ordinal() + "A", "Push B for Red, X for Blue");
            if (opMode.gamepad1.x) {
                param.colorIsRed = false;
            }
            if (opMode.gamepad1.b) {
                param.colorIsRed = true;
            }
        }

        currConfigStepCheck = ConfigStep.START_NEAR_MTN;
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "Starting near mountain: " + param.startNearMountain);
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            opMode.telemetry.addData("C" + configStepState.ordinal() + "A", "Push Y for TRUE, A for FALSE");
            if (y1 && !lastY1) {
                param.startNearMountain = true;
            }
            if (a1 && !lastA1) {
                param.startNearMountain = false;
            }
        }

        currConfigStepCheck = ConfigStep.DELAY;
        // message to driver about state of this config parameter
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "Delay: " + param.delayInSec + " sec");
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            opMode.telemetry.addData("C" + configStepState.ordinal() + "A", "Push Y for +, A for -");
            if (y1 && !lastY1) {
                param.delayInSec++;
            }
            if (a1 && !lastA1) {
                param.delayInSec--;
                if (param.delayInSec < 0) {
                    param.delayInSec = 0;
                }
            }
        }

        currConfigStepCheck = ConfigStep.PUSH_BUTTON;
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "Push the button: " + param.pushButton);
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            opMode.telemetry.addData("C" + configStepState.ordinal() + "A", "Push Y for TRUE, A for FALSE");
            if (y1 && !lastY1) {
                param.pushButton = true;
            }
            if (a1 && !lastA1) {
                param.pushButton = false;
            }
        }

        currConfigStepCheck = ConfigStep.AUTON_TYPE;
        // message to driver about state of this config parameter
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal()) {
            opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "Auton: " + param.autonType.name());
        }
        // configure this parameter
        if (configStepState == currConfigStepCheck) {
            opMode.telemetry.addData("C" + configStepState.ordinal() + "A", "Push Y for +, A for -");
            if (y1 && !lastY1) {
                param.autonType = param.autonType.next();
            }
            if (a1 && !lastA1) {
                param.autonType = param.autonType.prev();
            }
        }

        currConfigStepCheck = ConfigStep.READY;
        // message to driver about state of this config parameter
        if (configStepState.ordinal() >= currConfigStepCheck.ordinal() ) {
            opMode.telemetry.addData("C" + currConfigStepCheck.ordinal(), "READY TO GO!");
        }

        if (configStepState != ConfigStep.READY) {
            opMode.telemetry.addData("D" + configStepState.ordinal(), "Push Start or Right Bumper for next option");
        }
        opMode.telemetry.addData("E" + configStepState.ordinal(), "Push Back or Left Bumper to go back");

        if (start1 && !lastStart1 && (configStepState.ordinal() < ConfigStep.READY.ordinal())) {
            configStepState = configStepState.next();
        }

        if (back1 && !lastBack1 && (configStepState.ordinal() >0) ) {
            configStepState = configStepState.prev();
        }

        // update toggle memory for next call
        lastBack1=back1;
        lastA1=a1;
        lastB2=b2;
        lastY1=y1;
        lastStart1=start1;
    }
}