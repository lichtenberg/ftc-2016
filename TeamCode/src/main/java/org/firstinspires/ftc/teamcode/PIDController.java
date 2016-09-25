package org.firstinspires.ftc.teamcode;

/**
 * Created by mpl on 9/23/16.
 */
public class PIDController {
    double windupGuard;     // "Wind up" guard.  Max error value for this loop
    double pGain;
    double iGain;
    double dGain;
    double prevError;
    double intError;
    double control;


    PIDController(double P, double I, double D)
    {
        pGain = P;
        iGain = I;
        dGain = D;
        prevError = 0;
        intError = 0;
        control = 0;
        windupGuard = 0;
    }

    void pidSetParams(double P,double I,double D)
    {
        pGain = P;
        iGain = I;
        dGain = D;
    }

    double pidUpdate(double curError,double dt) {
        double diff;
        double pTerm, iTerm, dTerm;

        intError = intError + (curError * dt);

        // Bound the error by windupGuard as necessary
        if (windupGuard != 0) {
            if (intError < -windupGuard) {
                intError = -windupGuard;
            } else if (intError > windupGuard) {
                intError = windupGuard;
            }
        }

        // OK, calculate the difference and the scaling.
        diff = (curError - prevError) / dt;
        pTerm = pGain * curError;
        iTerm = iGain * intError;
        dTerm = dGain * diff;

        // add it all up
        control = pTerm + iTerm + dTerm;

        prevError = curError;

        return control;
    }

    void pidReset()
    {
        prevError = 0;
        intError = 0;
    }
}
