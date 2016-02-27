package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by luke on 2/26/16.
 */
public class Straighten extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotor armAngle;
    DcMotor armExtend;

    OpticalDistanceSensor distance;

    double lastDistance = 0.0;
    double lastDirection = 0.0;

    // Start out moving in one direction.
    double left = 1.0;
    double right = -1.0;

    @Override
    void init()
    {
        treadRight = hardwareMap.dcMotor.get("right");
        treadLeft = hardwareMap.dcMotor.get("left");
        treadLeft.setDirection(DcMotor.Direction.REVERSE);
        treadRight.setDirection(DcMotor.Direction.FORWARD);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armExtend = hardwareMap.dcMotor.get("arm extend");

        distance = hardwareMap.opticalDistanceSensor.get("distance");
        lastDistance = distance.getLightDetected();
    }

    @Override
    void loop()
    {
        // First try rotating left.
        treadLeft.setPower(left);
        treadRight.setPower(right);

        // If it is positive, just continue until it changes direction.
        double direction = distance.getLightDetected() - lastDistance;

        if(direction < 0.0) direction = -1.0;
        else if(direction > 0.0) direction = 1.0;
        else direction = 0.0;

        // Hmm, has our direction changed?
        if(lastDirection != direction)
        {
            // Move in the other direction
            left = -left * .5;
            right = -right * .5;

            // A direction change in the next frame must be in the opposite direction, if we don't
            // flip it lastDirection will always be different, meaning this will flip.
            lastDirection = -direction;
        }
        else
        {
            lastDirection = direction;
        }
    }
}
