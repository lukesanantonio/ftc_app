package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by luke on 2/26/16.
 */
public class SophServos {

    // Defines policy for servo names but not default position.

    Servo climberHigh;
    Servo climberLow;

    Servo leftSideServo;
    Servo rightSideServo;

    public boolean initServos(HardwareMap hardwareMap)
    {
        climberHigh = hardwareMap.servo.get("climber high");
        climberLow = hardwareMap.servo.get("climber low");
        leftSideServo = hardwareMap.servo.get("left side servo");
        rightSideServo = hardwareMap.servo.get("right side servo");

        // TODO: Catch error and potentially return false
        return true;
    }

    public boolean setValues(ServoValues values)
    {
        values.clipValues();

        climberHigh.setPosition(values.climberHighPos);
        climberLow.setPosition(values.climberLowPos);
        leftSideServo.setPosition(values.leftSidePos);
        rightSideServo.setPosition(values.rightSidePos);
        return true;
    }
}
