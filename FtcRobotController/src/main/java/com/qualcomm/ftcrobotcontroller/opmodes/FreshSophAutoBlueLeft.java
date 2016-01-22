/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class FreshSophAutoBlueLeft extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;
    DcMotor armAngle;
    DcMotor armExtend;

    Servo climberHigh;
    Servo climberLow;
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

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        treadLeft = hardwareMap.dcMotor.get("right");
        treadRight = hardwareMap.dcMotor.get("left");
        treadRight.setDirection(DcMotor.Direction.REVERSE);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armExtend = hardwareMap.dcMotor.get("arm extend");

        climberHigh = hardwareMap.servo.get("climber high");
        climberLow = hardwareMap.servo.get("climber low");

        climberHigh.setPosition(1.0);
        climberLow.setPosition(1.0);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        float left = 0;
        float right = 0;
        if(time < 1.0)
        {
            left = 1.0f;
            right = 1.0f;
        }
        else if(time < 2.0)
        {
            left = 1.0f;
            right = -1.0f;
        }
        else if(time < 3.0)
        {
            left = 1.0f;
            right = 1.0f;
        }
        else if(time < 4.0)
        {
            left = 1.0f;
            right = -1.0f;
        }
        else if(time < 4.5)
        {
            left = 1.0f;
            right = 1.0f;
        }

        float armAnglePow = 0.0f;
        float armExtendPow = 0.0f;

        treadLeft.setPower(left);
        treadRight.setPower(right);
        armAngle.setPower(armAnglePow);
        armExtend.setPower(armExtendPow);

        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("arm angle", armAnglePow);
        telemetry.addData("arm extend", armExtendPow);

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