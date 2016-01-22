/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class FreshSophK9TeleOp extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;
    DcMotor armAngle;
    DcMotor armExtend;

    double climberHighPos = 0.0;
    double climberLowPos = 0.0;

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
        treadLeft.setDirection(DcMotor.Direction.REVERSE);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armExtend = hardwareMap.dcMotor.get("arm extend");

        climberHigh = hardwareMap.servo.get("climber high");
        climberLow = hardwareMap.servo.get("climber low");
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
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        float right = (float) scaleInput(Range.clip(gamepad1.left_stick_y, -1, 1));
        float left = (float) scaleInput(Range.clip(gamepad1.right_stick_y, -1, 1));

        float armAnglePow = (float) scaleInputSlow(Range.clip(gamepad2.left_stick_y, -1, 1));
        float armExtendPow = (float) scaleInputSlow(Range.clip(gamepad2.right_stick_y, -1, 1));

        treadLeft.setPower(left);
        treadRight.setPower(right);
        armAngle.setPower(armAnglePow);
        armExtend.setPower(armExtendPow);

        if(gamepad1.dpad_up) climberHighPos = Range.clip(climberHighPos + .01, 0, 1);
        if(gamepad1.dpad_down) climberHighPos = Range.clip(climberHighPos - .01, 0, 1);
        if(gamepad1.dpad_right) climberLowPos = Range.clip(climberLowPos + .01, 0, 1);
        if(gamepad1.dpad_left) climberLowPos = Range.clip(climberLowPos - .01, 0, 1);;

        climberHigh.setPosition(climberHighPos);
        climberLow.setPosition(climberLowPos);

        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("arm angle", armAnglePow);
        telemetry.addData("arm extend", armExtendPow);
        telemetry.addData("climber high", climberHighPos);
        telemetry.addData("climber low", climberLowPos);

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
        double[] scaleArray = { 0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.15, 0.20,
                0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60, 0.70, 1.00 };

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
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     *
     * EVEN MORE PRECISE
     */
    double scaleInputSlow(double dVal)  {
        double[] scaleArray = { 0.0, 0.02, 0.03, 0.03, 0.04, 0.04, 0.06, 0.07,
                0.10, 0.10, 0.12, 0.15, 0.17, 0.20, 0.23, 0.26, .3 };

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
