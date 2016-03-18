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
enum LiftServoState
{
    Up, Middle, Down
};

public class JuniorK9TeleOp extends OpMode {

    DcMotor motorRightTread;
    DcMotor motorLeftTread;
    DcMotor motorRightWheel;
    DcMotor motorLeftWheel;
    DcMotor armMotor;
    DcMotor elbowMotor;
    DcMotor winchMotor;
    double winchPower;


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
        motorRightTread = hardwareMap.dcMotor.get("right tread");
        motorLeftTread = hardwareMap.dcMotor.get("left tread");
        motorLeftTread.setDirection(DcMotor.Direction.REVERSE);

        motorRightWheel = hardwareMap.dcMotor.get("right wheel");
        motorRightWheel.setDirection(DcMotor.Direction.REVERSE);
        motorLeftWheel = hardwareMap.dcMotor.get("left wheel");

        armMotor = hardwareMap.dcMotor.get("arm");
        elbowMotor = hardwareMap.dcMotor.get("elbow");

        winchMotor = hardwareMap.dcMotor.get("winch");
        winchPower = 0.0;
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

        // I don't even know
        float right = Range.clip(gamepad1.right_stick_y, -1, 1);
        float left = Range.clip(-gamepad1.left_stick_y, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left = (float) scaleInput(left);
        right =  (float) scaleInput(right);

        // write the values to the motors
        motorRightWheel.setPower(left);
        motorRightTread.setPower(left);

        motorLeftWheel.setPower(right);
        motorLeftTread.setPower(right);

        float arm1 = Range.clip(-gamepad2.left_stick_y, -1, 1);
        arm1 = (float) scaleInput(arm1);
        armMotor.setPower(arm1);

        float arm2 = Range.clip(gamepad2.right_stick_y, -1, 1);
        arm2 = (float) scaleInput(arm2);
        elbowMotor.setPower(arm2);

        if(gamepad2.a) winchPower = -1.0;
        else if(gamepad2.b) winchPower = 0.0;
        else if(gamepad2.y) winchPower = 1.0;

        winchMotor.setPower(winchPower);

        telemetry.addData("Motor left", left);
        telemetry.addData("Motor right", right);
        telemetry.addData("Motor arm", arm1);
        telemetry.addData("Motor elbow", arm2);
        telemetry.addData("Motor winch", winchPower);
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

}