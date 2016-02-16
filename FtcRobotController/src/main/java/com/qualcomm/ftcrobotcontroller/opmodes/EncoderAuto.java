/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

enum Mode
{
    Move_Arm,
    Reset_Arm
}

public class EncoderAuto extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotorController arms;
    DcMotor armAngle;
    DcMotor armExtend;

    double climberHighPos = 0.6;
    double climberLowPos = 0.28;

    double leftSidePos = 0.0;
    double rightSidePos = 0.86;

    Servo climberHigh;
    Servo climberLow;

    Servo leftSideServo;
    Servo rightSideServo;

    Mode mode;

    String error;

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
        treadRight = hardwareMap.dcMotor.get("right");
        treadLeft = hardwareMap.dcMotor.get("left");
        treadLeft.setDirection(DcMotor.Direction.REVERSE);
        treadRight.setDirection(DcMotor.Direction.FORWARD);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        setMoveMode();

        armExtend = hardwareMap.dcMotor.get("arm extend");

        climberHigh = hardwareMap.servo.get("climber high");
        climberLow = hardwareMap.servo.get("climber low");
        leftSideServo = hardwareMap.servo.get("left side servo");
        rightSideServo = hardwareMap.servo.get("right side servo");
    }

    @Override
    public void start() {
        resetStartTime();
    }

    void setMoveMode()
    {
        armAngle.setTargetPosition(armAngle.getCurrentPosition() + (int) (1440 * -1.5));
        armAngle.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        mode = Mode.Move_Arm;
    }
    void setResetMode()
    {
        armAngle.setTargetPosition(0);
        mode = Mode.Reset_Arm;
    }

    void doMove()
    {
        // Moving the encoder in a negative direction requires a positive power, which in our case
        // lifts the arm from rest position towards the back of the robot, into the air, etc.
        armAngle.setPower(.5);
        telemetry.addData("arm angle power", .5);

    }

    void doReset()
    {
        armAngle.setPower(-.5);
        telemetry.addData("arm angle power", -.5);
    }

    void updateServos()
    {
        if(gamepad1.dpad_up) climberHighPos = Range.clip(climberHighPos + .01, 0, 1);
        if(gamepad1.dpad_down) climberHighPos = Range.clip(climberHighPos - .01, 0, 1);
        if(gamepad1.dpad_right) climberLowPos = Range.clip(climberLowPos + .01, 0, 1);
        if(gamepad1.dpad_left) climberLowPos = Range.clip(climberLowPos - .01, 0, 1);

        if(gamepad2.dpad_up) rightSidePos = Range.clip(rightSidePos + .01, 0, 1);
        if(gamepad2.dpad_down) rightSidePos = Range.clip(rightSidePos - .01, 0, 1);
        if(gamepad2.dpad_right) leftSidePos = Range.clip(leftSidePos + .01, 0, 1);
        if(gamepad2.dpad_left) leftSidePos = Range.clip(leftSidePos - .01, 0, 1);

        climberHigh.setPosition(climberHighPos);
        climberLow.setPosition(climberLowPos);

        leftSideServo.setPosition(leftSidePos);
        rightSideServo.setPosition(rightSidePos);

        telemetry.addData("climber high", climberHighPos);
        telemetry.addData("climber low", climberLowPos);
        telemetry.addData("left servo pos", leftSidePos);
        telemetry.addData("right servo pos", rightSidePos);
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

        // Find motor values.
        float right = (float) scaleInput(Range.clip(gamepad1.left_stick_y, -1, 1));
        float left = (float) scaleInput(Range.clip(gamepad1.right_stick_y, -1, 1));

        treadLeft.setPower(left);
        treadRight.setPower(right);

        updateServos();

        if(time > 30.0)
        {
            error = "RAN OUT OF TIME";
        }
        telemetry.addData("time", time);

        // A sets the arm to move, if it wasn't already doing so.
        if(gamepad1.a && mode != Mode.Move_Arm)
        {
            setMoveMode();
        }
        else if(gamepad1.b && mode != Mode.Reset_Arm)
        {
            setResetMode();
        }

        // Log the mode
        // And do it
        switch (mode) {
            case Move_Arm:
                doMove();
                telemetry.addData("mode", "move");
                break;
            case Reset_Arm:
                doReset();
                telemetry.addData("mode", "reset");
                break;
        }
        // Log arm position
        telemetry.addData("arm position", armAngle.getCurrentPosition());

        // Log other motors as power, these come from the controller currently.
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("arm extend", 0.0);

        // Log error if necessary
        if(error != null) telemetry.addData("error", error);
        else telemetry.addData("error", "NO error");

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop(){
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
