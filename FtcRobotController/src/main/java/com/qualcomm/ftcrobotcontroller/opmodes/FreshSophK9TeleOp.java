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
import com.qualcomm.robotcore.robocol.Telemetry;
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

class LinearSpeadModifier {

    public float scale;

    public float reqSpeed;

    public float curPower;

    public int curPosition;
    public int lastPosition;

    // Returns motor power.
    float getMotorPower(Telemetry tm)
    {
        // What is the position of the motor currently?
        // Positive movement of the motor should correspond to a positive speed member above. Use
        // a negative scale to reverse the motor direction.
        int posChange = curPosition - lastPosition;

        // Figure out what speed value we just experienced was
        float instSpeed = posChange / scale;

        tm.addData("zz instSpeed", instSpeed);

        // Using past measurements might be useful and may be necessarily, but I want to keep this
        // function stateless (I know it's in a class but it basically provide persistent
        // parameters. If this were C++ I would have it accept a template with the members to
        // provide this same thing.

        // When the requested speed is larger than what we observed to be the current speed, we
        // need a positive increase to whatever our power was previously, to properly compensate
        // for that issue.
        float powerError = reqSpeed - instSpeed;
        return Range.clip(curPower + powerError, -1.0f, 1.0f);
    }

};

public class FreshSophK9TeleOp extends OpMode {

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

    float avg_turns_per_loop = 0;
    int num_queries = 0;

    float req_speed = 0;

    boolean record_motors;
    FileOutputStream fileStream;
    ObjectOutputStream objectStream;

    String error;

    LinearSpeadModifier linearSpeedMod;

    public FreshSophK9TeleOp(boolean record)
    {
        record_motors = record;

        linearSpeedMod = new LinearSpeadModifier();
        linearSpeedMod.scale = -10;
    }

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
        armExtend = hardwareMap.dcMotor.get("arm extend");

        armAngle.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        armExtend.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        climberHigh = hardwareMap.servo.get("climber high");
        climberLow = hardwareMap.servo.get("climber low");
        leftSideServo = hardwareMap.servo.get("left side servo");
        rightSideServo = hardwareMap.servo.get("right side servo");
    }

    @Override
    public void start() {
        resetStartTime();
        if (record_motors) {
            FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
            try {
                fileStream = app.context.openFileOutput("sampled_auto.txt", Context.MODE_PRIVATE);
                objectStream = new ObjectOutputStream(fileStream);
            } catch (FileNotFoundException e) {
                // Turn off recording
                record_motors = false;
                error = "File Not Found Exception";
            } catch (IOException e) {
                record_motors = false;
                error = "IOException opening output stream";
            }
        }
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

        float armAnglePow = 0.0f;
        float armExtendPow = 0.0f;
        if(gamepad2.left_bumper)
        {
            // Do it slower
            armAnglePow = (float) scaleInputSlow(Range.clip(gamepad2.right_stick_y, -1, 1));
            armExtendPow = (float) scaleInputSlow(Range.clip(gamepad2.left_stick_y, -1, 1));
        }
        else
        {
            // Regular
            armAnglePow = (float) scaleInput(Range.clip(gamepad2.right_stick_y, -1, 1));
            armExtendPow = (float) scaleInput(Range.clip(gamepad2.left_stick_y, -1, 1));
        }



        if(record_motors && objectStream != null)
        {
            try {
                objectStream.writeFloat(right);
                objectStream.writeFloat(left);
                objectStream.writeFloat(armAnglePow);
                objectStream.writeFloat(armExtendPow);
            } catch (IOException e) {
                closeStreams();
                error = "IO Exception writing";
                record_motors = false;
            }
        }
        if(time > 30.0)
        {
            record_motors = false;
            closeStreams();
            error = "RAN OUT OF TIME";
        }

        treadLeft.setPower(left);
        treadRight.setPower(right);

        armAngle.setPower(armAnglePow);

        linearSpeedMod.reqSpeed = armExtendPow;

        linearSpeedMod.lastPosition = linearSpeedMod.curPosition;
        linearSpeedMod.curPosition = armExtend.getCurrentPosition();

        linearSpeedMod.curPower = (float) armExtend.getPower();

        armExtend.setPower(linearSpeedMod.getMotorPower(telemetry));

        telemetry.addData("zz reqSpeed", linearSpeedMod.reqSpeed);
        telemetry.addData("zz curPosition", linearSpeedMod.curPosition);
        telemetry.addData("zz power used", armExtend.getPower());
        telemetry.addData("zz curPower", linearSpeedMod.curPower);
        telemetry.addData("zz lastPosition", linearSpeedMod.lastPosition);

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

        // Ready to measure?
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("arm angle", armAnglePow);
        telemetry.addData("arm extend", armExtend.getPower());
        telemetry.addData("arm angle position", armAngle.getCurrentPosition());
        telemetry.addData("arm extend position", armExtend.getCurrentPosition());
        telemetry.addData("climber high", climberHighPos);
        telemetry.addData("climber low", climberLowPos);
        telemetry.addData("left servo pos", leftSidePos);
        telemetry.addData("right servo pos", rightSidePos);
        if(error != null) telemetry.addData("error", error);
        else telemetry.addData("error", "NO error");

        telemetry.addData("time", time);
    }

    public void closeStreams()
    {
        try {
            if(objectStream != null) objectStream.close();
            if(fileStream != null) fileStream.close();
        } catch (IOException e) {

        }
        catch(NullPointerException e) {

        }
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        closeStreams();
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
