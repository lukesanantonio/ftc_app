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



public class FreshSophK9TeleOp extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotor armAngle;
    DcMotor armExtend;

    ServoValues servoValues;
    SophServos sophServos;

    boolean record_motors;
    FileOutputStream fileStream;
    ObjectOutputStream objectStream;

    ErrorManager error;

    DefaultServoPositions positions;

    public FreshSophK9TeleOp(boolean record)
    {
        record_motors = record;

        if(record_motors) {
            error = new ErrorManager("Driving OP mode with recording");
        } else {
            error = new ErrorManager("Driving OP mode without recording");
        }
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
        armAngle.setDirection(DcMotor.Direction.REVERSE);
        armExtend = hardwareMap.dcMotor.get("arm extend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        armAngle.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        armExtend.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        sophServos = new SophServos();
        sophServos.initServos(hardwareMap);

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
                error.setError("File Not Found Exception");
            } catch (IOException e) {
                record_motors = false;
                error.setError("IOException opening output stream");
            }
        }

        FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
        positions = new DefaultServoPositions(app.context);

        // Load default positions
        servoValues = positions.read();
        if(servoValues != null) sophServos.setValues(servoValues);
        else
            servoValues = new ServoValues();
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
                error.setError("IO Exception writing");
                record_motors = false;
            }
        }
        if(time > 30.0)
        {
            record_motors = false;
            closeStreams();
            error.setError("RAN OUT OF TIME");
        }

        treadLeft.setPower(left);
        treadRight.setPower(right);

        armAngle.setPower(armAnglePow);
        armExtend.setPower(armExtendPow);

        // Adjust servo values
        if(gamepad1.dpad_up) servoValues.climberHighPos = Range.clip(servoValues.climberHighPos + .01, 0, 1);
        if(gamepad1.dpad_down) servoValues.climberHighPos = Range.clip(servoValues.climberHighPos - .01, 0, 1);
        if(gamepad1.dpad_right) servoValues.climberLowPos = Range.clip(servoValues.climberLowPos + .01, 0, 1);
        if(gamepad1.dpad_left) servoValues.climberLowPos = Range.clip(servoValues.climberLowPos - .01, 0, 1);

        if(gamepad2.dpad_up) servoValues.rightSidePos = Range.clip(servoValues.rightSidePos + .01, 0, 1);
        if(gamepad2.dpad_down) servoValues.rightSidePos = Range.clip(servoValues.rightSidePos - .01, 0, 1);
        if(gamepad2.dpad_right) servoValues.leftSidePos = Range.clip(servoValues.leftSidePos + .01, 0, 1);
        if(gamepad2.dpad_left) servoValues.leftSidePos = Range.clip(servoValues.leftSidePos - .01, 0, 1);

        // Clip and adjust actual servos
        sophServos.setValues(servoValues);
        // Log them
        servoValues.logValues(telemetry);

        // Ready to measure?
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("arm angle", armAngle.getPower());
        telemetry.addData("arm extend", armExtend.getPower());
        telemetry.addData("arm angle position", armAngle.getCurrentPosition());
        telemetry.addData("arm extend position", armExtend.getCurrentPosition());

        error.logError(telemetry);
        positions.error.logError(telemetry);

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
