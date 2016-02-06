/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.StreamCorruptedException;
import java.nio.ByteBuffer;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class FreshSophAuto extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;
    DcMotor armAngle;
    DcMotor armExtend;

    Servo climberHigh;
    Servo climberLow;

    FileInputStream sampledDataFileStream;
    ObjectInputStream objectStream;

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

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armExtend = hardwareMap.dcMotor.get("arm extend");

        climberHigh = hardwareMap.servo.get("climber high");
        climberLow = hardwareMap.servo.get("climber low");

        climberHigh.setPosition(1.0);
        climberLow.setPosition(1.0);
    }

    @Override
    public void start(){
        resetStartTime();

        FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
        try {
            sampledDataFileStream = app.context.openFileInput("sampled_auto.txt");
            objectStream = new ObjectInputStream(sampledDataFileStream);
        } catch (FileNotFoundException e) {

            error = "Failed to find file";
        } catch (StreamCorruptedException e) {
            error = "Stream corrupted";
        } catch (IOException e) {
            error = "IO Exception corrupted";
        }
    }

    public void closeStreams()
    {
        try {
            if(objectStream != null) objectStream.close();
            if(sampledDataFileStream != null) sampledDataFileStream.close();
        } catch (IOException e) {

        }
        catch(NullPointerException e) {

        }
    }

    float readValue() throws IOException {
        float ret = 0.0f;
        if(objectStream == null){
            error = "Bad Object Stream";
            return ret;
        }
        ret = objectStream.readFloat();
        return ret;
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        float left = 0.0f;
        float right = 0.0f;
        float armAnglePow = 0.0f;
        float armExtendPow = 0.0f;
        try {
            right = readValue();
            left = readValue();
            armAnglePow = readValue();
            armExtendPow = readValue();
        } catch (IOException e)
        {
            error = "IO exception in loop";
        }

        treadLeft.setPower(left);
        treadRight.setPower(right);
        armAngle.setPower(armAnglePow);
        armExtend.setPower(armExtendPow);

        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("arm angle", armAnglePow);
        telemetry.addData("arm extend", armExtendPow);

        FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
        telemetry.addData("file path", app.context.getFilesDir().getAbsolutePath());
        if(error != null) telemetry.addData("error", error);
        else telemetry.addData("error", "NO error");
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