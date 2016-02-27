/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class StraightAuto extends OpMode {

    DcMotor treadLeft;
    DcMotor treadRight;

    ServoValues servoValues;
    SophServos sophServos;

    double time_delay;

    public StraightAuto(double delay)
    {
        time_delay = delay;
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

        // Load servos
        sophServos = new SophServos();
        sophServos.initServos(hardwareMap);

    }

    @Override
    public void start(){
        resetStartTime();

        // Set servos
        FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
        DefaultServoPositions positions = new DefaultServoPositions(app.context);

        // Load default positions
        servoValues = positions.read();
        sophServos.setValues(servoValues);
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

        if(time < time_delay)
        {
            // Both left and right will remain zero.
        }
        else if(time < 8.0 + time_delay){
            left = -0.5f;
            right = -0.5f;
        }

        treadLeft.setPower(left);
        treadRight.setPower(right);

        telemetry.addData("left", left);
        telemetry.addData("right", right);
    }
}