/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name = "Andrew Mode")
public class AndrewMode extends OpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor spinner;

    enum State {
        Resting,
        Spinning,
    }

    private State state = State.Resting;
    private double time_spin_started = 0.0f;

    private static double LAUNCH_SPINNING_TIME = .1f;

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("front left");
        backLeft = hardwareMap.dcMotor.get("back left");
        frontRight = hardwareMap.dcMotor.get("front right");
        backRight = hardwareMap.dcMotor.get("back right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        spinner = hardwareMap.dcMotor.get("spinner");
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        float left = gamepad1.left_stick_y;
        frontLeft.setPower(left);
        backLeft.setPower(left);

        float right = gamepad1.right_stick_y;
        frontRight.setPower(right);
        backRight.setPower(right); // hey luke. Hey Colin

        // The spinner controlled by the joystick needs to be a lot slower.
        if (state == State.Resting) {
            spinner.setPower(gamepad2.right_stick_y + gamepad2.left_stick_y * .25f);
        }

        // Run it at full speed for a short time with a button
        if (gamepad2.a && state == State.Resting) {
            state = State.Spinning;
            time_spin_started = time;
        }

        if (state == State.Spinning) {
            if (time - time_spin_started < LAUNCH_SPINNING_TIME) {
                spinner.setPower(-1.0f);
            } else {
                spinner.setPower(0.0f);
                state = State.Resting;
            }
        }
    }
}
