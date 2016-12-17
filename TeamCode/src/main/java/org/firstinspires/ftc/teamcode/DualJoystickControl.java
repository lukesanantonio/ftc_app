/* Copyright (C) 2015 Luke San Antonio
 * All rights reserved.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="Dual Joystick Control")
public class DualJoystickControl extends OpMode {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("front left");
        motorBackLeft = hardwareMap.dcMotor.get("back left");
        motorFrontRight = hardwareMap.dcMotor.get("front right");
        motorBackRight = hardwareMap.dcMotor.get("back right");
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        float left = gamepad1.left_stick_y;
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);

        float right = gamepad1.right_stick_y;
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right); // hey luke. Hey Colin
    }
}
