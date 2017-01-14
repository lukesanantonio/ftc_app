package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Noah Pietrafesa on 12/3/16.
 */
@TeleOp
public class NoahOpMode extends OpMode {

    DcMotor mFrontRight;
    DcMotor mFrontLeft;
    DcMotor mBackRight;
    DcMotor mBackLeft;

    @Override
    public void init() {
        mFrontRight = hardwareMap.dcMotor.get("front right");
        mFrontLeft = hardwareMap.dcMotor.get("front left");
        mBackRight = hardwareMap.dcMotor.get("back right");
        mBackLeft = hardwareMap.dcMotor.get("back left");

        mBackLeft.setDirection(DcMotor.Direction.REVERSE);
        mFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    //Noah was here
    //Now Luke is here

    public float linInterp(float fac, float a, float b)
    {
        return a + (b - a) * fac;
    }

    @Override
    public void loop() {

        float frontPower = linInterp(Math.abs(gamepad1.left_stick_y), -1.0f * gamepad1.left_stick_x,
                gamepad1.left_stick_y);

        float backPower = linInterp(Math.abs(gamepad1.left_stick_y), gamepad1.left_stick_x,
                gamepad1.left_stick_y);

        mFrontLeft.setPower(frontPower);
        mFrontRight.setPower(frontPower);

        mBackLeft.setPower(backPower);
        mBackRight.setPower(backPower);

        telemetry.addData("front power", frontPower);
        telemetry.addData("back power", backPower);
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("turning power", gamepad1.left_trigger - gamepad1.right_trigger);


        if (gamepad1.right_bumper) {
            //horizontal
            mFrontLeft.setPower(-1.0f * gamepad1.left_stick_x);
            mBackLeft.setPower(gamepad1.left_stick_x);
            mFrontRight.setPower(-1.0f * gamepad1.left_stick_x);
            mBackRight.setPower(gamepad1.left_stick_x);

        }
        //turn
        else if (gamepad1.left_bumper) {
            mFrontLeft.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            mBackLeft.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            mFrontRight.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            mBackRight.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        } else {
            //vertical
            //mFrontLeft.setPower(gamepad1.right_stick_y);
            //mBackLeft.setPower(gamepad1.right_stick_y);
            //mFrontRight.setPower(-1 * (gamepad1.right_stick_y));
            //mBackRight.setPower(-1 * (gamepad1.right_stick_y));
        }
        //Diagonals
//       else if(gamepad1.left_trigger ==0) {
//            left2.setPower(gamepad1.left_stick_y);
//            right1.setPower(-1 * (gamepad1.left_stick_y));
//        }
//        else {
//            left1.setPower(gamepad1.left_stick_x);
//            right2.setPower(-1 * (gamepad1.left_stick_x));
//        }

        //non mecanum wheels config
//        left1.setPower(gamepad1.left_stick_y);
//        left2.setPower(gamepad1.left_stick_y);
//        right2.setPower(-1*(gamepad1.right_stick_y));
//        right1.setPower(-1*(gamepad1.right_stick_y));
    }
}
