package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class AndrewMode extends OpMode {

    DcMotor mFrontRight;
    DcMotor mFrontLeft;
    DcMotor mBackRight;
    DcMotor mBackLeft;

    DcMotor mPropLeft;
    DcMotor mPropRight;

    @Override
    public void init()
    {
        mFrontRight = hardwareMap.dcMotor.get("front right");
        mFrontLeft = hardwareMap.dcMotor.get("front left");
        mBackRight = hardwareMap.dcMotor.get("back right");
        mBackLeft = hardwareMap.dcMotor.get("back left");

        mBackLeft.setDirection(DcMotor.Direction.FORWARD);
        mFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        mBackRight.setDirection(DcMotor.Direction.REVERSE);
        mFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        float leftstep = gamepad1.left_trigger - gamepad1.right_trigger;
        if (Math.abs(leftstep) > 0.0f)
        {
            mBackLeft.setPower(-leftstep);
            mFrontLeft.setPower(leftstep);
            mBackRight.setPower(leftstep);
            mFrontRight.setPower(-leftstep);
        }
        else
        {
            // Don't change the motor direction in init because that will alter the
            mBackLeft.setPower(gamepad1.left_stick_y);
            mFrontLeft.setPower(gamepad1.left_stick_y);
            mBackRight.setPower(gamepad1.right_stick_y);
            mFrontRight.setPower(gamepad1.right_stick_y);
        }
        telemetry.addData("leftstep", leftstep);
        telemetry.addData("gamepad2 left_stick_y", gamepad2.left_stick_y);
    }
}