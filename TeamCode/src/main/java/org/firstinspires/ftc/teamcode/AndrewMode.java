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

    DcMotor mSlide;
    DcMotor mButton;

    float fac;
    boolean slow_mode;
    boolean just_changed;

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

        mSlide = hardwareMap.dcMotor.get("slide");
        mSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mButton = hardwareMap.dcMotor.get("button");

        fac = 1.0f;
        slow_mode = false;
    }
    @Override
    public void loop() {

        if(slow_mode) fac = .4f;
        else fac = 1.0f;

        float leftstep = gamepad1.left_trigger - gamepad1.right_trigger * fac;
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
            mBackLeft.setPower(gamepad1.left_stick_y * fac);
            mFrontLeft.setPower(gamepad1.left_stick_y * fac);
            mBackRight.setPower(gamepad1.right_stick_y * fac);
            mFrontRight.setPower(gamepad1.right_stick_y * fac);
        }
        telemetry.addData("leftstep", leftstep);
        telemetry.addData("gamepad2 left_stick_y", gamepad2.left_stick_y);

        telemetry.addData("slide", mSlide.getCurrentPosition());

        if(gamepad1.left_bumper && !just_changed)
        {
            slow_mode = !slow_mode;
            just_changed = true;
        }
        else if(!gamepad1.left_bumper)
        {
            just_changed = false;
        }

        mSlide.setPower(gamepad2.left_stick_y * fac);
        mButton.setPower(gamepad2.right_stick_y * fac);
    }
}