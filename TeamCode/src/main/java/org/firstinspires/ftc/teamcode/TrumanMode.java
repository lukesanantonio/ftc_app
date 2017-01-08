package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TrumanMode extends OpMode {

    DcMotor mFrontRight;
    DcMotor mFrontLeft;
    DcMotor mBackRight;
    DcMotor mBackLeft;

    DcMotor mRamp;
    DcMotor mPropLeft;
    DcMotor mPropRight;

    CRServo sSlide;
    Servo sLeft;

    @Override
    public void init()
    {
        mFrontRight = hardwareMap.dcMotor.get("front right");
        mFrontLeft = hardwareMap.dcMotor.get("front left");
        mBackRight = hardwareMap.dcMotor.get("back right");
        mBackLeft = hardwareMap.dcMotor.get("back left");

        mBackLeft.setDirection(DcMotor.Direction.REVERSE);
        mFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        //mRamp = hardwareMap.dcMotor.get("ramp");
        mPropRight = hardwareMap.dcMotor.get("prop right");
        mPropLeft = hardwareMap.dcMotor.get("prop left");
        sSlide = hardwareMap.crservo.get("servo slide");
        sLeft = hardwareMap.servo.get("servo left");
        sLeft.setPosition(0.0);
    }
    @Override
    public void loop()
    {
        mBackLeft.setPower(gamepad1.left_stick_y);
        mFrontLeft.setPower(gamepad1.left_stick_y);

        mBackRight.setPower(gamepad1.right_stick_y);
        mFrontRight.setPower(gamepad1.right_stick_y);

        if(gamepad1.left_bumper) {
            mPropLeft.setPower(1.0f);
            mPropRight.setPower(1.0f);
        }
        else {
            mPropLeft.setPower(0.0f);
            mPropRight.setPower(0.0f);
        }

        if(gamepad2.a) {
            sLeft.setPosition(Range.clip(sLeft.getPosition() + .01, 0.0, 1.0));
        }
        else if(gamepad2.b) {
            sLeft.setPosition(Range.clip(sLeft.getPosition() - .01, 0.0, 1.0));
        }

        sSlide.setPower(gamepad2.left_stick_y);
        telemetry.addData("sSlide power", sSlide.getPower());
        telemetry.addData("gamepad2 left_stick_y", gamepad2.left_stick_y);
    }
}
