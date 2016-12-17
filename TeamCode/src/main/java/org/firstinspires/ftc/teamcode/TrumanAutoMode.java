package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by luke on 11/19/16.
 */

@Autonomous
public class TrumanAutoMode extends OpMode {

    DcMotor mFrontRight;
    DcMotor mFrontLeft;
    DcMotor mBackRight;
    DcMotor mBackLeft;

    DcMotor mRamp;
    DcMotor mPropLeft;
    DcMotor mPropRight;

    CRServo sSlide;
    Servo sLeft;
    Servo sRight;

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
        sSlide.setPower(0.0);
    }

    @Override
    public void start()
    {
        resetStartTime();
    }
    @Override
    public void loop()
    {
        if(time < 2.0f)
        {
            mFrontRight.setPower(1.0f);
            mBackRight.setPower(1.0f);
            mFrontLeft.setPower(1.0f);
            mBackRight.setPower(1.0f);
        }
        else
        {
            mFrontRight.setPower(0.0f);
            mBackRight.setPower(0.0f);
            mFrontLeft.setPower(0.0f);
            mBackRight.setPower(0.0f);
        }
    }
}
