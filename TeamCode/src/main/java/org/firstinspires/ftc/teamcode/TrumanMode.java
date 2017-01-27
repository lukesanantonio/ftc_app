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

    Servo sSlide;
    float slidePosition;

    private static float SERVO_BOTTOM = .33f;
    private static float SERVO_TOP = .70f;

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

        //mRamp = hardwareMap.dcMotor.get("ramp");
        mPropRight = hardwareMap.dcMotor.get("prop right");
        mPropLeft = hardwareMap.dcMotor.get("prop left");
        sSlide = hardwareMap.servo.get("servo slide");

        slidePosition = 0.5f;
    }
    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            mPropLeft.setPower(1.0f);
            mPropRight.setPower(1.0f);
        } else {
            mPropLeft.setPower(0.0f);
            mPropRight.setPower(0.0f);
        }

        float leftstep = gamepad1.left_trigger - gamepad1.right_trigger;
        if (Math.abs(leftstep) > 0.0f)
        {
            mBackLeft.setPower(-leftstep);
            mFrontLeft.setPower(leftstep);
            mBackRight.setPower(-leftstep);
            mFrontRight.setPower(leftstep);
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

        slidePosition += gamepad2.left_stick_y / 50.0f;
        slidePosition = Range.clip(slidePosition, SERVO_BOTTOM, SERVO_TOP);
        sSlide.setPosition(slidePosition);
        telemetry.addData("sSlide power", sSlide.getPosition());
        // top .77
        // bottom .44
        telemetry.addData("gamepad2 left_stick_y", gamepad2.left_stick_y);
    }
}
