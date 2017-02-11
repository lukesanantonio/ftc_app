package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class NicoleMode extends OpMode {

    TankHardware tank;

    DcMotor mLift;
    DcMotor mPropLeft;
    DcMotor mPropRight;

    CRServo sSlide;

    boolean propOn;
    double lastTimeProp;

    boolean slowOn;
    double lastTimeSlow;

    private static final float SLOW_TURN_MULTIPLIER = .65f;
    private static final float SLOW_MULTIPLIER = .275f;

    @Override
    public void init() {
        tank = new TankHardware(hardwareMap);
        tank.setTelemetry(telemetry);

        mPropLeft = hardwareMap.dcMotor.get("prop left");
        mPropRight = hardwareMap.dcMotor.get("prop right");

        mPropLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mPropRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mPropRight.setDirection(DcMotorSimple.Direction.REVERSE);

        mLift = hardwareMap.dcMotor.get("lift");
        sSlide = hardwareMap.crservo.get("slide");

        propOn = false;
        lastTimeProp = 0.0;

        slowOn = false;
        lastTimeSlow = 0.0f;
    }

    @Override
    public void loop() {

        if (gamepad2.right_bumper && time - lastTimeProp >= 1.0f) {
            propOn = !propOn;
            lastTimeProp = time;
        }
        if (gamepad1.right_bumper && time - lastTimeSlow >= 1.0f) {
            slowOn = !slowOn;
            lastTimeSlow = time;
        }

        if (propOn) {
            mPropLeft.setPower(0.3f);
            mPropRight.setPower(0.3f);
        } else {
            mPropLeft.setPower(0.0f);
            mPropRight.setPower(0.0f);
        }

        sSlide.setPower(gamepad2.right_stick_y);

        if (gamepad2.dpad_up) {
            mLift.setPower(1.0f);
        } else if (gamepad2.dpad_down) {
            mLift.setPower(-1.0f);
        } else {
            mLift.setPower(0.0f);
        }

        float leftstep = gamepad1.left_trigger - gamepad1.right_trigger;
        if (Math.abs(leftstep) > 0.0f) {
            tank.setLeft(leftstep);
        } else {
            float leftPower = -gamepad1.left_stick_y;
            float rightPower = -gamepad1.right_stick_y;

            if (slowOn && ((leftPower < 0.0f && rightPower > 0.0f) ||
                    (leftPower > 0.0f && rightPower < 0.0f))) {
                // We are turning
                leftPower *= SLOW_TURN_MULTIPLIER;
                rightPower *= SLOW_TURN_MULTIPLIER;
            } else if (slowOn) {
                leftPower *= SLOW_MULTIPLIER;
                rightPower *= SLOW_MULTIPLIER;
            }
            tank.setForward(leftPower, rightPower);
        }

        if (slowOn) {
            telemetry.addData("stop mode", "ON");
        } else {
            telemetry.addData("stop mode", "OFF");
        }

        tank.logPower();
    }
}
