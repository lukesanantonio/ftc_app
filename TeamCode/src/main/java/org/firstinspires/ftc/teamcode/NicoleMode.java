package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class NicoleMode extends OpMode {

    TankHardware tank;

    DcMotor mLift;
    DcMotor mPropLeft;
    DcMotor mPropRight;

    CRServo sSlide;

    boolean propOn;
    double lastTimeSwitched;

    @Override
    public void init()
    {
        tank = new TankHardware(hardwareMap);
        tank.setTelemetry(telemetry);

        mPropLeft = hardwareMap.dcMotor.get("prop left");
        mPropRight = hardwareMap.dcMotor.get("prop right");

        mPropLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mPropRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mLift = hardwareMap.dcMotor.get("lift");
        sSlide = hardwareMap.crservo.get("slide");

        propOn = false;
        lastTimeSwitched = 0.0;
    }
    @Override
    public void loop() {

        if(gamepad2.right_bumper && time - lastTimeSwitched >= 1.0f)
        {
            propOn = !propOn;
            lastTimeSwitched = time;
        }

        if(propOn) {
            mPropLeft.setPower(1.0f);
            mPropRight.setPower(1.0f);
        }
        else
        {
            mPropLeft.setPower(0.0f);
            mPropRight.setPower(0.0f);
        }

        sSlide.setPower(gamepad2.right_stick_y);
        mLift.setPower(gamepad2.left_stick_y);

        float leftstep = gamepad1.left_trigger - gamepad1.right_trigger;
        if (Math.abs(leftstep) > 0.0f)
        {
            tank.setLeft(leftstep);
        }
        else
        {
            tank.setForward(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
        tank.logPower();
    }
}
