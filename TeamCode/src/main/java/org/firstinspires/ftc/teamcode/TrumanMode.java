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

    TankHardware tank;

    DcMotor mRamp;
    DcMotor mPropLeft;
    DcMotor mPropRight;

    Servo sSlide;
    float slidePosition;

    float propPower;

    private static float PROP_POWER_FACTOR = .005f;

    private static float SERVO_BOTTOM = .33f;
    private static float SERVO_TOP = .70f;

    @Override
    public void init()
    {
        tank = new TankHardware(hardwareMap);
        tank.setTelemetry(telemetry);

        //mRamp = hardwareMap.dcMotor.get("ramp");
        mPropRight = hardwareMap.dcMotor.get("prop right");
        mPropLeft = hardwareMap.dcMotor.get("prop left");
        sSlide = hardwareMap.servo.get("servo slide");

        // Rogue Motor
        //mMotor = hardwareMap.dcMotor.get("motor");

        slidePosition = SERVO_BOTTOM;
        propPower = 0.0f;
    }
    @Override
    public void loop() {
        if(gamepad2.left_bumper)
        {
            propPower += PROP_POWER_FACTOR;
        }
        else
        {
            propPower -= PROP_POWER_FACTOR;
        }
        propPower = Range.clip(propPower, 0.0f, 1.0f);

        mPropLeft.setPower(propPower);
        mPropRight.setPower(propPower);

        float leftstep = gamepad1.left_trigger - gamepad1.right_trigger;
        if (Math.abs(leftstep) > 0.0f)
        {
            tank.setLeft(leftstep);
        }
        else
        {
            // Don't change the motor direction in init because that will alter the
            tank.setForward(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
        telemetry.addData("leftstep", leftstep);

        slidePosition += gamepad2.left_stick_y / 50.0f;
        slidePosition = Range.clip(slidePosition, SERVO_BOTTOM, SERVO_TOP);
        sSlide.setPosition(slidePosition);
        telemetry.addData("sSlide power", sSlide.getPosition());
        // top .77
        // bottom .44
        telemetry.addData("gamepad2 left_stick_y", gamepad2.left_stick_y);

        tank.logPower();
    }
}
