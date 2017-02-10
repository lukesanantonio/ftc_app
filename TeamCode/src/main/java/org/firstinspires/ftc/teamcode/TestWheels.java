package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by luke on 2/4/17.
 */

@Autonomous
public class TestWheels extends OpMode {
    DcMotor mFrontLeft;
    DcMotor mFrontRight;
    DcMotor mBackLeft;
    DcMotor mBackRight;

    int motor_i;
    boolean forward;
    double last_time;

    private static final double TIME = .8;
    private static final float POWER = .6f;

    @Override
    public void init()
    {
        mFrontLeft = hardwareMap.dcMotor.get("front left");
        mFrontRight = hardwareMap.dcMotor.get("front right");
        mBackLeft = hardwareMap.dcMotor.get("back left");
        mBackRight = hardwareMap.dcMotor.get("back right");

        mFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        mFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start()
    {
        motor_i = 0;
        forward = true;

        last_time = time;
    }

    @Override
    public void loop() {
        if (time - last_time < TIME)
        {
            float power = POWER;
            if(!forward) power = -POWER;

            switch(motor_i)
            {
                case 0:
                    mFrontLeft.setPower(power);
                    break;
                case 1:
                    mFrontRight.setPower(power);
                    break;
                case 2:
                    mBackRight.setPower(power);
                    break;
                case 3:
                    mBackLeft.setPower(power);
                    break;
                default:
                    break;
            }
        }
        else
        {
            mFrontLeft.setPower(0.0f);
            mFrontRight.setPower(0.0f);
            mBackLeft.setPower(0.0f);
            mBackRight.setPower(0.0f);

            // Change "state"
            forward = !forward;
            if(forward) {
                ++motor_i;
            }
            if(motor_i > 4) motor_i = 4;

            last_time = time;
        }

        telemetry.addData("time", time);
        telemetry.addData("last_time", last_time);
        telemetry.addData("motor_i", motor_i);
        telemetry.addData("forward", forward);

    }
}
