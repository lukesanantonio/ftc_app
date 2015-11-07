package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by localt on 10/3/2015.
 */
public class RampAutoSlow extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;

    boolean first_run;
    ElapsedTime time;

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");

        // Remove this when it's time to experiment with a gyro, we can't have
        // this distorting our control of the motors, etc.
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        first_run = true;
    }
    @Override
    public void loop() {
        if(first_run)
        {
            motorLeft.setPower(0.2f);
            motorRight.setPower(0.2f);
            time.reset();
            first_run = false;
        }
        else
        {
            if(time.time() > 5.0)
            {
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
            }
        }
    }
}
