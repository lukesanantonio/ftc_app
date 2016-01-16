package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by localt on 10/3/2015.
 */
public class JuniorAuto extends JuniorK9TeleOp {

    @Override
    public void loop()
    {
        if(time < 3.0)
        {
            super.motorLeftTread.setPower(.7);
            super.motorRightTread.setPower(.7);
            super.motorLeftWheel.setPower(.7);
            super.motorRightWheel.setPower(.7);
        }
        else if(time < 6.0)
        {
            super.armMotor.setPower(1.0);
        }
        else
        {
            super.motorLeftTread.setPower(0.0);
            super.motorRightTread.setPower(0.0);
            super.motorLeftWheel.setPower(0.0);
            super.motorRightWheel.setPower(0.0);
            super.armMotor.setPower(0.0);
        }
    }
}
