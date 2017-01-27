package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by luke on 1/24/17.
 */

@TeleOp
public class SingleMotorControl extends OpMode {
    DcMotor motor;
    @Override
    public void init()
    {
        motor = hardwareMap.dcMotor.get("motor");
    }

    @Override
    public void loop()
    {
        motor.setPower(gamepad1.left_stick_y);
    }
}
