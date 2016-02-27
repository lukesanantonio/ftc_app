package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RecordSequence extends OpMode {

    ServoValues values;
    SophServos sophServos;

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotor armAngle;
    DcMotor armExtend;

    boolean recording_a = false;
    boolean recording_b = false;
    boolean recording_x = false;
    boolean recording_y = false;

    @Override
    public void init()
    {
        sophServos = new SophServos();
        sophServos.initServos(hardwareMap);

        // Get motors
        treadRight = hardwareMap.dcMotor.get("right");
        treadLeft = hardwareMap.dcMotor.get("left");
        treadLeft.setDirection(DcMotor.Direction.REVERSE);
        treadRight.setDirection(DcMotor.Direction.FORWARD);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armExtend = hardwareMap.dcMotor.get("arm extend");
    }

    @Override
    public void start()
    {
        FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
        DefaultServoPositions defaults = new DefaultServoPositions(app);

        // Load defaults for servos
        values = defaults.read();
        if(values == null)
        {
            values = new ServoValues();

            values.climberLowPos = 0.0;
            values.climberHighPos = 0.0;
            values.leftSidePos = 0.0;
            values.rightSidePos = 0.0;
        }
    }
    @Override
    public void loop()
    {
        // Adjust them
        values.climberLowPos = values.climberLowPos + gamepad1.left_stick_y * .005;
        values.climberHighPos = values.climberHighPos + gamepad1.right_stick_y * .005;

        values.leftSidePos = values.leftSidePos + -gamepad2.left_stick_y * .005;
        values.rightSidePos = values.rightSidePos + gamepad2.right_stick_y * .005;

        // Set them
        sophServos.setValues(values);

        // Press a button on either controller to save the values as they are.
        if(gamepad1.a)
        {
            // Record sequence a
            recording_a = true;
        }
        else if(gamepad1.b)
        {
            recording_b = true;
        }
        else if(gamepad1.y)
        {
            recording_y = true;
        }
        else if(gamepad1.x)
        {
            recording_x = true;
        }

        // Log
        values.logValues(telemetry);
    }
}
