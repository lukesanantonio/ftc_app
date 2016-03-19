package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.CameraPreview;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import android.util.Log;

import java.util.List;
import java.util.ArrayList;

enum Turn
{
    Right, Left
}
enum WhiteLineMode
{
    Start,
    Searching,
    Moving_Beyond,
    Moving_Timed,
    Orienting,
    Moving,
    Lifting,
    Extending,
    Done
}

public class WhiteLineOp extends OpMode {

    WhiteLineMode mode;
    Turn turning = Turn.Right;
    double time_at_start = 0.0;
    double time_to_move = 0.0;

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotor armAngle;
    DcMotor armExtend;

    ServoValues servoValues;
    SophServos sophServos;

    OpticalDistanceSensor distance;
    ColorSensor color;

    WhiteLineOp(boolean left)
    {
        if(left)
        {
            turning = Turn.Left;
        }
        else
        {
            turning = Turn.Right;
        }
    }

    @Override
    public void init() {
        // Initialize everything
        // TODO: Abstract this!
        treadRight = hardwareMap.dcMotor.get("right");
        treadLeft = hardwareMap.dcMotor.get("left");
        treadLeft.setDirection(DcMotor.Direction.REVERSE);
        treadRight.setDirection(DcMotor.Direction.FORWARD);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armAngle.setDirection(DcMotor.Direction.REVERSE);
        armExtend = hardwareMap.dcMotor.get("arm extend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        armAngle.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        sophServos = new SophServos();
        sophServos.initServos(hardwareMap);

        distance = hardwareMap.opticalDistanceSensor.get("distance");
        color = hardwareMap.colorSensor.get("color");
    }

    @Override
    public void stop() {
    }

    @Override
    public void start() {
        mode = WhiteLineMode.Start;
    }

    @Override
    public void loop() {
        switch(mode)
        {
            case Start:
                mode = WhiteLineMode.Searching;
                time_at_start = time;
                break;
            case Searching:
                telemetry.addData("doing", "searching");
                treadLeft.setPower(-0.5f);
                treadRight.setPower(-0.5f);
                if(color.alpha() >= 9) {
                    mode = WhiteLineMode.Moving_Beyond;
                    time_at_start = time;
                }
                break;
            case Moving_Beyond:
                telemetry.addData("doing", "moving beyond");
                treadLeft.setPower(-0.5f);
                treadRight.setPower(-0.5f);
                if(color.alpha() < 3) {
                    mode = WhiteLineMode.Moving_Timed;
                    time_to_move = time - time_at_start;
                    time_at_start = time;
                }
                break;
            case Moving_Timed:
                telemetry.addData("doing", "moving timed");
                treadLeft.setPower(-0.5f);
                treadRight.setPower(-0.5f);
                if(time - time_at_start > time_to_move)
                {
                    mode = WhiteLineMode.Orienting;
                    time_at_start = time;
                }
                break;
            case Orienting:
                telemetry.addData("doing", "orienting");
                if(turning == Turn.Left)
                {
                    treadLeft.setPower(1.f);
                    treadRight.setPower(-1.f);
                }
                else
                {
                    treadLeft.setPower(1.f);
                    treadRight.setPower(-1.f);
                }
                if(color.alpha() >= 9) {
                    mode = WhiteLineMode.Moving;
                    time_at_start = time;
                }
                break;
            case Moving:
                telemetry.addData("doing", "moving");
                treadLeft.setPower(-0.5f);
                treadRight.setPower(-0.5f);
                if(distance.getLightDetectedRaw() > 1)
                {
                    mode = WhiteLineMode.Lifting;
                    time_at_start = time;
                }
                break;
            case Lifting:
                telemetry.addData("doing", "lifting");
                treadLeft.setPower(-0.5f);
                treadRight.setPower(-0.5f);

                armAngle.setTargetPosition(-500);
                armAngle.setPower(-1.0f);
                if(armAngle.getCurrentPosition() < -490)
                {
                    mode = WhiteLineMode.Extending;
                    time_at_start = time;
                }
                break;
            case Extending:
                telemetry.addData("doing", "extending");
                armExtend.setTargetPosition(8*1440);
                armExtend.setPower(1.0f);
                if(armExtend.getCurrentPosition() > (8*1440) - 10)
                {
                    mode = WhiteLineMode.Done;
                    time_at_start = time;
                }
            case Done:
            default:
                telemetry.addData("doing", "done");
                treadLeft.setPower(0.0f);
                treadRight.setPower(0.0f);
                break;
        }
        telemetry.addData("color", color.alpha());
        telemetry.addData("distance", distance.getLightDetectedRaw());
        telemetry.addData("time_at_start", time_at_start);
    }
}