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
    Begin,
    Start,
    Searching,
    Moving_Beyond,
    Moving_Timed,
    Orienting,
    Orienting_Further,
    Orienting_Back,
    Moving,
    Lifting,
    Extending,
    Retracting,
    Lowering,
    Backing,
    Done
}

public class WhiteLineOp extends OpMode {

    WhiteLineMode mode;
    Turn turning = Turn.Right;
    double time_at_start = 0.0;
    double time_to_move = 0.0;

    private static final int ENCODER_THRESHOLD = 10;

    private static final float TURNING_POWER = 1.0f;

    private static final float ARM_ANGLE_TARGET_POWER = -1.0f;
    private static final int ARM_ANGLE_TARGET_POSITION = -380;

    private static final float ARM_ANGLE_RESET_POWER = 1.0f;
    private static final int ARM_ANGLE_RESET_POSITION = 0;

    private static final float ARM_EXTEND_TARGET_POWER = 1.0f;
    private static final int ARM_EXTEND_TARGET_POSITION = 8*1440;

    private static final float ARM_EXTEND_RESET_POWER = -1.0f;
    private static final int ARM_EXTEND_RESET_POSITION = 0;

    // TODO: Maybe increase the robot power?
    private static final float MOTOR_POWER = -1.0f;
    private static final float STOP_POWER = 0.0f;

    // TODO: Possibly change this to 5 and see if that works better.
    private static final int DISTANCE_MINIMUM = 5;

    private static final int COLOR_ON_WHITE_THRESHOLD = 9;
    private static final int COLOR_OFF_WHITE_THRESHOLD = 2;

    private static final double BACKING_UP_POWER = 1.0f;
    private static final double BACKING_UP_TIME = 4.0f;

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotor armAngle;
    DcMotor armExtend;

    ServoValues servoValues;
    SophServos sophServos;

    OpticalDistanceSensor distance;
    ColorSensor color;

    double time_delay;

    WhiteLineOp(boolean left, double delay)
    {
        if(left)
        {
            turning = Turn.Left;
        }
        else
        {
            turning = Turn.Right;
        }

        time_delay = delay;
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
        mode = WhiteLineMode.Begin;
    }

    @Override
    public void loop() {
        switch(mode)
        {
            case Begin:
                resetStartTime();
                mode = WhiteLineMode.Start;
                time_at_start = time;
                break;
            case Start:
                if(time >= time_delay) {
                    mode = WhiteLineMode.Searching;
                    time_at_start = time;
                }
                break;
            case Searching:
                telemetry.addData("doing", "searching");
                treadLeft.setPower(MOTOR_POWER);
                treadRight.setPower(MOTOR_POWER);
                if(color.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    mode = WhiteLineMode.Moving_Beyond;
                    time_at_start = time;
                }
                break;
            case Moving_Beyond:
                telemetry.addData("doing", "moving beyond");
                treadLeft.setPower(MOTOR_POWER);
                treadRight.setPower(MOTOR_POWER);
                if(color.alpha() <= COLOR_OFF_WHITE_THRESHOLD) {
                    mode = WhiteLineMode.Moving_Timed;
                    time_to_move = time - time_at_start;
                    time_at_start = time;
                }
                break;
            case Moving_Timed:
                telemetry.addData("doing", "moving timed");
                treadLeft.setPower(MOTOR_POWER);
                treadRight.setPower(MOTOR_POWER);
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
                    treadLeft.setPower(-TURNING_POWER);
                    treadRight.setPower(TURNING_POWER);
                }
                else
                {
                    treadLeft.setPower(TURNING_POWER);
                    treadRight.setPower(-TURNING_POWER);
                }
                if(color.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    mode = WhiteLineMode.Orienting_Further;
                    time_at_start = time;
                }
                break;
            case Orienting_Further:
                telemetry.addData("doing", "orienting further");
                if(turning == Turn.Left)
                {
                    treadLeft.setPower(-TURNING_POWER);
                    treadRight.setPower(TURNING_POWER);
                }
                else
                {
                    treadLeft.setPower(TURNING_POWER);
                    treadRight.setPower(-TURNING_POWER);
                }
                if(color.alpha() <= COLOR_OFF_WHITE_THRESHOLD) {
                    mode = WhiteLineMode.Orienting_Back;
                    time_at_start = time;
                    time_to_move = time - time_at_start;

                }
                break;
            case Orienting_Back:
                telemetry.addData("doing", "orienting back");
                if(turning == Turn.Left)
                {
                    treadLeft.setPower(TURNING_POWER);
                    treadRight.setPower(-TURNING_POWER);
                }
                else
                {
                    treadLeft.setPower(-TURNING_POWER);
                    treadRight.setPower(TURNING_POWER);
                }
                if(time - time_at_start > time_to_move / 2) {
                    mode = WhiteLineMode.Moving;
                    time_at_start = time;
                }
                break;
            case Moving:
                // Maybe add a hard limit on the time we can wait for the distance sensor.
                // Or not, IMO the distance sensor probably won't fail and more likely it will make
                // the robot drop the climbers unnecessarily, screwing us over during the driver
                // period.

                telemetry.addData("doing", "moving");
                treadLeft.setPower(MOTOR_POWER);
                treadRight.setPower(MOTOR_POWER);
                if(distance.getLightDetectedRaw() >= DISTANCE_MINIMUM)
                {
                    mode = WhiteLineMode.Lifting;
                    time_at_start = time;
                }
                if(time - time_at_start > 3.0f)
                {
                    mode = WhiteLineMode.Backing;
                    time_at_start = time;
                }
                break;
            case Lifting:
                telemetry.addData("doing", "lifting");

                // Do we need this?
                treadLeft.setPower(STOP_POWER);
                treadRight.setPower(STOP_POWER);

                // We can pretty much back off unless we are here, so make sure we are still in
                // the allotted time limit.

                armAngle.setTargetPosition(ARM_ANGLE_TARGET_POSITION);
                armAngle.setPower(ARM_ANGLE_TARGET_POWER);
                if(Math.abs(ARM_ANGLE_TARGET_POSITION - armAngle.getCurrentPosition()) < ENCODER_THRESHOLD)
                {
                    mode = WhiteLineMode.Extending;
                    time_at_start = time;
                }
                break;
            case Extending:
                telemetry.addData("doing", "extending");
                armExtend.setTargetPosition(ARM_EXTEND_TARGET_POSITION);
                armExtend.setPower(ARM_EXTEND_TARGET_POWER);
                if(Math.abs(ARM_EXTEND_TARGET_POSITION - armExtend.getCurrentPosition()) < ENCODER_THRESHOLD)
                {
                    // Change to WhiteLineMode.Backing if we want to just back up right away.
                    mode = WhiteLineMode.Backing;
                    time_at_start = time;
                }
                break;
            case Retracting:
                telemetry.addData("doing", "retracting");
                armExtend.setTargetPosition(ARM_EXTEND_RESET_POSITION);
                armExtend.setPower(ARM_EXTEND_RESET_POWER);
                if(Math.abs(ARM_EXTEND_RESET_POSITION - armExtend.getCurrentPosition()) < ENCODER_THRESHOLD)
                {
                    mode = WhiteLineMode.Lowering;
                    time_at_start = time;
                }
                break;
            case Lowering:
                telemetry.addData("doing", "lowering");

                // Why is this here?
                //treadLeft.setPower(MOTOR_POWER);
                //treadRight.setPower(MOTOR_POWER);

                armAngle.setTargetPosition(ARM_ANGLE_RESET_POSITION);
                armAngle.setPower(ARM_ANGLE_RESET_POWER);
                if(Math.abs(ARM_ANGLE_RESET_POSITION - armAngle.getCurrentPosition()) < ENCODER_THRESHOLD)
                {
                    mode = WhiteLineMode.Backing;
                    time_at_start = time;
                }
                break;
            case Backing:
                // If it hasn't been 10 seconds it is very unsafe to backup.
                if(time < 10.0f) break;
                treadLeft.setPower(BACKING_UP_POWER);
                treadRight.setPower(BACKING_UP_POWER);
                if(time - time_at_start >= BACKING_UP_TIME)
                {
                    mode = WhiteLineMode.Done;
                    time_at_start = time;
                }
                break;
            case Done:
            default:
                telemetry.addData("doing", "done");
                treadLeft.setPower(STOP_POWER);
                treadRight.setPower(STOP_POWER);
                break;
        }
        telemetry.addData("color", color.alpha());
        telemetry.addData("distance", distance.getLightDetectedRaw());
        telemetry.addData("time_at_start", time_at_start);
        telemetry.addData("time", time);
    }
}