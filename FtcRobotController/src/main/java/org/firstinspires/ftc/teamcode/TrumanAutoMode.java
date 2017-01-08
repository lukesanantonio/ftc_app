package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class TrumanAutoMode extends OpMode {

    // Constants
    private static final float MOTOR_POWER = 1.0f;

    private static final int DISTANCE_MINIMUM = 5;

    private static final float SCAN_LEFT_SLIDE_POWER = -1.0f;
    private static final float SCAN_RIGHT_SLIDE_POWER = 1.0f;

    private static final int COLOR_ON_WHITE_THRESHOLD = 9;
    private static final int COLOR_OFF_WHITE_THRESHOLD = 2;

    private static final double CLICKER_SLIDE_TIME = .4f;
    private static final double CLICKING_FORWARD_TIME = 1.0f;

    private static final double BACKING_UP_TIME = 1.0f;

    public enum Turn {
        Right, Left
    }

    public enum Color {
        Red, Blue
    }

    public enum State {
        Begin,
        RampUpShootStageOne,
        RampUpShootStageTwo,
        ShootingBalls,
        RampDownShootStageOne,
        RampDownShootStageTwo,
        Turning,
        Start,
        Searching,
        Moving_Beyond,
        Moving_Timed,
        Orienting,
        Orienting_Further,
        Orienting_Back,
        Moving,
        ScanningLeft,
        CenteringFromLeft,
        ScanningRight,
        CenteringFromRight,
        Picking,
        GoLeft,
        GoRight,
        Clicking,
        Backing,
        Done
    }

    // Motors and servos
    DcMotor mFrontRight;
    DcMotor mFrontLeft;
    DcMotor mBackRight;
    DcMotor mBackLeft;

    DcMotor mRamp;
    DcMotor mPropLeft;
    DcMotor mPropRight;

    CRServo sSlide;
    Servo sBallGuard;
    Servo sLeft;
    Servo sRight;

    // Miscellaneous Hardware
    OpticalDistanceSensor distance;
    ColorSensor bottomColor;
    ColorSensor frontColor;

    // Init state
    Turn turning = Turn.Right;
    Color seekColor = Color.Red;
    double timeDelay;

    // State
    State state;
    double time_at_start = 0.0;
    double time_to_move = 0.0;
    double time_to_scan = 0.0f;
    Color leftSideGuess = null;
    Color rightSideGuess = null;

    public TrumanAutoMode(Turn turn, Color color, double delay) {
        turning = turn;
        seekColor = color;
        timeDelay = delay;
    }

    @Override
    public void init() {
        mFrontRight = hardwareMap.dcMotor.get("front right");
        mFrontLeft = hardwareMap.dcMotor.get("front left");
        mBackRight = hardwareMap.dcMotor.get("back right");
        mBackLeft = hardwareMap.dcMotor.get("back left");

        mBackLeft.setDirection(DcMotor.Direction.REVERSE);
        mFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        mPropRight = hardwareMap.dcMotor.get("prop right");
        mPropLeft = hardwareMap.dcMotor.get("prop left");

        sBallGuard = hardwareMap.servo.get("servo guard");
        sBallGuard.setPosition(1.0f);

        sSlide = hardwareMap.crservo.get("servo slide");
        sLeft = hardwareMap.servo.get("servo left");
        sLeft.setPosition(0.0);
        sSlide.setPower(0.0);

        distance = hardwareMap.opticalDistanceSensor.get("distance");
        frontColor = hardwareMap.colorSensor.get("front");
        bottomColor = hardwareMap.colorSensor.get("bottom");
    }

    @Override
    public void stop() {
    }

    @Override
    public void start() {
        state = State.Begin;
    }

    private void changeState(State new_state) {
        time_at_start = time;
        state = new_state;
    }

    private void setMotorsForward() {
        mFrontRight.setPower(MOTOR_POWER);
        mBackRight.setPower(MOTOR_POWER);
        mFrontLeft.setPower(MOTOR_POWER);
        mBackLeft.setPower(MOTOR_POWER);
    }

    private void setMotorsBackward() {
        mFrontRight.setPower(-MOTOR_POWER);
        mBackRight.setPower(-MOTOR_POWER);
        mFrontLeft.setPower(-MOTOR_POWER);
        mBackLeft.setPower(-MOTOR_POWER);
    }

    private void setMotorsStopped() {
        mFrontRight.setPower(0.0f);
        mBackRight.setPower(0.0f);
        mFrontLeft.setPower(0.0f);
        mBackLeft.setPower(0.0f);
    }

    private void setTurnRight() {
        mFrontRight.setPower(-MOTOR_POWER);
        mBackRight.setPower(-MOTOR_POWER);
        mFrontLeft.setPower(MOTOR_POWER);
        mBackLeft.setPower(MOTOR_POWER);
    }

    private void setTurnLeft() {
        mFrontRight.setPower(MOTOR_POWER);
        mBackRight.setPower(MOTOR_POWER);
        mFrontLeft.setPower(-MOTOR_POWER);
        mBackLeft.setPower(-MOTOR_POWER);
    }

    @Override
    public void loop() {
        switch (state) {
            case Begin:
                resetStartTime();
                changeState(State.RampUpShootStageOne);
                break;
            case RampUpShootStageOne:
                telemetry.addData("doing", "ramp up shoot stage one");
                if (time - time_at_start < .4f) {
                    mPropRight.setPower(.4f);
                    mPropLeft.setPower(.4f);
                } else {
                    changeState(State.RampUpShootStageTwo);
                }
                break;
            case RampUpShootStageTwo:
                telemetry.addData("doing", "ramp up shoot stage two");
                if (time - time_at_start < .4f) {
                    mPropRight.setPower(.7f);
                    mPropLeft.setPower(.7f);
                } else {
                    changeState(State.ShootingBalls);
                }
                break;
            case ShootingBalls:
                telemetry.addData("doing", "shooting balls");
                if (time - time_at_start < 2.0f) {
                    mPropRight.setPower(1.0f);
                    mPropLeft.setPower(1.0f);
                    sBallGuard.setPosition(0.0f);
                } else {
                    changeState(State.Start);
                }
                break;
            case RampDownShootStageOne:
                telemetry.addData("doing", "ramp down shoot stage one");
                if (time - time_at_start < .4f) {
                    mPropRight.setPower(.7f);
                    mPropLeft.setPower(.7f);
                } else {
                    changeState(State.RampDownShootStageTwo);
                }
                break;
            case RampDownShootStageTwo:
                telemetry.addData("doing", "ramp down shoot stage two");
                if (time - time_at_start < .4f) {
                    mPropRight.setPower(.4f);
                    mPropLeft.setPower(.4f);
                } else {
                    changeState(State.Start);
                }
                break;
            case Start:
                telemetry.addData("doing", "starting");

                // Turn them off completely
                mPropRight.setPower(0.0f);
                mPropLeft.setPower(0.0f);

                if (time >= timeDelay) {
                    changeState(State.Turning);
                }
                break;
            case Turning:
                telemetry.addData("doing", "turning");
                if (turning == Turn.Left) {
                    setTurnLeft();
                } else if (turning == Turn.Right) {
                    setTurnRight();
                }
                if (time - time_at_start > 1.0f) {
                    changeState(State.Searching);
                }
                break;
            case Searching:
                telemetry.addData("doing", "searching");
                setMotorsForward();

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    changeState(State.Moving_Beyond);
                }
                break;
            case Moving_Beyond:
                telemetry.addData("doing", "moving beyond");
                if (bottomColor.alpha() <= COLOR_OFF_WHITE_THRESHOLD) {
                    time_to_move = time - time_at_start;
                    changeState(State.Moving_Timed);
                }
                break;
            case Moving_Timed:
                telemetry.addData("doing", "moving timed");
                setMotorsForward();

                if (time - time_at_start > time_to_move) {
                    changeState(State.Orienting);
                }
                break;
            case Orienting:
                telemetry.addData("doing", "orienting");
                if (turning == Turn.Left) {
                    setTurnLeft();
                } else {
                    setTurnRight();
                }
                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    changeState(State.Orienting_Further);
                }
                break;
            case Orienting_Further:
                telemetry.addData("doing", "orienting further");

                if (bottomColor.alpha() <= COLOR_OFF_WHITE_THRESHOLD) {
                    time_to_move = time - time_at_start;
                    changeState(State.Orienting_Back);
                }

                break;
            case Orienting_Back:
                telemetry.addData("doing", "orienting back");
                if (turning == Turn.Left) {
                    setTurnRight();
                } else {
                    setTurnLeft();
                }

                if (time - time_at_start > time_to_move / 2) {
                    changeState(State.Moving);

                    // Cut the motors so we don't go too far
                    setMotorsStopped();
                }
                break;
            case Moving:
                // Maybe add a hard limit on the time we can wait for the
                // distance sensor. Or not, IMO the distance sensor probably
                // won't fail and more likely it will make the robot drop the
                // climbers unnecessarily, screwing us over during the driver
                // period.

                telemetry.addData("doing", "moving");
                setMotorsForward();
                if (distance.getRawLightDetected() >= DISTANCE_MINIMUM) {
                    changeState(State.ScanningLeft);
                }
                if (time - time_at_start > 3.0f) {
                    changeState(State.Backing);
                }
                break;
            case ScanningLeft:

                // Try to guess this color!
                leftSideGuess = guessFrontColor();

                if (time - time_at_start < 1.0f && leftSideGuess == null) {
                    sSlide.setPower(SCAN_LEFT_SLIDE_POWER);
                } else {
                    // How long did we scan for?
                    time_to_scan = time - time_at_start;
                    changeState(State.CenteringFromLeft);
                }
                break;
            case CenteringFromLeft:
                if (time - time_at_start < time_to_scan) {
                    // Scan right for the same amount of time to center it.
                    sSlide.setPower(SCAN_RIGHT_SLIDE_POWER);
                } else {
                    changeState(State.ScanningRight);
                }
                break;
            case ScanningRight:
                rightSideGuess = guessFrontColor();

                if (time - time_at_start < time_to_scan && rightSideGuess == null) {
                    // Go the same distance to the right
                    sSlide.setPower(SCAN_RIGHT_SLIDE_POWER);
                } else {
                    changeState(State.CenteringFromRight);
                }
                break;
            case CenteringFromRight:
                if (time - time_at_start < time_to_scan) {
                    sSlide.setPower(SCAN_RIGHT_SLIDE_POWER);
                }
                changeState(State.Picking);
                break;
            case Picking:
                if (leftSideGuess == seekColor) {
                    // Go left!
                    changeState(State.GoLeft);
                } else if (rightSideGuess == seekColor) {
                    changeState(State.GoRight);
                } else {
                    // The other robot already took it from us, it's safe to
                    // pick either one and go for it, I believe.
                    changeState(State.GoRight);
                }
                break;
            case GoLeft:
                if (time - time_at_start < CLICKER_SLIDE_TIME) {
                    sSlide.setPower(-1.0f);
                } else {
                    changeState(State.Clicking);
                }
                break;
            case GoRight:
                if (time - time_at_start < CLICKER_SLIDE_TIME) {
                    sSlide.setPower(1.0f);
                } else {
                    changeState(State.Clicking);
                }
                break;
            case Clicking:
                if (time - time_at_start < CLICKING_FORWARD_TIME) {
                    setMotorsForward();
                } else {
                    changeState(State.Backing);
                }
                break;
            case Backing:
                // If it hasn't been 10 seconds it is very unsafe to backup.
                if (time < 10.0f) break;

                setMotorsBackward();

                if (time - time_at_start >= BACKING_UP_TIME) {
                    changeState(State.Done);
                }
                break;
            case Done:
            default:
                telemetry.addData("doing", "done");
                setMotorsStopped();
                break;
        }
        telemetry.addData("color", frontColor.alpha());
        telemetry.addData("distance", distance.getRawLightDetected());
        telemetry.addData("time_at_start", time_at_start);
        telemetry.addData("time", time);
    }

    private Color guessFrontColor() {
        float red_blue_ratio = frontColor.red() / (float) frontColor.blue();
        if (red_blue_ratio > 2.0f) {
            return Color.Red;
        }
        float blue_red_ratio = frontColor.blue() / (float) frontColor.red();
        if (blue_red_ratio > 2.0f) {
            return Color.Blue;
        }
        return null;
    }
}
