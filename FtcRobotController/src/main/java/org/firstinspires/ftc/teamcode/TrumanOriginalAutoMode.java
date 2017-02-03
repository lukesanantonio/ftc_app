package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

public class TrumanOriginalAutoMode extends OpMode {

    // Constants
    private static final float MOTOR_POWER = 1.0f;
    private static final float SLOW_MOTOR_POWER = .1f;
    private static final float SLOW_TURN_POWER = .2f;

    private static final float TIME_STOPPED = 1.0f;

    private static final float RAMP_TIME = .6f;

    private static final float FIRST_TURING_TIME = 1.2f;

    private static final int DISTANCE_MINIMUM = 10;

    private static final float SCAN_LEFT_SLIDE_POWER = -1.0f;
    private static final float SCAN_RIGHT_SLIDE_POWER = 1.0f;

    private static final int COLOR_ON_WHITE_THRESHOLD = 3;
    private static final int COLOR_OFF_WHITE_THRESHOLD = 1;

    private static final double CLICKER_SLIDE_TIME = .4f;
    private static final double CLICKING_FORWARD_TIME = 1.0f;

    private static final double BACKING_UP_TIME = 1.0f;

    private static final double TIME_TOWARDS_BALL = 2.75f;
    private static final double TIME_FROM_BALL = 1.0f;

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
        DrivingTowardsBall,
        BackingFromBall,
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
    ColorSensor bottomColor;
    ColorSensor frontColor;

    RangeFinder range;

    // Init state
    Turn turning = Turn.Right;
    Color seekColor = Color.Red;
    double timeDelay;

    // State
    State state;
    double time_at_start;
    double time_to_move;
    double time_to_scan;
    Color leftSideGuess;
    Color rightSideGuess;

    public TrumanOriginalAutoMode(Turn turn, Color color, double delay) {
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
        sBallGuard.setPosition(0.5f);

        sSlide = hardwareMap.crservo.get("servo slide");
        sLeft = hardwareMap.servo.get("servo left");

        frontColor = hardwareMap.colorSensor.get("front");
        frontColor.setI2cAddress(I2cAddr.create8bit(0x2c));
        bottomColor = hardwareMap.colorSensor.get("bottom");
        bottomColor.setI2cAddress(I2cAddr.create8bit(0x1c));

        range = new RangeFinder(hardwareMap, "range");

        resetState();
    }

    @Override
    public void stop() {
    }

    @Override
    public void start() {
        resetState();
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

    public void resetState() {
        sLeft.setPosition(0.0);
        sSlide.setPower(0.0);

        time_at_start = 0.0f;
        time_to_move = 0.0f;
        time_to_scan = 0.0f;
        leftSideGuess = null;
        rightSideGuess = null;

        changeState(State.Begin);
    }

    @Override
    public void loop() {

        range.updateCache();

        switch (state) {
            case Begin:
                changeState(State.RampUpShootStageOne);
                break;
            case RampUpShootStageOne:
                telemetry.addData("doing", "ramp up shoot stage one");
                if (time - time_at_start < RAMP_TIME) {
                    mPropRight.setPower(.4f);
                    mPropLeft.setPower(.4f);
                } else {
                    changeState(State.RampUpShootStageTwo);
                }
                break;
            case RampUpShootStageTwo:
                telemetry.addData("doing", "ramp up shoot stage two");
                if (time - time_at_start < RAMP_TIME) {
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
                    changeState(State.RampDownShootStageOne);
                }
                break;
            case RampDownShootStageOne:
                telemetry.addData("doing", "ramp down shoot stage one");
                if (time - time_at_start < RAMP_TIME) {
                    mPropRight.setPower(.5f);
                    mPropLeft.setPower(.5f);
                } else {
                    changeState(State.RampDownShootStageTwo);
                }
                break;
            case RampDownShootStageTwo:
                telemetry.addData("doing", "ramp down shoot stage two");
                if (time - time_at_start < RAMP_TIME) {
                    mPropRight.setPower(.2f);
                    mPropLeft.setPower(.2f);
                } else {
                    mPropRight.setPower(0.0f);
                    mPropLeft.setPower(0.0f);
                    changeState(State.DrivingTowardsBall);
                }
                break;
            case DrivingTowardsBall:
                telemetry.addData("doing", "driving towards ball");
                setMotorsBackward();
                if (time - time_at_start >= TIME_TOWARDS_BALL) {
                    setMotorsStopped();
                    changeState(State.BackingFromBall);
                }
                break;
            case BackingFromBall:
                telemetry.addData("doing", "backing from ball");
                setMotorsForward();
                if (time - time_at_start >= TIME_FROM_BALL)
                {
                    setMotorsStopped();
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
        telemetry.addData("optical distance", range.optical());
        telemetry.addData("range distance", range.ultraSonic());
        telemetry.addData("time_at_start", time_at_start);
        telemetry.addData("time", time);
    }
}
