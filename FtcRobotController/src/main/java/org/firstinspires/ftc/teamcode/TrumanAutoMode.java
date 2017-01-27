package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class TrumanAutoMode extends OpMode {

    // Constants
    private static final float MOTOR_POWER = .2f;
    private static final float SLOW_MOTOR_POWER = .13f;
    private static final float SLOW_TURN_POWER = .13f;

    private static final float TIME_STOPPED = 1.0f;

    private static final float RAMP_TIME = .6f;

    //private static final float INITIAL_FORWARD_TIME = 1.0f;
    private static final float INITIAL_SIDE_TIME = 4.3f;

    private static final float SLIDE_START_POSITION = .3f;
    private static final float SLIDE_STOP_POSITION = .8f;

    private static final float FIRST_TURING_TIME = 1.2f;

    private static final int BEACON_DISTANCE_THRESHOLD = 17;

    private static final int COLOR_ON_WHITE_THRESHOLD = 8;
    private static final int COLOR_OFF_WHITE_THRESHOLD = 3;

    private static final double CLICKER_SLIDE_TIME = .4f;
    private static final double CLICKING_FORWARD_TIME = 1.0f;

    private static final double BACKING_UP_TIME = 0.25f;

    private static final double TIME_TOWARDS_BALL = 2.75f;
    private static final double TIME_FROM_BALL = 1.0f;

    private static final int DEFAULT_ANGLE_THRESHOLD = 2;

    private static final float TURNING_CONSTANT = .05f;

    private static final float TIME_SLIDING_TO_NEXT_BEACON = 1.0f;

    private static final int BALL_DISTANCE_THRESHOLD = 21;

    public enum Turn {
        Right, Left
    }

    public enum Color {
        Red, Blue
    }

    public enum State {
        Calibrating,
        RampUpShootStageOne,
        RampUpShootStageTwo,
        ShootingBalls,
        RampDownShootStageOne,
        RampDownShootStageTwo,
        DrivingTowardsBall,
        BackingFromBall,
        Turning,
        Start,
        InitialForward,
        InitialSide,
        Searching,
        Straightening,
        FindTheWhiteLine,
        ApproachingBeacon,
        ScanningLeft,
        ApproachingBeaconScanningColor,
        WaitingForColorChange,
        WaitingForNewColor,
        WaitingForChangeFrom,
        CenteringFromLeft,
        ScanningRight,
        CenteringFromRight,
        Picking,
        GoLeft,
        GoRight,
        Clicking,
        Backing,
        SlidingToNextBeacon,
        Stopped,
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

    Servo sSlide;

    // Miscellaneous Hardware
    ColorSensor bottomColor;
    ColorSensor frontColor;

    RangeSensor range;

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

    Color waitingForChangeFrom;

    // Stopped State
    double time_to_stop;
    State next_state;

    State after_scan_state;
    State after_approaching;
    State after_straighten;
    State after_clicking;

    boolean turnWithBackOnly;

    GyroSensor gyro;

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

        sSlide = hardwareMap.servo.get("servo slide");
        sSlide.setPosition(SLIDE_START_POSITION);

        frontColor = hardwareMap.colorSensor.get("front");
        frontColor.setI2cAddress(I2cAddr.create8bit(0x2c));
        bottomColor = hardwareMap.colorSensor.get("bottom");
        bottomColor.setI2cAddress(I2cAddr.create8bit(0x1c));

        frontColor.enableLed(false);
        bottomColor.enableLed(false);

        range = new RangeSensor(hardwareMap, "range");
        gyro = hardwareMap.gyroSensor.get("gyro");

        resetState();
    }

    @Override
    public void stop() {
    }

    @Override
    public void start() {
        bottomColor.enableLed(true);
        resetState();
    }

    private void changeState(State new_state) {
        time_at_start = time;
        state = new_state;
    }

    private void stopState(double time, State next)
    {
        time_to_stop = time;
        next_state = next;
        changeState(State.Stopped);
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
    private void setMotorsForwardSlow() {
        mFrontRight.setPower(SLOW_MOTOR_POWER);
        mBackRight.setPower(SLOW_MOTOR_POWER);
        mFrontLeft.setPower(SLOW_MOTOR_POWER);
        mBackLeft.setPower(SLOW_MOTOR_POWER);
    }

    private void setMotorsBackwardSlow() {
        mFrontRight.setPower(-SLOW_MOTOR_POWER);
        mBackRight.setPower(-SLOW_MOTOR_POWER);
        mFrontLeft.setPower(-SLOW_MOTOR_POWER);
        mBackLeft.setPower(-SLOW_MOTOR_POWER);
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
    private void setTurnRightSlow() {
        if(!turnWithBackOnly) {
            mFrontRight.setPower(-SLOW_TURN_POWER);
            mBackRight.setPower(-SLOW_TURN_POWER);
            mFrontLeft.setPower(SLOW_TURN_POWER);
            mBackLeft.setPower(SLOW_TURN_POWER);
        }
        else
        {
            // Our front right and back right are switched
            mFrontLeft.setPower(0.0f);
            mBackRight.setPower(0.0f);
            mBackLeft.setPower(SLOW_TURN_POWER);
            mFrontRight.setPower(-SLOW_TURN_POWER);
        }
    }

    private void setTurnLeftSlow() {
        if(!turnWithBackOnly) {
            mFrontRight.setPower(SLOW_TURN_POWER);
            mBackRight.setPower(SLOW_TURN_POWER);

            mFrontLeft.setPower(-SLOW_TURN_POWER);
            mBackLeft.setPower(-SLOW_TURN_POWER);
        }
        else
        {
            mFrontLeft.setPower(0.0f);
            mBackRight.setPower(0.0f);
            mBackLeft.setPower(-SLOW_TURN_POWER);
            mFrontRight.setPower(SLOW_TURN_POWER);
        }
    }


    private void setMotorsRight() {
        mFrontRight.setPower(MOTOR_POWER);
        mBackRight.setPower(-MOTOR_POWER);

        mFrontLeft.setPower(MOTOR_POWER);
        mBackLeft.setPower(-MOTOR_POWER);
    }
    private void setMotorsLeft() {
        mFrontRight.setPower(-MOTOR_POWER);
        mBackRight.setPower(MOTOR_POWER);

        mFrontLeft.setPower(-MOTOR_POWER);
        mBackLeft.setPower(MOTOR_POWER);
    }
    private void setMotorsRightSlow() {
        mFrontRight.setPower(SLOW_MOTOR_POWER);
        mBackRight.setPower(-SLOW_MOTOR_POWER);

        mFrontLeft.setPower(SLOW_MOTOR_POWER);
        mBackLeft.setPower(-SLOW_MOTOR_POWER);
    }
    private void setMotorsLeftSlow() {
        mFrontRight.setPower(-SLOW_MOTOR_POWER);
        mBackRight.setPower(SLOW_MOTOR_POWER);

        mFrontLeft.setPower(-SLOW_MOTOR_POWER);
        mBackLeft.setPower(SLOW_MOTOR_POWER);
    }

    private boolean isInAngle(float angle, float begin, float end)
    {
        // Normalize values
        begin = begin % 360.0f;
        end = end % 360.0f;
        angle = angle % 360.0f;

        if (end < begin)
        {
            return angle < end || begin < angle;
        }
        else
        {
            return begin < angle && angle < end;
        }
    }

    private void setMotorsForwardChecked(float target_angle) {
        // Figure out direction
        int upper_bound = ((int) target_angle + 178) % 360;
        int lower_bound = ((int) target_angle - 178) % 360;

        float current_gyro = gyro.getHeading();

        float turning_factor = Math.abs(current_gyro - target_angle) * TURNING_CONSTANT;

        telemetry.addData("Turning factor", turning_factor);

        if (isInAngle(current_gyro, lower_bound, target_angle))
        {
            // If the gyro is between target and lower add some power to the left.
            // Add power to the right
            mFrontRight.setPower(SLOW_MOTOR_POWER + turning_factor);
            mBackRight.setPower(SLOW_MOTOR_POWER + turning_factor);
            mFrontLeft.setPower(SLOW_MOTOR_POWER);
            mBackLeft.setPower(SLOW_MOTOR_POWER);

            telemetry.addData("turning factor direction", "left");
        }
        else if(isInAngle(current_gyro, target_angle, upper_bound))
        {
            // Add power to the left
            mFrontRight.setPower(SLOW_MOTOR_POWER);
            mBackRight.setPower(SLOW_MOTOR_POWER);
            mFrontLeft.setPower(SLOW_MOTOR_POWER + turning_factor);
            mBackLeft.setPower(SLOW_MOTOR_POWER + turning_factor);

            telemetry.addData("turning factor direction", "right");
        }
        else
        {
            setTurnRight();
        }

    }

    public void resetState() {
        sSlide.setPosition(SLIDE_START_POSITION);

        time_at_start = 0.0f;
        time_to_move = 0.0f;
        time_to_scan = 0.0f;
        leftSideGuess = null;
        rightSideGuess = null;

        // These are just sane defaults that probably won't crash the robot, they shouldn't be used
        // as defaults that can be relied on.
        time_to_stop = 1.0f;
        next_state = State.Done;

        // Calibrate the driver state
        gyro.calibrate();

        turnWithBackOnly = false;
        after_clicking = State.Backing;

        changeState(State.Calibrating);
    }

    @Override
    public void loop() {

        range.updateCache();

        switch (state) {
            case Calibrating:
                telemetry.addData("doing", "calibrating");
                if(!gyro.isCalibrating()) {
                    // Don't start searching until the gyro is finished calibrating.
                    after_scan_state = State.SlidingToNextBeacon;
                    changeState(State.Start);
                }
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
                    sSlide.setPosition(SLIDE_STOP_POSITION);
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
                    changeState(State.Start);
                }
                break;
            case DrivingTowardsBall:
                telemetry.addData("doing", "driving towards ball");
                setMotorsBackward();
                if (time - time_at_start >= TIME_TOWARDS_BALL) {
                    setMotorsStopped();
                    changeState(State.Start);
                }
                break;
            case Start:
                telemetry.addData("doing", "starting");

                if (time >= timeDelay) {
                    changeState(State.InitialForward);
                }

                break;
            case InitialForward:
                setMotorsForward();
                if(range.ultraSonic() <= BALL_DISTANCE_THRESHOLD)
                {
                    setMotorsStopped();
                    changeState(State.InitialSide);
                }
                break;
            case InitialSide:
                if(turning == Turn.Left) {
                    setMotorsRight();
                }
                else
                {
                    setMotorsLeft();
                }

                if(time - time_at_start >= INITIAL_SIDE_TIME)
                {
                    setMotorsStopped();
                    changeState(State.Searching);
                }
                break;
            case BackingFromBall:
                telemetry.addData("doing", "backing from ball");
                setMotorsForward();
                if (time - time_at_start >= TIME_FROM_BALL)
                {
                    setMotorsStopped();
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
                if (time - time_at_start > FIRST_TURING_TIME) {
                    changeState(State.Searching);
                }
                break;
            case Searching:
                telemetry.addData("doing", "searching");
                setMotorsForwardSlow();

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    // This state only runs the first time, so make sure that after we scan we go back
                    // to finding the white line again.
                    after_scan_state = State.SlidingToNextBeacon;
                    after_straighten = State.FindTheWhiteLine;
                    after_clicking = State.Backing;
                    changeState(State.Straightening);
                }
                break;
            case Straightening:
                telemetry.addData("doing", "straightening");

                if(turning == Turn.Left)
                {
                    if(gyro.getHeading() < 90 || gyro.getHeading() > 270)
                    {
                        setTurnRightSlow();
                    }
                    else if(gyro.getHeading() > 90)
                    {
                        setTurnLeftSlow();
                    }
                    if(robotIsAtAngle(90))
                    {
                        changeState(after_straighten);
                    }
                }
                else if(turning == Turn.Right)
                {
                    if(gyro.getHeading() < 90 || gyro.getHeading() > 270) {
                        setTurnLeftSlow();
                    }
                    else if(gyro.getHeading() > 90)
                    {
                        setTurnRightSlow();
                    }

                    if(robotIsAtAngle(270))
                    {
                        changeState(after_straighten);
                    }
                }
                break;
            case FindTheWhiteLine:
                telemetry.addData("doing", "finding the white line");

                if(turning == Turn.Left)
                {
                    setMotorsLeft();
                }
                else
                {
                    setMotorsRight();
                }

                if(bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD)
                {
                    // We found the white line
                    setMotorsStopped();
                    // If we haven't straightened like this do this
                    after_straighten = State.ApproachingBeacon;
                    after_approaching = State.ScanningLeft;
                    turnWithBackOnly = true;
                    changeState(State.Straightening);
                }
                break;
            case ApproachingBeacon:
                // Maybe add a hard limit on the time we can wait for the
                // distance sensor. Or not, IMO the distance sensor probably
                // won't fail and more likely it will make the robot drop the
                // climbers unnecessarily, screwing us over during the driver
                // period.

                telemetry.addData("doing", "approaching beacon");
                setMotorsForwardSlow();
                if (range.ultraSonic() <= BEACON_DISTANCE_THRESHOLD) {
                    setMotorsStopped();
                    changeState(after_approaching);
                }
                break;
            case ScanningLeft:
                telemetry.addData("doing", "scanning left");

                // Try to guess this color!
                leftSideGuess = guessFrontColor();

                setMotorsLeftSlow();
                if(time - time_at_start >= 1.0f && leftSideGuess == null)
                {
                    setMotorsStopped();
                    // Start approaching the beacon if we still can't find anything
                    changeState(State.ApproachingBeaconScanningColor);
                    break;
                }
                else if(leftSideGuess != null)
                {
                    if(leftSideGuess == seekColor)
                    {
                        changeState(State.Clicking);
                    }
                    else
                    {
                        // Move to the right
                        setMotorsStopped();
                        waitingForChangeFrom = leftSideGuess;
                        changeState(State.WaitingForColorChange);
                    }
                }
                break;
            case ApproachingBeaconScanningColor:
                telemetry.addData("doing", "approaching beacon scanning color");

                if(leftSideGuess == null)
                {
                    // Guess the front color.
                    leftSideGuess = guessFrontColor();
                }

                setMotorsForwardSlow();
                if (range.ultraSonic() <= BEACON_DISTANCE_THRESHOLD) {
                    setMotorsStopped();
                    waitingForChangeFrom = leftSideGuess;
                    changeState(State.WaitingForColorChange);
                }
                break;
            case WaitingForColorChange:
                telemetry.addData("doing", "waiting for color change");
                setMotorsRight();
                if (guessFrontColor() != waitingForChangeFrom)
                {
                    changeState(State.WaitingForNewColor);
                }
                break;
            case WaitingForNewColor:
                telemetry.addData("doing", "waiting for new color");
                setMotorsRight();
                rightSideGuess = guessFrontColor();
                if(rightSideGuess != null || time - time_at_start >= .5f)
                {
                    // No matter if this is the right color, if we got here it means we should click
                    // the button
                    setMotorsStopped();
                    changeState(State.Clicking);
                }
                break;
            case Clicking:
                telemetry.addData("doing", "clicking");

                setMotorsForward();

                if (time - time_at_start >= CLICKING_FORWARD_TIME) {
                    changeState(State.Backing);
                }
                break;
            case Backing:
                telemetry.addData("doing", "backing");

                setMotorsBackward();

                if (time - time_at_start >= BACKING_UP_TIME) {
                    changeState(after_scan_state);
                    setMotorsStopped();
                }
                break;
            case SlidingToNextBeacon:
                telemetry.addData("doing", "sliding to next beacon");
                // This state just sets up find the white line mode to finish by going into the done
                // state.
                if(turning == Turn.Left)
                {
                    setMotorsLeft();
                }
                else
                {
                    setMotorsRight();
                }
                // Well, we have to make sure we get off the white line first
                if(time - time_at_start >= TIME_SLIDING_TO_NEXT_BEACON) {
                    // Partially reset state
                    turnWithBackOnly = false;
                    after_scan_state = State.Done;
                    after_clicking = State.Done;
                    changeState(State.FindTheWhiteLine);
                }
                break;
            case Stopped:
                if (time - time_at_start <= time_to_stop)
                {
                    setMotorsStopped();
                }
                else
                {
                    changeState(next_state);
                }
                break;
            case Done:
            default:
                telemetry.addData("doing", "done");
                setMotorsStopped();
                break;
        }
        telemetry.addData("front color", frontColor.alpha());
        telemetry.addData("bottom color", bottomColor.alpha());
        telemetry.addData("optical distance", range.optical());
        telemetry.addData("range distance", range.ultraSonic());
        telemetry.addData("time_to_move", time_to_move);
        telemetry.addData("time_at_start", time_at_start);
        telemetry.addData("gyro heading", gyro.getHeading());
        telemetry.addData("time", time);
        if(leftSideGuess != null) {
            telemetry.addData("leftSideGuess", leftSideGuess.toString());
        } else
        {
            telemetry.addData("leftSideGuess", "null");
        }
        if(rightSideGuess != null) {
            telemetry.addData("rightSideGuess", rightSideGuess.toString());
        }
        else
        {
            telemetry.addData("rightSideGuess", "null");
        }

        Color frontColor = guessFrontColor();
        if(frontColor != null) {
            telemetry.addData("curColorGuess", frontColor.toString());
        } else
        {
            telemetry.addData("curColorGuess", "null");
        }
    }

    private Color guessFrontColor() {
        float red_blue_ratio = frontColor.red() / (float) frontColor.blue();
        if (red_blue_ratio > 3.0f) {
            return Color.Red;
        }
        float blue_red_ratio = frontColor.blue() / (float) frontColor.red();
        if (blue_red_ratio > 3.0f) {
            return Color.Blue;
        }
        return null;
    }

    public boolean robotIsAtAngle(int angle)
    {
        return robotIsAtAngle(angle, DEFAULT_ANGLE_THRESHOLD);
    }
    public boolean robotIsAtAngle(int angle, int threshold) {
        // Find the upper and the lower bound
        int upper = angle + Math.abs(threshold);
        int lower = angle - Math.abs(threshold);

        // Make sure they are in the proper range
        upper = upper % 360;
        lower = lower % 360;

        int heading = gyro.getHeading();

        return lower <= heading && heading <= upper;
    }
}
