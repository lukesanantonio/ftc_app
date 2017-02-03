package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

public class TrumanAutoMode extends OpMode {

    // Constants
    private static final float FAST_MOTOR_POWER = .45f;
    private static final float SLOW_MOTOR_POWER = .13f;
    private static final float SLOW_TURN_POWER = .13f;
    private static final float MOTOR_POWER = .19f;
    private static final float FIND_LINE_SIDE_POWER = .20f;

    private static final float TIME_STOPPED = 1.0f;

    private static final float RAMP_TIME = .6f;

    //private static final float INITIAL_FORWARD_TIME = 1.0f;
    private static final float INITIAL_SIDE_TIME = 4.5f;

    private static final float FIRST_TURING_TIME = 1.2f;

    private static final int BEACON_DISTANCE_THRESHOLD = 15;
    private static final int BEACON_DISTANCE_CLOSER_THRESHOLD = 14;

    private static final int COLOR_ON_WHITE_THRESHOLD = 8;
    private static final int COLOR_OFF_WHITE_THRESHOLD = 3;

    private static final double CLICKER_SLIDE_TIME = .4f;
    private static final double CLICKING_FORWARD_TIME = 1.0f;

    private static final double BACKING_UP_TIME = 0.75f;

    private static final double TIME_TOWARDS_BALL = 2.75f;
    private static final double TIME_FROM_BALL = 1.0f;

    private static final int DEFAULT_ANGLE_THRESHOLD = 2;

    private static final float TURNING_CONSTANT = .05f;

    private static final float TIME_SLIDING_TO_NEXT_BEACON = 1.0f;

    private static final int BALL_DISTANCE_THRESHOLD = 29;

    private static final float INITIAL_SLIDE_POSITION = .3f;
    private static final float FIRST_BALL_POSITION = .44f;
    private static final float SECOND_BALL_POSITION = .70f;

    private static final float SHOOTING_DELAY = 1.3f;
    private static final float SECOND_SHOOTING_DELAY = 1.5f;

    public enum Turn {
        Right, Left
    }

    public enum Color {
        Red, Blue
    }

    public enum State {
        Calibrating,
        Start,
        InitialForward,
        ShootingFirstBall,
        ShootingSecondBall,
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

        frontColor = hardwareMap.colorSensor.get("front");
        frontColor.setI2cAddress(I2cAddr.create8bit(0x2c));
        bottomColor = hardwareMap.colorSensor.get("bottom");
        bottomColor.setI2cAddress(I2cAddr.create8bit(0x1c));

        frontColor.enableLed(false);
        bottomColor.enableLed(false);

        range = new RangeFinder(hardwareMap, "range");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

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
        setMotorsForward(MOTOR_POWER);
    }
    private void setMotorsForward(float power) {
        mFrontRight.setPower(power);
        mBackRight.setPower(power);
        mFrontLeft.setPower(power);
        mBackLeft.setPower(power);
    }

    private void setMotorsBackward() {
        setMotorsBackward(MOTOR_POWER);
    }
    private void setMotorsBackward(float power) {
        mFrontRight.setPower(-power);
        mBackRight.setPower(-power);
        mFrontLeft.setPower(-power);
        mBackLeft.setPower(-power);
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
        setMotorsRight(MOTOR_POWER);
    }
    private void setMotorsLeft() {
        setMotorsLeft(MOTOR_POWER);
    }
    private void setMotorsRight(float power) {
        mFrontRight.setPower(power);
        mBackRight.setPower(-power);

        mFrontLeft.setPower(power);
        mBackLeft.setPower(-power);
    }
    private void setMotorsLeft(float power) {
        mFrontRight.setPower(-power);
        mBackRight.setPower(power);

        mFrontLeft.setPower(-power);
        mBackLeft.setPower(power);
    }
    private void setMotorsRightSlow() {
        setMotorsRight(SLOW_MOTOR_POWER);
    }
    private void setMotorsLeftSlow() {
        setMotorsLeft(SLOW_MOTOR_POWER);
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
        sSlide.setPosition(INITIAL_SLIDE_POSITION);

        time_at_start = 0.0f;
        time_to_move = 0.0f;
        time_to_scan = 0.0f;
        leftSideGuess = null;
        rightSideGuess = null;

        // These are just sane defaults that probably won't crash the robot, they shouldn't be used
        // as defaults that can be relied on.
        time_to_stop = 1.0f;
        next_state = State.Done;

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

                // Ramp up the props
                mPropLeft.setPower(.3f);
                mPropRight.setPower(.3f);

                if (!gyro.isCalibrating()) {

                    // Second ramp up step
                    mPropLeft.setPower(.6f);
                    mPropRight.setPower(.6f);

                    // Don't start searching until the gyro is finished calibrating.
                    after_scan_state = State.SlidingToNextBeacon;
                    changeState(State.Start);
                }
                break;
            case Start:
                telemetry.addData("doing", "starting");

                // Finish the ramp up
                mPropLeft.setPower(.87f);
                mPropRight.setPower(.87f);

                if (time >= timeDelay) {
                    changeState(State.InitialForward);
                }

                break;
            case InitialForward:
                telemetry.addData("doing", "initial forward");
                setMotorsForward(FAST_MOTOR_POWER);
                if (range.ultraSonic() <= BALL_DISTANCE_THRESHOLD) {
                    setMotorsStopped();
                    changeState(State.ShootingFirstBall);
                }
                break;
            case ShootingFirstBall:
                telemetry.addData("doing", "shooting first ball");

                // The prop wheels should be at the right speed
                sSlide.setPosition(FIRST_BALL_POSITION);
                if (time - time_at_start >= SHOOTING_DELAY) {
                    // Move on after the delay
                    changeState(State.ShootingSecondBall);
                }
                break;
            case ShootingSecondBall:
                telemetry.addData("doing", "shooting second ball");

                // The prop wheels should be at the right speed
                sSlide.setPosition(SECOND_BALL_POSITION);
                if (time - time_at_start >= SECOND_SHOOTING_DELAY) {
                    // Move on after the delay
                    changeState(State.InitialSide);

                    // Start ramping down the props
                    mPropLeft.setPower(.4f);
                    mPropRight.setPower(.4f);
                }
                break;
            case InitialSide:
                telemetry.addData("doing", "initial side");

                // Mostly ramp down - let the motors spin for awhile.
                mPropLeft.setPower(.05f);
                mPropRight.setPower(.05f);

                if (turning == Turn.Left) {
                    setMotorsRight();
                } else {
                    setMotorsLeft();
                }

                if (time - time_at_start >= INITIAL_SIDE_TIME) {
                    setMotorsStopped();
                    changeState(State.Searching);
                }
                break;
            case Searching:

                // Totally ramp down
                mPropLeft.setPower(0.0f);
                mPropRight.setPower(0.0f);

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

                if (turning == Turn.Left) {
                    if (gyro.getHeading() < 90 || gyro.getHeading() > 270) {
                        setTurnRightSlow();
                    } else if (gyro.getHeading() > 90) {
                        setTurnLeftSlow();
                    }
                    if (robotIsAtAngle(90)) {
                        changeState(after_straighten);
                    }
                } else if (turning == Turn.Right) {
                    if (gyro.getHeading() < 90 || gyro.getHeading() > 270) {
                        setTurnLeftSlow();
                    } else if (gyro.getHeading() > 90) {
                        setTurnRightSlow();
                    }

                    if (robotIsAtAngle(270)) {
                        changeState(after_straighten);
                    }
                }
                break;
            case FindTheWhiteLine:
                telemetry.addData("doing", "finding the white line");

                if (turning == Turn.Left) {
                    setMotorsLeft(FIND_LINE_SIDE_POWER);
                } else {
                    setMotorsRight(FIND_LINE_SIDE_POWER);
                }

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
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
                if (time - time_at_start >= 1.0f && leftSideGuess == null) {
                    setMotorsStopped();
                    // Start approaching the beacon if we still can't find anything
                    changeState(State.ApproachingBeaconScanningColor);
                    break;
                } else if (time - time_at_start >= .5f && leftSideGuess != null) {
                    if (leftSideGuess == seekColor) {
                        changeState(State.Clicking);
                    } else {
                        // Move to the right
                        setMotorsStopped();
                        waitingForChangeFrom = leftSideGuess;
                        changeState(State.ScanningRight);
                    }
                }
                break;
            case ApproachingBeaconScanningColor:
                telemetry.addData("doing", "approaching beacon scanning color");

                if (leftSideGuess == null) {
                    // Guess the front color.
                    leftSideGuess = guessFrontColor();
                }

                setMotorsForwardSlow();
                if (range.ultraSonic() <= BEACON_DISTANCE_CLOSER_THRESHOLD && leftSideGuess == null) {
                    setMotorsStopped();
                    waitingForChangeFrom = leftSideGuess;
                    changeState(State.ScanningRight);
                } else if (leftSideGuess == seekColor) {
                    changeState(State.Clicking);
                }
                break;
            case ScanningRight:
                telemetry.addData("doing", "waiting for color change");
                setMotorsRight();
                rightSideGuess = guessFrontColor();
                if (rightSideGuess != null && rightSideGuess != waitingForChangeFrom) {
                    // We found a color
                    if (rightSideGuess == seekColor)
                    {
                        changeState(State.Clicking);
                    }
                    else
                    {
                        // Shit, we can't click it, try scanning again
                        changeState(after_scan_state);
                    }
                }
                else if(time - time_at_start >= 1.0f)
                {
                    changeState(after_scan_state);
                }
                break;
            case Clicking:
                telemetry.addData("doing", "clicking");

                setMotorsForward();

                if (time - time_at_start >= CLICKING_FORWARD_TIME) {
                    changeState(after_clicking);
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
        if(frontColor.blue() == 0)
        {
            red_blue_ratio = frontColor.red();
        }
        if (red_blue_ratio >= 4.0f) {
            return Color.Red;
        }
        float blue_red_ratio = frontColor.blue() / (float) frontColor.red();
        if (blue_red_ratio == 0)
        {
            blue_red_ratio = frontColor.blue();
        }
        if (blue_red_ratio >= 4.0f) {
            return Color.Blue;
        }

        telemetry.addData("red / blue ratio", red_blue_ratio);
        telemetry.addData("blue / red ratio", blue_red_ratio);
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
