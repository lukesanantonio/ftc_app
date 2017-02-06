package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TrumanAutoMode extends OpMode {

    // Constants

    private static final float SCAN_POWER = 0.3f;
    private static final float APPROACH_POWER = 0.5f;
    private static final float SEARCH_POWER = 0.15f;
    private static final float FAST_STRAIGHTENING_POWER = 0.45f;
    private static final float SLOW_STRAIGHTENING_POWER = 0.30f;
    private static final float FAST_POWER = 1.0f;
    private static final float CLICK_POWER = .3f;
    private static final float FIND_WHITE_LINE_POWER = 0.4f;

    private static final float INITIAL_SIDE_TIME = 2.0f;
    private static final float SCANNING_TIME = 1.0f;
    private static final double CLICKING_FORWARD_TIME = 1.0f;
    private static final double BACKING_UP_TIME = 0.4f;

    // Slowdown to a maximum of half-speed.
    private static final float BEACON_APPROACH_MAX_SLOWDOWN = 1.0f;
    private static final int BEACON_SLOW_DOWN_MAX = 55;
    private static final int BEACON_SLOW_DOWN_MIN = 10;
    private static final int BEACON_DISTANCE_THRESHOLD = 20;

    private static final int COLOR_ON_WHITE_THRESHOLD = 17;

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
        FindWhiteLineAgain,
        ScanningLeft,
        ScanningRight,
        Clicking,
        Backing,
        SlidingToNextBeacon,
        Stopped,
        Done
    }

    // Motors and servos
    TankHardware tank;

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

    float cur_straighten_power;
    int straighten_threshold;

    float not_moving_add_power;
    float dist_derive = 0.0f;

    boolean straighteningFoundWhiteLine = false;

    GyroSensor gyro;

    public TrumanAutoMode(Turn turn, Color color, double delay) {
        turning = turn;
        seekColor = color;
        timeDelay = delay;
    }

    @Override
    public void init() {
        tank = new TankHardware(hardwareMap);
        tank.setTelemetry(telemetry);

        mPropRight = hardwareMap.dcMotor.get("prop right");
        mPropLeft = hardwareMap.dcMotor.get("prop left");

        mPropLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mPropRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

    private void stopState(double time, State next) {
        time_to_stop = time;
        next_state = next;
        changeState(State.Stopped);
    }

    private boolean isInAngle(float angle, float begin, float end) {
        // Normalize values
        begin = begin % 360.0f;
        end = end % 360.0f;
        angle = angle % 360.0f;

        if (end < begin) {
            return angle < end || begin < angle;
        } else {
            return begin < angle && angle < end;
        }
    }

    /*
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
    */
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

        cur_straighten_power = FAST_STRAIGHTENING_POWER;
        straighten_threshold = 4;

        not_moving_add_power = 0.0f;

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
                tank.setForward(FAST_POWER);
                if (range.ultraSonic() <= BALL_DISTANCE_THRESHOLD) {
                    tank.stop();
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

                // Ramp down completely - the wheels may spin for some time.
                mPropLeft.setPower(0.0f);
                mPropRight.setPower(0.0f);

                if (turning == Turn.Left) {
                    tank.setRight(FAST_POWER);
                } else {
                    tank.setLeft(FAST_POWER);
                }

                if (time - time_at_start >= INITIAL_SIDE_TIME) {
                    tank.stop();
                    changeState(State.Searching);
                }
                break;
            case Searching:
                telemetry.addData("doing", "searching");
                tank.setForward(SEARCH_POWER);

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    // This state only runs the first time, so make sure that
                    // after we scan we go back to finding the white line again.
                    after_scan_state = State.SlidingToNextBeacon;
                    after_straighten = State.FindTheWhiteLine;
                    after_clicking = State.Backing;
                    cur_straighten_power = FAST_STRAIGHTENING_POWER;
                    straighten_threshold = 5;
                    stopState(2.0f, State.Straightening);
                }
                break;
            case Straightening:
                telemetry.addData("doing", "straightening");

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    straighteningFoundWhiteLine = true;
                }

                if (turning == Turn.Left) {
                    if (robotIsAtAngle(90, straighten_threshold)) {
                        stopState(2.0f, after_straighten);
                    } else if (gyro.getHeading() < 90 || gyro.getHeading() > 270) {
                        tank.setTurnCW(cur_straighten_power, !turnWithBackOnly,
                                true);
                    } else if (gyro.getHeading() > 90) {
                        tank.setTurnCCW(cur_straighten_power, !turnWithBackOnly,
                                true);
                    }
                } else if (turning == Turn.Right) {
                    if (robotIsAtAngle(270, straighten_threshold)) {
                        stopState(2.0f, after_straighten);
                    } else if (gyro.getHeading() < 90 || gyro.getHeading() > 270) {
                        tank.setTurnCCW(cur_straighten_power, !turnWithBackOnly,
                                true);
                    } else if (gyro.getHeading() > 90) {
                        tank.setTurnCW(cur_straighten_power, !turnWithBackOnly,
                                true);
                    }
                }
                break;
            case FindTheWhiteLine:
                telemetry.addData("doing", "finding the white line");

                if (turning == Turn.Left) {
                    tank.setLeft(FIND_WHITE_LINE_POWER);
                } else {
                    tank.setRight(FIND_WHITE_LINE_POWER);
                }

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    // We found the white line
                    tank.stop();

                    // Approach the beacon
                    turnWithBackOnly = true;
                    cur_straighten_power = SLOW_STRAIGHTENING_POWER;
                    straighten_threshold = 2;
                    straighteningFoundWhiteLine = false;
                    stopState(2.0f, State.Straightening);
                    after_straighten = State.ApproachingBeacon;
                    after_approaching = State.FindWhiteLineAgain;
                }
                break;
            case ApproachingBeacon:
                // Maybe add a hard limit on the time we can wait for the
                // distance sensor. Or not, IMO the distance sensor probably
                // won't fail and more likely it will make the robot drop the
                // climbers unnecessarily, screwing us over during the driver
                // period.

                telemetry.addData("doing", "approaching beacon");

                float approach_power = APPROACH_POWER;

                int divisor =
                        BEACON_SLOW_DOWN_MAX - BEACON_SLOW_DOWN_MIN;

                float dif = range.ultraSonic() - BEACON_SLOW_DOWN_MIN;
                // Distance is how far we are to the destination distance
                // in the range [0.0, 1.0] where 1.0 is farthest and 0.0f is
                // closest.
                float dist = Range.clip(dif / (float) divisor, 0.0f, 1.0f);

                float slowdown_fac = 1.0f - dist;
                approach_power -=
                        approach_power * slowdown_fac * BEACON_APPROACH_MAX_SLOWDOWN;
                telemetry.addData("dist", dist);

                approach_power += not_moving_add_power;

                tank.setForward(approach_power);
                if (range.ultraSonic() <= BEACON_DISTANCE_THRESHOLD) {
                    tank.stop();
                    stopState(2.0f, after_approaching);
                }
                break;
            case FindWhiteLineAgain:
                telemetry.addData("doing", "finding white line again");

                // If we found the white line during the straightening move
                // sideways until we find the white line
                if ((straighteningFoundWhiteLine && turning == Turn.Left) ||
                        (!straighteningFoundWhiteLine && turning == Turn.Right)) {
                    // Move left until we find the white
                    tank.setLeft(SCAN_POWER);
                } else if ((straighteningFoundWhiteLine && turning == Turn.Right) ||
                        (!straighteningFoundWhiteLine && turning == Turn.Left)) {
                    // Move right until we find the white line, or after a
                    // period of time, no color.
                    tank.setRight(SCAN_POWER);
                }

                if (bottomColor.alpha() >= COLOR_ON_WHITE_THRESHOLD) {
                    // Move on to scanning.
                    stopState(2.0f, State.ScanningLeft);
                }
            case ScanningLeft:
                telemetry.addData("doing", "scanning left");

                tank.setLeft(SCAN_POWER);

                // Try to guess this color!
                leftSideGuess = guessFrontColor();

                if (leftSideGuess == seekColor) {
                    // Click!
                    tank.stop();
                    changeState(State.Clicking);
                } else if (leftSideGuess != null ||
                        time - time_at_start >= SCANNING_TIME) {
                    // Get outta here and scan right
                    tank.stop();
                    waitingForChangeFrom = leftSideGuess;
                    changeState(State.ScanningRight);
                }
                break;
            case ScanningRight:
                telemetry.addData("doing", "scanning right");

                tank.setRight(SCAN_POWER);

                rightSideGuess = guessFrontColor();

                if (rightSideGuess == seekColor) {
                    tank.stop();
                    changeState(State.Clicking);
                } else if (time - time_at_start >= SCANNING_TIME) {
                    // Don't click, just try the next beacon if applicable.
                    tank.stop();
                    changeState(after_scan_state);
                }
                break;
            case Clicking:
                telemetry.addData("doing", "clicking");

                tank.setForward(CLICK_POWER);

                if (time - time_at_start >= CLICKING_FORWARD_TIME) {
                    changeState(after_clicking);
                }
                break;
            case Backing:
                telemetry.addData("doing", "backing");

                tank.setBackward(CLICK_POWER);

                if (time - time_at_start >= BACKING_UP_TIME) {
                    changeState(after_scan_state);
                    tank.stop();
                }
                break;
            case SlidingToNextBeacon:
                telemetry.addData("doing", "sliding to next beacon");
                // This state just sets up find the white line mode to finish by going into the done
                // state.
                if (turning == Turn.Left) {
                    tank.setLeft(FAST_POWER);
                } else {
                    tank.setRight(FAST_POWER);
                }
                // Well, we have to make sure we get off the white line first
                if (time - time_at_start >= TIME_SLIDING_TO_NEXT_BEACON) {
                    // Partially reset state
                    turnWithBackOnly = false;
                    after_scan_state = State.Done;
                    after_clicking = State.Done;
                    changeState(State.FindTheWhiteLine);
                }
                break;
            case Stopped:
                if (time - time_at_start <= time_to_stop) {
                    tank.stop();
                } else {
                    changeState(next_state);
                }
                break;
            case Done:
            default:
                telemetry.addData("doing", "done");
                tank.stop();
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
        if (leftSideGuess != null) {
            telemetry.addData("leftSideGuess", leftSideGuess.toString());
        } else {
            telemetry.addData("leftSideGuess", "null");
        }
        if (rightSideGuess != null) {
            telemetry.addData("rightSideGuess", rightSideGuess.toString());
        } else {
            telemetry.addData("rightSideGuess", "null");
        }

        Color frontColor = guessFrontColor();
        if (frontColor != null) {
            telemetry.addData("curColorGuess", frontColor.toString());
        } else {
            telemetry.addData("curColorGuess", "null");
        }

        tank.logPower();
    }

    private Color guessFrontColor() {
        float red_blue_ratio = frontColor.red() / (float) frontColor.blue();
        if (frontColor.blue() == 0) {
            red_blue_ratio = frontColor.red();
        }
        if (red_blue_ratio >= 4.0f) {
            return Color.Red;
        }
        float blue_red_ratio = frontColor.blue() / (float) frontColor.red();
        if (blue_red_ratio == 0) {
            blue_red_ratio = frontColor.blue();
        }
        if (blue_red_ratio >= 4.0f) {
            return Color.Blue;
        }

        telemetry.addData("red / blue ratio", red_blue_ratio);
        telemetry.addData("blue / red ratio", blue_red_ratio);
        return null;
    }

    public boolean robotIsAtAngle(int angle) {
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
