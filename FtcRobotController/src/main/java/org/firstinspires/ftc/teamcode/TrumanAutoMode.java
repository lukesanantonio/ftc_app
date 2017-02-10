package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class TrumanAutoMode extends OpMode {

    // = Constants

    // == Power
    private static final float SCAN_POWER = 1.0f;
    private static final float CLICK_POWER = 1.0f;
    private static final float APPROACH_POWER = 1.0f;
    private static final float FAST_POWER = 1.0f;
    private static final float MIN_TURN_POWER = .1f;
    private static final float MIN_APPROACH_POWER = .1f;

    // == Thresholds
    private static final int ANGLE_THRESHOLD = 4;
    private static final int DISTANCE_THRESHOLD = 2;

    // == Distances
    private static final int BALL_DISTANCE = 26;
    private static final int WALL_DISTANCE = 11;

    // Accept clicking from 10-4.
    private static final int CLICK_DISTANCE_THRESHOLD = 2;
    private static final int CLICK_DISTANCE = 5;

    // == Timers
    private static final float SHOOTING_DELAY = 1.3f;
    private static final float SECOND_SHOOTING_DELAY = 1.5f;
    private static final double INITIAL_SIDE_TIME = 1.5f;
    private static final double MAX_SCAN_TIME = 20.0f;
    private static final double IGNORING_TIME = 4.0f;

    // == Servo constants
    private static final float INITIAL_SLIDE_POSITION = .3f;
    private static final float FIRST_BALL_POSITION = .44f;
    private static final float SECOND_BALL_POSITION = .70f;

    // == Approaching constants
    private static final int APPROACH_START_SLOWDOWN = 50;

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
        Slide,
        Click,
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
    ColorSensor frontColor;

    RangeFinder range;

    // Init state
    Color seekColor = Color.Red;
    double timeDelay;

    // State
    State state;
    double timeAtStart;
    double timeToMove;
    double timeToScan;

    double lastSeenTime;
    int beaconCount;
    boolean useFrontWheels;

    // Stopped State
    double timeToStop;
    State afterStop;

    GyroSensor gyro;

    public TrumanAutoMode(Color color, double delay) {
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

        frontColor.enableLed(false);

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
        resetState();
    }

    private void changeState(State new_state) {
        timeAtStart = time;
        state = new_state;
    }

    private void stopState(double time, State next) {
        timeToStop = time;
        afterStop = next;
        changeState(State.Stopped);
    }

    private boolean isInAngleThreshold(float test_angle, float angle,
                                       float threshold) {
        return isInAngle(test_angle, angle - threshold, angle + threshold);
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

    private boolean isInThreshold(float test, float target, float threshold) {
        return (test > target - threshold) && (test < target + threshold);
    }

    public void resetState() {
        sSlide.setPosition(INITIAL_SLIDE_POSITION);

        timeAtStart = 0.0f;
        timeToMove = 0.0f;
        timeToScan = 0.0f;

        // These are just sane defaults that probably won't crash the robot, they shouldn't be used
        // as defaults that can be relied on.
        timeToStop = 1.0f;
        afterStop = State.Done;

        beaconCount = 0;
        lastSeenTime = 0.0f;

        changeState(State.Calibrating);
    }

    private boolean turnToAngle(int targetAngle) {
        return turnToAngle(targetAngle, gyro.getHeading(), ANGLE_THRESHOLD,
                MIN_TURN_POWER, true, true);
    }

    private boolean turnToAngle(int target_angle, int current_angle,
                                int threshold, float min_power)
    {
        return turnToAngle(target_angle, current_angle, threshold, min_power,
                true, true);
    }

    private boolean turnToAngle(int target_angle, int current_angle,
                                int threshold, float min_power,
                                boolean front, boolean back) {
        if (isInAngle(current_angle, target_angle - threshold,
                target_angle + threshold)) {
            // We are in the threshold just stop
            tank.stop();
            return true;
        }

        target_angle = target_angle % 360;
        current_angle = current_angle % 360;
        int dif = Math.abs(target_angle - current_angle);

        // Scale the power by difference from target angle
        if (dif > 180) {
            dif = Math.abs(target_angle - current_angle + 360);
        }

        // At 180 degrees or larger, run at full power
        float power = Math.max(min_power, Math.min(dif / 180.0f, 1.0f));

        telemetry.addData("target angle", target_angle);
        telemetry.addData("current angle", current_angle);

        if (isInAngle(current_angle, target_angle - 180, target_angle)) {
            tank.setTurnCW(power, front, back);
        } else if (isInAngle(current_angle, target_angle, target_angle + 180)) {
            tank.setTurnCCW(power, front, back);
        } else {
            telemetry.addData("power", power);
            telemetry.addData("dif", dif);
            telemetry.addData("current angle", current_angle);
            telemetry.addData("target angle", target_angle);
            tank.stop();
            return false;
        }

        return false;

    }

    private boolean approachWall(int targetDistance) {
        return approachWall(targetDistance, DISTANCE_THRESHOLD);
    }

    private boolean approachWall(int targetDistance, int threshold) {
        return approachWall(targetDistance, range.ultraSonic(),
                threshold, APPROACH_POWER, MIN_APPROACH_POWER,
                APPROACH_START_SLOWDOWN);
    }

    private boolean approachWall(int targetDistance, int threshold,
                                 float power) {
        return approachWall(targetDistance, range.ultraSonic(),
                threshold, power, MIN_APPROACH_POWER, APPROACH_START_SLOWDOWN);
    }

    private boolean approachWall(int targetDistance, int curDistance,
                                 int threshold, float power, float minPower,
                                 int divisor) {

        // We're in the threshold, stop
        if (Math.abs(curDistance - targetDistance) <= threshold) {
            tank.stop();
            return true;
        }

        telemetry.addData("currentDistance", curDistance);
        telemetry.addData("targetDistance", targetDistance);


        float dif = curDistance - targetDistance;

        // Distance is how far we are to the destination distance
        // in the range [0.0, 1.0] where 1.0 is farthest and 0.0f is
        // closest.
        float dist = Range.clip(dif / (float) divisor, -1.0f, 1.0f);

        power *= dist;
        float absPower = Math.abs(power);

        float actualPower = Math.max(minPower, absPower);

        if (power > 0.0f) {
            tank.setForward(actualPower);
        } else {
            tank.setBackward(actualPower);
        }

        return false;
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
                if (range.ultraSonic() <= BALL_DISTANCE) {
                    tank.stop();
                    changeState(State.ShootingFirstBall);
                }
                break;
            case ShootingFirstBall:
                telemetry.addData("doing", "shooting first ball");

                // The prop wheels should be at the right speed
                sSlide.setPosition(FIRST_BALL_POSITION);
                if (time - timeAtStart >= SHOOTING_DELAY) {
                    // Move on after the delay
                    changeState(State.ShootingSecondBall);
                }
                break;
            case ShootingSecondBall:
                telemetry.addData("doing", "shooting second ball");

                // The prop wheels should be at the right speed
                sSlide.setPosition(SECOND_BALL_POSITION);
                if (time - timeAtStart >= SECOND_SHOOTING_DELAY) {
                    // Move on after the delay
                    changeState(State.InitialSide);

                    // Start ramping down the props
                    mPropLeft.setPower(0.0f);
                    mPropRight.setPower(0.0f);
                }
                break;
            case InitialSide:
                telemetry.addData("doing", "initial side");

                if (seekColor == Color.Blue) {
                    tank.setRight(FAST_POWER);
                } else {
                    tank.setLeft(FAST_POWER);
                }

                if (time - timeAtStart >= INITIAL_SIDE_TIME) {
                    useFrontWheels = false;
                    changeState(State.Slide);
                }
                break;
            case Slide:
                telemetry.addData("doing", "slide");

                if (time - timeAtStart >= MAX_SCAN_TIME) {
                    changeState(State.Done);
                    break;
                }

                int angle = 85;
                if (seekColor == Color.Red) angle = 275;

                float minPower = .1f;
                if(useFrontWheels) {
                    minPower = .5f;
                }

                if (!turnToAngle(angle, gyro.getHeading(), ANGLE_THRESHOLD,
                        minPower,
                        useFrontWheels, true)) {
                    // If aren't straight enough, stop here
                    break;
                }

                useFrontWheels = true;

                if (!approachWall(WALL_DISTANCE)) {
                    // If we aren't close enough, stop here
                    break;
                }

                // If we already found the beacons and we are all set up just
                // stop.
                if (beaconCount >= 2) {
                    changeState(State.Done);
                    break;
                }

                // Now scan
                if (seekColor == Color.Blue) {
                    tank.setLeft(SCAN_POWER);
                } else {
                    tank.setRight(SCAN_POWER);
                }

                // Guess the color in front.
                if (guessFrontColor() == seekColor &&
                        time - lastSeenTime >= IGNORING_TIME) {
                    changeState(State.Click);
                    tank.stop();
                    break;
                }
                break;
            case Click:
                telemetry.addData("doing", "click");
                if (approachWall(CLICK_DISTANCE, CLICK_DISTANCE_THRESHOLD,
                        CLICK_POWER)) {
                    changeState(State.Slide);
                    ++beaconCount;
                    lastSeenTime = time;
                }
                break;
            case Stopped:
                telemetry.addData("doing", "stopped");
                if (time - timeAtStart <= timeToStop) {
                    tank.stop();
                } else {
                    changeState(afterStop);
                }
                break;
            case Done:
                telemetry.addData("doing", "done");
                tank.stop();
                break;
            default:
                telemetry.addData("doing", "default");
                break;
        }
        telemetry.addData("range", range.ultraSonic());
        telemetry.addData("gyro", gyro.getHeading());

        telemetry.addData("time", time);
        telemetry.addData("timeToMove", timeToMove);
        telemetry.addData("timeAtStart", timeAtStart);

        Color frontColor = guessFrontColor();
        if (frontColor != null) {
            telemetry.addData("curColorGuess", frontColor.toString());
        } else {
            telemetry.addData("curColorGuess", "unknown");
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
}
