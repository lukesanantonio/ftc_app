package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AndrewAutoMode extends OpMode {

    // Constants
    private static final float MOTOR_POWER = 1.0f;

    private static final double TIME_TOWARDS_BALL = 1.0f;
    private static final double TIME_TURNING = 0.7f;

    private static final int HUE_DIFFERENCE_THRESHOLD = 10;

    private static final float BUTTON_PUSHER_POWER = .5f;
    private static final double BUTTON_PUSHER_TIME = .5f;

    public enum Turn {
        Right, Left
    }

    public enum Color {
        Red, Blue
    }

    public enum State {
        Begin,
        DrivingTowardsBall,
        DrivingBack,
        TurningTowardsWall,
        TurningToDriveByGhost,
        DrivingByGhost,
        FiguringOutColor,
        WaitingForColorChange,
        WaitingForButton,
        PushingButton,
        Done
    }

    // Motors and servos
    DcMotor mFrontRight;
    DcMotor mFrontLeft;
    DcMotor mBackRight;
    DcMotor mBackLeft;

    DcMotor mButtonPusher;

    // Miscellaneous Hardware
    ColorSensor frontColor;

    // Init state
    Turn turning = Turn.Right;
    Color seekColor = Color.Red;
    double timeDelay;

    // State
    State state;
    double time_at_start = 0.0;
    int hue = 0;
    double time_to_move = 0.0;
    double time_to_scan = 0.0f;
    Color firstColorGuess = null;
    Color secondColorGuess = null;

    public AndrewAutoMode(Turn turn, Color color, double delay) {
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

        mFrontRight.setDirection(DcMotor.Direction.REVERSE);
        mBackRight.setDirection(DcMotor.Direction.REVERSE);

        mButtonPusher = hardwareMap.dcMotor.get("button pusher");

        frontColor = hardwareMap.colorSensor.get("front");
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
                telemetry.addData("doing", "begin");
                resetStartTime();
                changeState(State.DrivingTowardsBall);
                break;
            case DrivingTowardsBall:
                telemetry.addData("doing", "driving towards ball");
                setMotorsForward();
                if (time - time_at_start >= TIME_TOWARDS_BALL) {
                    changeState(State.DrivingBack);
                }
                break;
            case DrivingBack:
                telemetry.addData("doing", "driving back");
                setMotorsBackward();
                if (time - time_at_start >= TIME_TOWARDS_BALL) {
                    changeState(State.Done);
                }
                break;
            case TurningTowardsWall:
                telemetry.addData("doing", "turning towards wall");
                if (turning == Turn.Right) {
                    setTurnRight();
                } else {
                    setTurnLeft();
                }
                if (time - time_at_start >= TIME_TURNING) {
                    setMotorsStopped();
                    changeState(State.TurningToDriveByGhost);
                }
                break;
            case TurningToDriveByGhost:
                telemetry.addData("doing", "turning to drive by ghost");
                if (turning == Turn.Right) {
                    setTurnLeft();
                } else {
                    setTurnRight();
                }
                if (time - time_at_start >= TIME_TURNING) {
                    // Figure out what the color sensor is reading
                    hue = frontColor.argb();
                    // Start moving forward
                    setMotorsForward();
                    // Start driving by the ghost
                    changeState(State.DrivingByGhost);
                }
                break;
            case DrivingByGhost:
                telemetry.addData("doing", "driving by ghost");
                // Wait for changes in the color sensor?
                if (Math.abs(frontColor.argb() - hue) > HUE_DIFFERENCE_THRESHOLD) {
                    // Something happened, we probably are now side-by-side
                    // with the ghost.
                    changeState(State.FiguringOutColor);
                }

                break;
            case FiguringOutColor:
                telemetry.addData("doing", "figuring out color");
                firstColorGuess = guessFrontColor();
                if (firstColorGuess == seekColor) {
                    // We found our color, go for the button
                    changeState(State.WaitingForButton);
                } else if (firstColorGuess != null) {
                    // It doesn't equal the seek color but is definitely a
                    // color.
                    changeState(State.WaitingForColorChange);
                    hue = frontColor.argb();
                }
                break;
            case WaitingForColorChange:
                telemetry.addData("doing", "waiting for color change");
                if (Math.abs(frontColor.argb() - hue) > HUE_DIFFERENCE_THRESHOLD) {
                    // The hue changed now, look at the color again
                    changeState(State.FiguringOutColor);
                }
                break;
            case WaitingForButton:
                telemetry.addData("doing", "waiting for button");
                if (Math.abs(frontColor.argb() - hue) > HUE_DIFFERENCE_THRESHOLD) {
                    // The hue changed now, that must be the button
                    setMotorsStopped();
                    changeState(State.PushingButton);
                }
                break;
            case PushingButton:
                telemetry.addData("doing", "pushing button");
                // Activate the button pusher
                mButtonPusher.setPower(BUTTON_PUSHER_POWER);
                if (time - time_at_start >= BUTTON_PUSHER_TIME) {
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
        telemetry.addData("timeAtStart", time_at_start);
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
