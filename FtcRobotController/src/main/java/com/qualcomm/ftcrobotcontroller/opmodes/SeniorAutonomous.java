package com.qualcomm.ftcrobotcontroller.opmodes;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;

/**
 * Created by localt on 1/15/2016.
 */
public class SeniorAutonomous extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;
    DcMotor motorHook;

    ElapsedTime time;

    @Override
    public void init() {

        motorFrontRight = hardwareMap.dcMotor.get("front right");
        motorFrontLeft = hardwareMap.dcMotor.get("front left");
        motorBackRight = hardwareMap.dcMotor.get("back right");
        motorBackLeft = hardwareMap.dcMotor.get("back left");
        motorLift = hardwareMap.dcMotor.get("scissor lift");
        motorHook = hardwareMap.dcMotor.get("hook");

    }

        @Override
        public void loop () {

            while (time.time() < 1.0) {

                motorFrontRight.setPower(0.5);
                motorFrontLeft.setPower (0.5);
                motorBackRight.setPower (0.5);
                motorBackLeft.setPower  (0.5);

            }

            while (time.time() > 1.0 && time.time() < 2.5) {

                motorFrontRight.setPower(-1.0);
                motorFrontLeft.setPower ( 1.0);
                motorBackRight.setPower (-1.0);
                motorBackLeft.setPower  ( 1.0);

            }

            while (time.time() > 2.5 && time.time() < 5.0) {

                motorFrontRight.setPower(1.0);
                motorFrontLeft.setPower (1.0);
                motorBackRight.setPower (1.0);
                motorBackLeft.setPower  (1.0);
        }

            while (time.time() > 5.0) {

                motorFrontRight.setPower(0);
                motorFrontLeft.setPower (0);
                motorBackRight.setPower (0);
                motorBackLeft.setPower  (0);
            }

        }

    }