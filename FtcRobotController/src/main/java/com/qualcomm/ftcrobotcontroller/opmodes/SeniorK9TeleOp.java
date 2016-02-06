package com.qualcomm.ftcrobotcontroller.opmodes;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;

/**
 * Created by localt on 10/24/2015.
 */
public class SeniorK9TeleOp extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;
    DcMotor motorHook;

    Servo claw;
    Servo red;
    Servo blue;

    final double RESET_RED_SERVO = 0.0;
    final double ACTIVATE_RED_SERVO = 1.0;

    final double RESET_BLUE_SERVO = 1.0;
    final double ACTIVATE_BLUE_SERVO = 0.0;

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
        motorFrontRight = hardwareMap.dcMotor.get("front right");
        motorFrontLeft  = hardwareMap.dcMotor.get("front left");
        motorBackRight  = hardwareMap.dcMotor.get("back right");
        motorBackLeft   = hardwareMap.dcMotor.get("back left");
        motorLift       = hardwareMap.dcMotor.get("scissor lift");
        motorHook       = hardwareMap.dcMotor.get("hook");

        claw = hardwareMap.servo.get("claw");
        red  = hardwareMap.servo.get("red");
        blue = hardwareMap.servo.get("blue");
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        if(gamepad1.b)
        {
             blue.setPosition(ACTIVATE_BLUE_SERVO);
        }
       /* else if(gamepad1.y)
        {
            blue.setPosition(ACTIVATE_BLUE_SERVO);
            red.setPosition(ACTIVATE_RED_SERVO);
        } */
        else if(gamepad1.x)
        {
            red.setPosition(ACTIVATE_RED_SERVO);
        }
        else if(gamepad1.a)
        {
            red.setPosition(RESET_RED_SERVO);
            blue.setPosition(RESET_BLUE_SERVO);
        }

        if(gamepad2.a)
        {
            claw.setPosition(0.4);
        }

        if(gamepad2.b)
        {
            claw.setPosition(0.5);
        }

        if(gamepad2.x) {
            claw.setPosition(0.6);
        }

        float left  = -gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float lift  = gamepad2.left_stick_y;
        float hook  = gamepad2.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left  = Range.clip(left,  -1, 1);
        lift  = Range.clip(lift,  -1, 1);
        hook  = Range.clip(hook,  -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left  = (float)scaleInput(left);
        lift  = (float)scaleInput(lift);
        hook  = (float)scaleInput(hook);

        // write the values to the motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
        motorLift.setPower(lift);
        motorHook.setPower(hook);

        telemetry.addData("Motor left", left);
        telemetry.addData("Motor right", right);
        telemetry.addData("Motor lift", lift);
        telemetry.addData("Motor hook", hook);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
