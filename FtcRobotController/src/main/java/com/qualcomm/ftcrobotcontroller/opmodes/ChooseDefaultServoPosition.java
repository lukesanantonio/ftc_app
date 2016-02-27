package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ChooseDefaultServoPosition extends OpMode {

    DefaultServoPositions servoPositions;
    ServoValues values;
    SophServos sophServos;

    @Override
    public void init()
    {
        FtcRobotControllerActivity app = (FtcRobotControllerActivity) hardwareMap.appContext;
        servoPositions = new DefaultServoPositions(app.context);

        sophServos = new SophServos();
        sophServos.initServos(hardwareMap);
    }

    @Override
    public void start()
    {
        values = servoPositions.read();
        if(values == null)
        {
            values = new ServoValues();

            values.climberLowPos = 0.0;
            values.climberHighPos = 0.0;
            values.leftSidePos = 0.0;
            values.rightSidePos = 0.0;
        }
    }
    @Override
    public void loop()
    {
        // Adjust them
        values.climberLowPos = values.climberLowPos + gamepad1.left_stick_y * .005;
        values.climberHighPos = values.climberHighPos + gamepad1.right_stick_y * .005;

        values.leftSidePos = values.leftSidePos + -gamepad2.left_stick_y * .005;
        values.rightSidePos = values.rightSidePos + gamepad2.right_stick_y * .005;

        // Set them
        sophServos.setValues(values);

        // Press a button on either controller to save the values as they are.
        if(gamepad1.a || gamepad2.a)
        {
            // Potentially write
            servoPositions.write(values);
        }

        // Log
        values.logValues(telemetry);
        servoPositions.error.logError(telemetry);
    }
}
