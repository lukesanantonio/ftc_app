package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@Autonomous
public class Sensors extends OpMode {

    ColorSensor frontColor;
    RangeFinder rangeFinder;
    GyroSensor gyroSensor;

    enum Color
    {
        Blue, Red;
    };

    @Override
    public void init() {
        frontColor = hardwareMap.colorSensor.get("front");
        frontColor.setI2cAddress(I2cAddr.create8bit(0x2c));
        frontColor.enableLed(false);

        rangeFinder = new RangeFinder(hardwareMap, "range");

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
    }

    @Override
    public void start() {
        gyroSensor.calibrate();
    }

    @Override
    public void loop() {
        telemetry.addData("front red", frontColor.red());
        telemetry.addData("front green", frontColor.green());
        telemetry.addData("front blue", frontColor.blue());
        telemetry.addData("front argb", frontColor.argb());
        telemetry.addData("front string", frontColor.toString());
        telemetry.addData("front alpha", frontColor.alpha());
        telemetry.addData("front i2c", frontColor.getI2cAddress().toString());

        rangeFinder.updateCache();
        telemetry.addData("range optical", rangeFinder.optical());
        telemetry.addData("range ultrasonic", rangeFinder.ultraSonic());

        Color color = guessFrontColor();
        if(color != null) {
            telemetry.addData("front color", color.toString());
        }
        else
        {
            telemetry.addData("front color", "null");
        }

        telemetry.addData("calibrating", gyroSensor.isCalibrating());

        try {
            telemetry.addData("gyro rotationFraction", gyroSensor.getRotationFraction());
        }
        catch(UnsupportedOperationException e)
        {
            telemetry.addData("gyro rotationFraction", "unsupported");
        }

        try {
            telemetry.addData("gyro heading", gyroSensor.getHeading());
        }
        catch(UnsupportedOperationException e)
        {
            telemetry.addData("gyro heading", "unsupported");
        }

        try {
            telemetry.addData("gyro rawx", gyroSensor.rawX());
            telemetry.addData("gyro rawy", gyroSensor.rawY());
            telemetry.addData("gyro rawz", gyroSensor.rawZ());
        }
        catch(UnsupportedOperationException e)
        {
            telemetry.addData("gyro rawx", "unsupported");
            telemetry.addData("gyro rawy", "unsupported");
            telemetry.addData("gyro rawz", "unsupported");
        }
    }
    private Color guessFrontColor() {
        float red_blue_ratio = frontColor.red() / (float) frontColor.blue();
        if (frontColor.blue() == 0) {
            red_blue_ratio = frontColor.red();
        }
        if (red_blue_ratio >= 1.5f ||
                (frontColor.blue() == 0 && frontColor.red() >= 2)) {
            return Color.Red;
        }
        float blue_red_ratio = frontColor.blue() / (float) frontColor.red();
        if (frontColor.red() == 0) {
            blue_red_ratio = frontColor.blue();
        }
        if (blue_red_ratio >= 1.5f ||
                (frontColor.red() == 0 && frontColor.blue() >= 2)) {
            return Color.Blue;
        }

        telemetry.addData("red / blue ratio", red_blue_ratio);
        telemetry.addData("blue / red ratio", blue_red_ratio);
        return null;
    }
}
