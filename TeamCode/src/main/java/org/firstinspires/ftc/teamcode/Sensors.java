package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class Sensors extends OpMode {

    ColorSensor frontColor;
    ColorSensor bottomColor;
    RangeSensor rangeSensor;
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
        bottomColor = hardwareMap.colorSensor.get("bottom");
        bottomColor.setI2cAddress(I2cAddr.create8bit(0x1c));
        bottomColor.enableLed(false);

        rangeSensor = new RangeSensor(hardwareMap, "range");

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
    }

    @Override
    public void start() {
        bottomColor.enableLed(true);
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

        telemetry.addData("bottom red", bottomColor.red());
        telemetry.addData("bottom green", bottomColor.green());
        telemetry.addData("bottom blue", bottomColor.blue());
        telemetry.addData("bottom argb", bottomColor.argb());
        telemetry.addData("bottom string", bottomColor.toString());
        telemetry.addData("bottom alpha", bottomColor.alpha());
        telemetry.addData("bottom i2c", bottomColor.getI2cAddress().toString());

        rangeSensor.updateCache();
        telemetry.addData("range optical", rangeSensor.optical());
        telemetry.addData("range ultrasonic", rangeSensor.ultraSonic());

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
        if(frontColor.blue() == 0)
        {
            red_blue_ratio = (float) frontColor.red();
        }
        if (red_blue_ratio > 2.5f) {
            return Color.Red;
        }
        float blue_red_ratio = frontColor.blue() / (float) frontColor.red();
        if(frontColor.red() == 0)
        {
            blue_red_ratio = (float) frontColor.blue();
        }
        if (blue_red_ratio >= 2.5f) {
            return Color.Blue;
        }

        telemetry.addData("red / blue ratio", red_blue_ratio);
        telemetry.addData("blue / red ratio", blue_red_ratio);
        return null;
    }
}
