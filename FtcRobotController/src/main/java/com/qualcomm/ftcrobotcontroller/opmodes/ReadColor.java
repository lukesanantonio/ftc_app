package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by luke on 2/26/16.
 */
public class ReadColor extends OpMode {

    ColorSensor color;

    @Override
    public void init()
    {
        color = hardwareMap.colorSensor.get("color");
    }

    @Override
    public void loop()
    {
        telemetry.addData("red", color.red());
        telemetry.addData("green", color.green());
        telemetry.addData("blue", color.blue());
        telemetry.addData("argb", color.argb());
        telemetry.addData("string", color.toString());
        telemetry.addData("alpha", color.alpha());
    }
}
