package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous
public class ReadColor extends OpMode {

    ColorSensor frontColor;
    ColorSensor bottomColor;

    @Override
    public void init() {
        frontColor = hardwareMap.colorSensor.get("front");
        bottomColor = hardwareMap.colorSensor.get("bottom");
    }

    @Override
    public void loop() {
        telemetry.addData("front red", frontColor.red());
        telemetry.addData("front green", frontColor.green());
        telemetry.addData("front blue", frontColor.blue());
        telemetry.addData("front argb", frontColor.argb());
        telemetry.addData("front string", frontColor.toString());
        telemetry.addData("front alpha", frontColor.alpha());

        telemetry.addData("bottom red", bottomColor.red());
        telemetry.addData("bottom green", bottomColor.green());
        telemetry.addData("bottom blue", bottomColor.blue());
        telemetry.addData("bottom argb", bottomColor.argb());
        telemetry.addData("bottom string", bottomColor.toString());
        telemetry.addData("bottom alpha", bottomColor.alpha());
    }
}
