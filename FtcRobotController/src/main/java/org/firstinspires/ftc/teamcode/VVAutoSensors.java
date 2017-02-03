package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by luke on 1/31/17.
 */

public class VVAutoSensors {
    RangeFinder rangeFinder;
    ColorSensor frontColor;
    ColorSensor bottomColor;
    GyroSensor gyro;

    public VVAutoSensors(HardwareMap hardwareMap)
    {
        rangeFinder = new RangeFinder(hardwareMap, "range");

        frontColor = hardwareMap.colorSensor.get("front");
        frontColor.setI2cAddress(I2cAddr.create8bit(0x2c));

        bottomColor = hardwareMap.colorSensor.get("bottom");
        bottomColor.setI2cAddress(I2cAddr.create8bit(0x1c));

        gyro = hardwareMap.gyroSensor.get("gyro");
    }
}
