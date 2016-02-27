package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by luke on 2/26/16.
 */
public class ReadDistance extends OpMode {

    OpticalDistanceSensor distance;

    @Override
    public void init()
    {
        distance = hardwareMap.opticalDistanceSensor.get("distance");
    }

    @Override
    public void loop()
    {
        telemetry.addData("distance", distance.getLightDetected());
        telemetry.addData("distanceRaw", distance.getLightDetectedRaw());
    }
}
