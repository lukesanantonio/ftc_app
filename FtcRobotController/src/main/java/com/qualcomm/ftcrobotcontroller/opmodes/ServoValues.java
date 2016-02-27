package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by luke on 2/26/16.
 */
public class ServoValues {
    public double climberHighPos = 1.0;
    public double climberLowPos = 0.3;

    public double leftSidePos = 0.16;
    public double rightSidePos = 0.67;

    public void clipValues()
    {
        this.climberHighPos = Range.clip(this.climberHighPos, 0.0, 1.0);
        this.climberLowPos = Range.clip(this.climberLowPos, 0.0, 1.0);
        this.leftSidePos = Range.clip(this.leftSidePos, 0.0, 1.0);
        this.rightSidePos = Range.clip(this.rightSidePos, 0.0, 1.0);
    }

    public void logValues(Telemetry telemetry)
    {
        telemetry.addData("climber high", this.climberHighPos);
        telemetry.addData("climber low", this.climberLowPos);
        telemetry.addData("left servo pos", this.leftSidePos);
        telemetry.addData("right servo pos", this.rightSidePos);
    }

}
