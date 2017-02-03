package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by luke on 1/31/17.
 */

public class VVHardware extends TankHardware {
    public DcMotor wheelLeft;
    public DcMotor wheelRight;

    public VVHardware(HardwareMap hardwareMap) {
        super(hardwareMap);

        wheelLeft = hardwareMap.dcMotor.get("prop left");
        wheelRight = hardwareMap.dcMotor.get("prop right");

        // We don't want gears on these wheels to grind.
        wheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setShooterPower(float power)
    {
        wheelLeft.setPower(power);
        wheelRight.setPower(power);
    }
}
