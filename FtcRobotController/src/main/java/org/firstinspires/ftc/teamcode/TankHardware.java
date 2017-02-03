package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankHardware {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;

    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public TankHardware(HardwareMap hardwareMap)
    {
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backRightMotor = hardwareMap.dcMotor.get("back right");
    }

    public void setForward(float power)
    {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }
    public void setBackward(float power)
    {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
    }

    public void setTurnCW(float power)
    {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);

        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
    }

    public void setTurnCCW(float power)
    {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);

        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void setLeft(float power)
    {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(power);

        frontRightMotor.setPower(power);
        backRightMotor.setPower(-power);
    }
    public void setRight(float power)
    {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(-power);

        frontRightMotor.setPower(-power);
        backRightMotor.setPower(power);
    }

    public void setStop()
    {
        frontLeftMotor.setPower(0.0f);
        frontRightMotor.setPower(0.0f);
        backLeftMotor.setPower(0.0f);
        backRightMotor.setPower(0.0f);
    }
}
