package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TankHardware {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;

    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    private Telemetry logger;

    public TankHardware(HardwareMap hardwareMap)
    {
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void logPower()
    {
        logger.addData("front left power", frontLeftMotor.getPower());
        logger.addData("front right power", frontRightMotor.getPower());
        logger.addData("back right power", backRightMotor.getPower());
        logger.addData("back left power", backLeftMotor.getPower());
    }

    public void setTelemetry(Telemetry telem)
    {
        logger = telem;
    }

    public void setForward(float power)
    {
        setForward(power, power);
    }
    public void setForward(float left, float right)
    {
        frontLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backLeftMotor.setPower(left);
        backRightMotor.setPower(right);
    }
    public void setBackward(float power)
    {
        setBackward(power, power);
    }
    public void setBackward(float left, float right)
    {
        frontLeftMotor.setPower(-left);
        frontRightMotor.setPower(-right);
        backLeftMotor.setPower(-left);
        backRightMotor.setPower(-right);
    }

    public void setTurnCW(float power)
    {
        setTurnCW(power, true, true);
    }
    public void setTurnCW(float power, boolean front, boolean back)
    {
        if(front) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
        } else {
            frontLeftMotor.setPower(0.0f);
            frontRightMotor.setPower(0.0f);
        }
        if (back) {
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
        } else {
            backLeftMotor.setPower(0.0f);
            backRightMotor.setPower(0.0f);
        }
    }

    public void setTurnCCW(float power)
    {
        setTurnCCW(power, true, true);
    }
    public void setTurnCCW(float power, boolean front, boolean back)
    {
        if(front) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
        } else {
            frontLeftMotor.setPower(0.0f);
            frontRightMotor.setPower(0.0f);
        }
        if (back) {
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
        } else {
            backLeftMotor.setPower(0.0f);
            backRightMotor.setPower(0.0f);
        }
    }

    public void setLeft(float power)
    {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(-power * 1.0f);

        frontRightMotor.setPower(-power);
        backRightMotor.setPower(power * 1.0f);
    }
    public void setRight(float power)
    {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(power * 1.0f);

        frontRightMotor.setPower(power);
        backRightMotor.setPower(-power * 1.0f);
    }

    public void stop()
    {
        frontLeftMotor.setPower(0.0f);
        frontRightMotor.setPower(0.0f);
        backLeftMotor.setPower(0.0f);
        backRightMotor.setPower(0.0f);
    }
}
