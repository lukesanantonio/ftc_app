package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.CameraPreview;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.hardware.Camera;
import android.view.SurfaceView;

/**
 * Created by localt on 10/3/2015.
 */
public class JuniorAuto extends JuniorK9TeleOp {

    public CameraPreview preview;

    public enum Mode {
        On_Red_Team, On_Blue_Team
    };
    Mode cur_mode;

    public JuniorAuto(Mode mode)
    {
        cur_mode = mode;
    }

    @Override
    public void init()
    {
        // Initialize camera somehow
    }
    @Override
    public void stop()
    {
        // Don't forget to release camera
    }

    @Override
    public void loop()
    {
        if(time < 2.0)
        {
            super.motorLeftTread.setPower(-1.0);
            super.motorRightTread.setPower(-1.0);
            super.motorLeftWheel.setPower(-1.0);
            super.motorRightWheel.setPower(-1.0);
        }
        else if(time < 2.5) {
            if (cur_mode == Mode.On_Blue_Team) {
                super.motorLeftTread.setPower(1.0);
                super.motorRightTread.setPower(-1.0);
                super.motorLeftWheel.setPower(1.0);
                super.motorRightWheel.setPower(-1.0);
            }
            else
            {
                super.motorLeftTread.setPower(-1.0);
                super.motorRightTread.setPower(1.0);
                super.motorLeftWheel.setPower(-1.0);
                super.motorRightWheel.setPower(1.0);
            }
        }
        else if(time < 3.0)
        {
            super.motorLeftTread.setPower(1.0);
            super.motorRightTread.setPower(1.0);
            super.motorLeftWheel.setPower(1.0);
            super.motorRightWheel.setPower(1.0);
            //super.armMotor.setPower(1.0);
        }
        else
        {
            super.motorLeftTread.setPower(0.0);
            super.motorRightTread.setPower(0.0);
            super.motorLeftWheel.setPower(0.0);
            super.motorRightWheel.setPower(0.0);
        }
    }
}
