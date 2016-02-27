package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public class Sequence {

    boolean recording;
    boolean running;

    Context context;

    ObjectOutputStream outStream;
    ObjectInputStream inStream;

    String filename;

    ErrorManager error;

    public Sequence(String name, Context context)
    {
        this.context = context;
        filename = "sequence_" + name + ".txt";
        error = new ErrorManager("Sequence " + name);
    }

    public boolean recording() { return outStream != null; }
    public boolean playing() { return inStream != null; }

    public void stopRecording() {
        if(recording())
        {
            // We are currently recording / writing
            try { outStream.close(); } catch (IOException e) {}

            // Now that we closed the write stream we're done, get ready to replay.
            outStream = null;
        }
    }

    public void startPlaying()
    {
        stopRecording();
    }
    public double readDouble() {
        try {
            if (inStream != null) return inStream.readDouble();
            else return 0.0;
        } catch(IOException e) {
            return 0.0;
        }
    }
    public int readInt()
    {
        return 0;
    }
}
