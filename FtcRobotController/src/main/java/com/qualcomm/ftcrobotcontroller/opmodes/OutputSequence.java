package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

import org.apache.http.impl.conn.ProxySelectorRoutePlanner;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

/**
 * Created by luke on 2/26/16.
 */
public class OutputSequence {

    public ErrorManager error;
    public ObjectOutputStream stream;

    // Load default_servo.txt read the file.
    public OutputSequence(int sequence_num, Context ctx)
    {
        error = new ErrorManager("output sequence manager");

        try {
            String num = (new Integer(sequence_num)).toString();
            FileOutputStream file = ctx.openFileOutput("sequence_" + num + ".txt", Context.MODE_PRIVATE);
            stream = new ObjectOutputStream(file);
        } catch (FileNotFoundException e) {
            error.setError("Output file not found exception");
        } catch(IOException e) {
            error.setError("IOException opening output stream");
        }

    }

    public ObjectOutputStream stream()
    {
        return stream;
    }
}
