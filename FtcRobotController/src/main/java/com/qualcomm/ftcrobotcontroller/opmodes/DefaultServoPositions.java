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
public class DefaultServoPositions {

    public ErrorManager error;

    public Context ctx;

    // Load default_servo.txt read the file.
    public DefaultServoPositions(Context ctx_in)
    {
        error = new ErrorManager("default servo position");

        ctx = ctx_in;
    }

    private ObjectOutputStream openOutput()
    {
        try {
            FileOutputStream file = ctx.openFileOutput("default_servo.txt", Context.MODE_PRIVATE);
            ObjectOutputStream stream = new ObjectOutputStream(file);
            return stream;
        } catch (FileNotFoundException e) {
            error.setError("Output file not found exception");
        } catch(IOException e) {
            error.setError("IOException opening output stream");
        }

        return null;
    }

    private ObjectInputStream openInput()
    {
        try {
            FileInputStream file = ctx.openFileInput("default_servo.txt");
            ObjectInputStream stream = new ObjectInputStream(file);
            return stream;
        } catch (FileNotFoundException e) {
            error.setError("Input file not found exception");
        } catch(IOException e) {
            error.setError("IOException opening input stream");
        }

        return null;
    }

    // Returns whether the thing was actually written, we want this to be true.
    public boolean write(ServoValues values)
    {
        ObjectOutputStream stream = openOutput();

        if(stream != null)
        {
            try {
                // Clip then write
                values.clipValues();

                stream.writeDouble(values.climberHighPos);
                stream.writeDouble(values.climberLowPos);
                stream.writeDouble(values.leftSidePos);
                stream.writeDouble(values.rightSidePos);

                stream.close();

                return true;
            } catch(IOException e) {
                error.setError("IOException writing values to file");
                return false;
            }
        }
        return false;
    }

    public ServoValues read()
    {
        ObjectInputStream stream = openInput();

        ServoValues values = new ServoValues();

        if(stream != null)
        {
            try {
                values.climberHighPos = stream.readDouble();
                values.climberLowPos = stream.readDouble();
                values.leftSidePos = stream.readDouble();
                values.rightSidePos = stream.readDouble();

                stream.close();

                return values;
            } catch(IOException e) {
                error.setError("IOException writing values to file");
                return null;
            }
        }
        return null;
    }
}
