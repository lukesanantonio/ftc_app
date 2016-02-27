package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by luke on 2/26/16.
 */
public class ErrorManager {

    String error;
    String errorType;

    public ErrorManager(String errT)
    {
        errorType = errT;
    }

    public void setError(String err)
    {
        error = err;
    }

    public void logError(Telemetry t)
    {
        if(error != null) t.addData("Error " + errorType, error);
    }
}
