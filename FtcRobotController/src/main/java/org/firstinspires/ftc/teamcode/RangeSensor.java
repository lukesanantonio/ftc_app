package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by luke on 1/14/17.
 */

public class RangeSensor {
    private I2cDevice range;
    private I2cDeviceSynch rangeSynch;

    private byte[] rCache;

    private I2cAddr rangeAddr = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE_READ_START = 0x04; //Register to start reading
    public static final int RANGE_READ_LENGTH = 2; //Number of byte to read

    public RangeSensor(HardwareMap map, String name)
    {
        range = map.i2cDevice.get(name);
        rangeSynch = new I2cDeviceSynchImpl(range, rangeAddr, false);
        rangeSynch.engage();
    }

    public void updateCache()
    {
        rCache = rangeSynch.read(RANGE_READ_START, RANGE_READ_LENGTH);
    }

    public int ultraSonic()
    {
        return rCache[0] & 0xff;
    }
    public int optical()
    {
        return rCache[1] & 0xff;
    }

}
