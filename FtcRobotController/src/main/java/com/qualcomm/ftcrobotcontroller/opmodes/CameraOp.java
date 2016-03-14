package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.CameraPreview;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import android.util.Log;

import java.util.List;
import java.util.ArrayList;

import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;

class CameraOpListener implements CameraBridgeViewBase.CvCameraViewListener2 {
    Mat hsvMat;
    Mat redMat;
    Mat blueMat;
    Mat old;

    int width, height;

    public float leftWheelPower = 0.0f;
    public float rightWheelPower = 0.0f;
    public boolean considering = true;

    private static final float FORWARD_POWER = 0.3f;
    private static final float TURNING_POWER = 0.3f;

    private static final String TAG = "CameraOpListener";

    @Override
    public void onCameraViewStarted(int width, int height) {
        hsvMat = new Mat();
        redMat = new Mat();
        blueMat = new Mat();
        old = new Mat();

        this.width = width;
        this.height = height;
    }

    @Override
    public void onCameraViewStopped() {
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        // For now stop immediately once we are done *considering*
        if(!considering) return inputFrame.rgba();

        Imgproc.cvtColor(inputFrame.rgba(), hsvMat, Imgproc.COLOR_RGB2HSV);


        //double[] vals = hsvMat.get(0,0);
        //Log.i(TAG, "H: " + vals[0] + " S: " + vals[1] + " V: " + vals[2]);
        //return hsvMat;

        Core.inRange(hsvMat, new Scalar(0xa4, 0x14, 0xe0), new Scalar(0xc0, 0x50, 0xff), redMat);
        //Core.bitwise_not(redMat, redMat);

        // Do the red mat
        // First remove small things
        Imgproc.erode(redMat, redMat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(10, 10)));
        Imgproc.dilate(redMat, redMat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(10, 10)));

        // Blue now
        //Core.inRange(hsvMat, new Scalar(0x20, 0x35, 0x80), new Scalar(0x90, 0x70, 0xff), blueMat);
        //Core.bitwise_not(blueMat, blueMat);

        // Do the red mat
        // First remove small things
        //Imgproc.erode(blueMat, blueMat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(7, 7)));
        //Imgproc.dilate(blueMat, blueMat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(7, 7)));

        // Accumulate the two images
        //Imgproc.accumulate(redMat, blueMat);

        //Point shift = Imgproc.phaseCorrelate(redMat, old);
        //old = redMat;

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(redMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = -1;
        int maxAreaIdx = -1;
        Rect boundingRect = new Rect();
        for (int idx = 0; idx < contours.size(); idx++) {
            boundingRect = Imgproc.boundingRect(contours.get(idx));

            Mat contour = contours.get(idx);
            double contourarea = Imgproc.contourArea(contour);
            if (contourarea > maxArea) {
                maxArea = contourarea;
                maxAreaIdx = idx;
            }
        }

        // These values represent motor values that we don't have to worry about synchronization
        float targetLeft = 0.0f;
        float targetRight = 0.0f;
        boolean targetConsidering = true;

        // This is the largest contour
        if(maxAreaIdx != -1 && boundingRect != null)
        {
            // Find the center point
            Point p = new Point();
            p.x = boundingRect.tl().x + (boundingRect.br().x - boundingRect.tl().x / 2);
            p.y = boundingRect.tl().y + (boundingRect.br().y - boundingRect.tl().y / 2);

            // We got the center point.
            // Find out where it is in the range [-1,1] where 0 is right in the center.

            Point p;

            p.x /= (width / 2.0) - 1.0;
            p.y /= (height / 2.0) - 1.0;

            // Now that we know, use the x value to orient the robot.
            if(-0.2 <= p.x && p.x <= 0.2) {
                targetLeft = FORWARD_POWER;
                targetRight = FORWARD_POWER;
            }
            else if(p.x < -0.1) {
                targetLeft = -TURNING_POWER;
                targetRight = +TURNING_POWER;
            }
            else if(0.1 < p.x)
            {
                targetLeft = +TURNING_POWER;
                targetRight = -TURNING_POWER;
            }

            // We consider the thing up to a maximum size, this probably won't work until we get
            // better at detecting the surface
            // Possibly use the distance sensor
            if(maxArea > 10000.0f)
            {
                targetConsidering = false;
            }

            // Print the area
            Log.w(TAG, "area = " + maxArea);
            Imgproc.rectangle(redMat, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 1);
        }
        else
        {
            // The contour disappeared
            // Keep moving forward
            targetLeft = FORWARD_POWER;
            targetRight = FORWARD_POWER;
        }

        synchronized (this) {
            considering = targetConsidering;
            leftWheelPower = targetLeft;
            rightWheelPower = targetRight;
        }

        return redMat;
    }
}

public class CameraOp extends OpMode {

    CameraOpListener listener;

    DcMotor treadLeft;
    DcMotor treadRight;

    DcMotor armAngle;
    DcMotor armExtend;

    ServoValues servoValues;
    SophServos sophServos;

    @Override
    public void init() {
        listener = new CameraOpListener();

        ((FtcRobotControllerActivity)hardwareMap.appContext).setCameraListener(listener);

        // Initialize everything
        // TODO: Abstract this!
        treadRight = hardwareMap.dcMotor.get("right");
        treadLeft = hardwareMap.dcMotor.get("left");
        treadLeft.setDirection(DcMotor.Direction.REVERSE);
        treadRight.setDirection(DcMotor.Direction.FORWARD);

        armAngle = hardwareMap.dcMotor.get("arm angle");
        armAngle.setDirection(DcMotor.Direction.REVERSE);
        armExtend = hardwareMap.dcMotor.get("arm extend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        armAngle.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        armExtend.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        sophServos = new SophServos();
        sophServos.initServos(hardwareMap);
    }

    @Override
    public void loop() {
        // Get the wheels

        // Let's cheat and not worry about the value of considering!
        // TODO: When you get a race condition rethink this
        if(listener.considering) {
            synchronized (listener) {
                treadLeft.setPower(listener.leftWheelPower);
                treadRight.setPower(listener.rightWheelPower);
            }
        }
        else {
            // Start the other character sequence

            treadLeft.setPower(0.0f);
            treadRight.setPower(0.0f);
        }

        telemetry.addData("considering", listener.considering);
        telemetry.addData("left", treadLeft.getPower());
        telemetry.addData("right", treadRight.getPower());
    }
}