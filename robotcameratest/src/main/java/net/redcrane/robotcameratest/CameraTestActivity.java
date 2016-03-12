package net.redcrane.robotcameratest;

import android.annotation.SuppressLint;

import java.util.List;
import java.util.ArrayList;

import android.app.ActionBar;
import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.features2d.*;

import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;

import org.opencv.imgproc.Imgproc;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */
public class CameraTestActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2 {

    private CameraBridgeViewBase mOpenCvCameraView;

    static final String TAG = "CameraTestActivity";

    Mat hsvMat;
    Mat redMat;
    Mat blueMat;
    Mat old;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_camera_test);

        // OpenCV Init
        Log.i("CameraTestActivity", "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.HelloOpenCvView);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i("CameraTestActivity", "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };
    @Override
    protected void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, getApplicationContext(), mLoaderCallback);
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        hsvMat = new Mat();
        redMat = new Mat();
        blueMat = new Mat();
        old = new Mat();
    }

    public void onCameraViewStopped() {
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Imgproc.cvtColor(inputFrame.rgba(), hsvMat, Imgproc.COLOR_RGB2HSV);


        // Average Red color
        // H: 7 S: 55 V: 200
        // Blue
        // H: S: V:

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

        // This is the largest contour
        if(maxAreaIdx != -1 && boundingRect != null)
        {
            // Print the area
            Log.w(TAG, "area = " + maxArea);
            Imgproc.rectangle(redMat, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 1);
        }

        return redMat;
    }
}
