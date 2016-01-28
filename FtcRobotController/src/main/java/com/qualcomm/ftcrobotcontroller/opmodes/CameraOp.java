package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.CameraPreview;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.io.ByteArrayOutputStream;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;

enum Color
{
    R, G, B
};
class ColorComp
{
    final int value;
    final Color component;
    public ColorComp(final int val, final Color comp)
    {
        value = val;
        component = comp;
    }
};

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class CameraOp extends OpMode {
    private Camera camera;
    public CameraPreview preview;
    public Bitmap image;
    private int width;
    private int height;
    private YuvImage yuvImage = null;
    private int looped = 0;
    private String data;

    private int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    private int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    private int blue(int pixel) {
        return pixel & 0xff;
    }

    private Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            Camera.Parameters parameters = camera.getParameters();
            width = parameters.getPreviewSize().width;
            height = parameters.getPreviewSize().height;
            yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
            looped += 1;
        }
    };

    private void convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        // Remember to do this: Can we even do this here?
        ((FtcRobotControllerActivity)hardwareMap.appContext).openBackCamera();

        camera = ((FtcRobotControllerActivity)hardwareMap.appContext).camera;
        camera.setPreviewCallback(previewCallback);

        Camera.Parameters parameters = camera.getParameters();
        data = parameters.flatten();

        ((FtcRobotControllerActivity) hardwareMap.appContext).initPreview(camera, this, previewCallback);
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    public ColorComp highestColor(int red, int green, int blue) {
        int[] color = {red,green,blue};
        int color_i = 0;
        for (int i = 1; i < 3; i++) {
            if (color[color_i] < color[i]) {
                color_i = i;
            }
        }
        Color c;
        if(color_i == 0) c = Color.R;
        else if(color_i == 1) c = Color.G;
        else c = Color.B;
        return new ColorComp(color[color_i], c);
    }

    @Override
    public void loop() {

        // Idea:
        // - Gaussian Blur
        // - Check three segments
        // - Rotate until ramp color is center
        // - Move forward
        // - Do less blur, more segments?
        if (yuvImage != null) {
            int segments = 9;
            ColorComp[] colors = new ColorComp[segments];
            for(int i = 0; i < segments; ++i) {
                int redValue = 0;
                int blueValue = 0;
                int greenValue = 0;
                convertImage();
                // Does this make sense?
                // We are separating our preview into 9 segments, then we are taking the most color
                // in each.
                int start = (width / segments) * segments;
                int end = Math.min(width, start + width / segments);
                for (int x = start; x < end; x++) {
                    for (int y = 0; y < height; y++) {
                        int pixel = image.getPixel(x, y);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                        greenValue += green(pixel);
                    }
                }

                colors[i] = highestColor(redValue, greenValue, blueValue);
            }

            // We have the colors of each segment.
            // Figure out which color we actually want.
        }
    }
}