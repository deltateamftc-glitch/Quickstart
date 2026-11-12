package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

// Color probe: measures the average color inside a small box in the middle of the camera picture
// and draws that box (white square) on the DS Camera Stream.
// How to use: put a ball so it fills the white box, then read Y / Cr / Cb on the telemetry.
// Those are the numbers a ColorRange (YCrCb) needs. Add it to the VisionPortal AFTER the white balance,
// so it measures the same colors the color locator sees.
public class ColorSampler implements VisionProcessor {

    static final int BOX_SIZE = 16;   // pixels (in the 320x240 picture)

    private int width;
    private int height;

    // volatile: the camera thread and the OpMode thread both use these.
    private volatile double y = 0;
    private volatile double cr = 0;
    private volatile double cb = 0;
    private volatile double red = 0;
    private volatile double green = 0;
    private volatile double blue = 0;

    public double getY() { return y; }
    public double getCr() { return cr; }
    public double getCb() { return cb; }
    public double getRed() { return red; }
    public double getGreen() { return green; }
    public double getBlue() { return blue; }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Rect box = new Rect((frame.cols() - BOX_SIZE) / 2, (frame.rows() - BOX_SIZE) / 2, BOX_SIZE, BOX_SIZE);
        Mat sub = frame.submat(box);
        Scalar mean = Core.mean(sub);   // the picture is RGB(A)
        sub.release();
        double r = mean.val[0];
        double g = mean.val[1];
        double b = mean.val[2];
        // Same formula OpenCV uses for RGB -> YCrCb (8 bit), so the numbers match the color locator.
        double luma = 0.299 * r + 0.587 * g + 0.114 * b;
        red = r;
        green = g;
        blue = b;
        y = luma;
        cr = (r - luma) * 0.713 + 128;
        cb = (b - luma) * 0.564 + 128;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(android.graphics.Color.WHITE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3 * scaleCanvasDensity);
        float left = (width - BOX_SIZE) / 2f * scaleBmpPxToCanvasPx;
        float top = (height - BOX_SIZE) / 2f * scaleBmpPxToCanvasPx;
        float size = BOX_SIZE * scaleBmpPxToCanvasPx;
        canvas.drawRect(left, top, left + size, top + size, paint);
    }
}
