package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

// Automatic white balance in software ("gray world"). On average a scene is gray (an FTC field is gray tiles),
// so if the whole picture averages out yellow, the light is yellow: boost blue and lower red/green until
// red, green and blue average out equal.
// Add it to the VisionPortal BEFORE the color locators. Processors run in order on the same picture,
// so the locators (and the DS Camera Stream) see the corrected colors.
//
// Options (the defaults behave like the first version, used by Ball Detect TeleOp):
//   setIgnoreColorfulPixels(true): only measure the light on grayish pixels (floor, walls, metal). Strongly colored
//       things, like yellow balls, are left out of the measurement, so they can't make the correction go too far.
//   setStrength(0..1): how much of the correction is applied. 1 = all of it, 0.5 = half way.
//   setMaxGain(x): the most a color may be boosted (and 1/x the most it may be lowered).
public class WhiteBalanceProcessor implements VisionProcessor {

    static final double MIN_GAIN = 0.5;
    static final double SMOOTHING = 0.2;   // 0..1, how fast the correction follows the light (lower = steadier)
    // A pixel counts as "grayish" if its colors differ by less than this share of its brightest color.
    // In yellow light a gray floor still looks a bit yellow (about 0.3-0.4); a yellow ball is about 0.8.
    static final double GRAYISH_MAX_SATURATION = 0.5;
    static final double DARK_LIMIT = 30;   // pixels darker than this (0..255) are left out, they are mostly noise
    static final double MIN_GRAYISH_SHARE = 0.05;   // fewer grayish pixels than this: keep the old correction

    // volatile: the camera thread and the OpMode thread both use these.
    private volatile boolean enabled = true;
    private volatile boolean ignoreColorfulPixels = false;
    private volatile double strength = 1.0;
    private volatile double maxGain = 3.0;
    private volatile double redGain = 1;
    private volatile double greenGain = 1;
    private volatile double blueGain = 1;
    private volatile double measuredShare = 1;   // part of the picture the light was measured on (0..1)

    // Reused every frame (making new ones each time would be slow).
    private final List<Mat> channels = new ArrayList<>();
    private final Mat maxRgb = new Mat();
    private final Mat minRgb = new Mat();
    private final Mat limit = new Mat();
    private final Mat grayish = new Mat();
    private final Mat bright = new Mat();

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setIgnoreColorfulPixels(boolean ignore) {
        this.ignoreColorfulPixels = ignore;
    }

    public void setStrength(double strength) {
        this.strength = Math.max(0, Math.min(1, strength));
    }

    public void setMaxGain(double maxGain) {
        this.maxGain = Math.max(1, maxGain);
    }

    public double getRedGain() {
        return redGain;
    }

    public double getGreenGain() {
        return greenGain;
    }

    public double getBlueGain() {
        return blueGain;
    }

    // How much of the picture (0..1) the light was measured on. Always 1 unless setIgnoreColorfulPixels(true).
    public double getMeasuredShare() {
        return measuredShare;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (!enabled) {
            return null;
        }
        Scalar mean;
        if (ignoreColorfulPixels) {
            mean = meanOfGrayishPixels(frame);
            if (mean == null) {
                applyGains(frame);   // almost nothing grayish in view (e.g. a ball right at the lens): keep the old correction
                return null;
            }
        } else {
            mean = Core.mean(frame);   // average red, green, blue of the whole picture
            measuredShare = 1;
        }
        double gray = (mean.val[0] + mean.val[1] + mean.val[2]) / 3;
        redGain = smooth(redGain, gray / Math.max(1, mean.val[0]));
        greenGain = smooth(greenGain, gray / Math.max(1, mean.val[1]));
        blueGain = smooth(blueGain, gray / Math.max(1, mean.val[2]));
        applyGains(frame);
        return null;
    }

    // Changes the picture in place, so the processors after this one see the corrected colors.
    // The 4th value (1) leaves the alpha channel alone.
    private void applyGains(Mat frame) {
        Core.multiply(frame, new Scalar(redGain, greenGain, blueGain, 1), frame);
    }

    // Average color of the grayish, not-too-dark pixels only, or null if there are too few of them.
    private Scalar meanOfGrayishPixels(Mat frame) {
        for (Mat m : channels) {
            m.release();
        }
        channels.clear();
        Core.split(frame, channels);
        Mat r = channels.get(0);
        Mat g = channels.get(1);
        Mat b = channels.get(2);
        Core.max(r, g, maxRgb);
        Core.max(maxRgb, b, maxRgb);
        Core.min(r, g, minRgb);
        Core.min(minRgb, b, minRgb);
        // grayish: (max - min) < GRAYISH_MAX_SATURATION * max
        Core.subtract(maxRgb, minRgb, minRgb);                                   // minRgb now holds max - min
        Core.multiply(maxRgb, new Scalar(GRAYISH_MAX_SATURATION), limit);
        Core.compare(minRgb, limit, grayish, Core.CMP_LT);
        Core.compare(maxRgb, new Scalar(DARK_LIMIT), bright, Core.CMP_GT);
        Core.bitwise_and(grayish, bright, grayish);

        int count = Core.countNonZero(grayish);
        measuredShare = count / (double) Math.max(1, frame.rows() * frame.cols());
        if (measuredShare < MIN_GRAYISH_SHARE) {
            return null;
        }
        return Core.mean(frame, grayish);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    private double smooth(double oldGain, double newGain) {
        newGain = 1 + (newGain - 1) * strength;   // only part of the correction if strength < 1
        newGain = Math.max(Math.max(MIN_GAIN, 1 / maxGain), Math.min(maxGain, newGain));
        return oldGain + (newGain - oldGain) * SMOOTHING;
    }
}
