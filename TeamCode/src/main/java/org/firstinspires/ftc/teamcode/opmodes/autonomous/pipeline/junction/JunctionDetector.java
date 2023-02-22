package org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.junction;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HLS;

public class JunctionDetector {

    private final OpenCvCamera camera;
    private final JunctionDetectionPipeline pipeline = new JunctionDetectionPipeline();
    private volatile Pair<Integer, Integer> result = null;
    private volatile boolean saveImageNext = true;
    private Telemetry telemetry;

    public JunctionDetector(OpMode opMode, Telemetry telemetry) {

        this.telemetry = telemetry;
        WebcamName camName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Status", "Opened");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error Code", errorCode);
            }
        });
    }

    public void saveImage() {
        saveImageNext = true;
    }

    public Optional<Pair<Integer, Integer>> currentlyDetected() {
        return Optional.ofNullable(result);
    }

    public void stop() {
        camera.stopStreaming();
    }

    public void close() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    class JunctionDetectionPipeline extends OpenCvPipeline {

        //TODO: Tune HSV Values
        private final Scalar lowerRange = new Scalar(0, 200, 60);
        private final Scalar upperRange = new Scalar(180, 255, 255);

    //JUNCTION DETECTION CONSTANTS
        static final double MAXIMUM_JUNCTION_AREA = 21500;

        private final Mat test = new Mat(),
                edgeDetector = new Mat(),
                smoothEdges = new Mat(),
                contourDetector = new Mat();

        private final MatOfPoint2f polyDpResult = new MatOfPoint2f();
        private final List<Rect> bounds = new ArrayList<>();
        private final Size gaussianKernelSize = new Size(9, 9);

        private int x;
        private int y;

        private ArrayList rgb = new ArrayList<Mat>();

        @SuppressLint("SdCardPath")
        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, test, COLOR_RGB2HLS);
            Core.inRange(test, lowerRange, upperRange, edgeDetector);
            Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);

            ArrayList<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(smoothEdges, contours, contourDetector, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            extractRectBounds(contours);

            for (Rect t: bounds) {
                x += (t.x + (t.width / 2)) / bounds.size();
                y += (t.y + (t.height / 2)) / bounds.size();
            }

            result = new Pair(x, y);

            if (saveImageNext) {
                Mat cvt = new Mat();
                Log.i("RingStackDetector", "saving current pipeline image");
                for (Rect r : bounds) {
                    Log.i("RingStackDetector", String.format("result x=%d y=%d width=%d height=%d area=%.2f", r.x, r.y, r.width, r.height, r.area()));
                }
                Imgcodecs.imwrite("/sdcard/FIRST/pipe-img.png", cvt);
                Imgcodecs.imwrite("/sdcard/FIRST/pipe-img-smoothEdges.png", smoothEdges);
                saveImageNext = false;
                cvt.release();
            }

            return input;
        }

        private void extractRectBounds(ArrayList<MatOfPoint> contours) {
            bounds.clear();
            for (MatOfPoint contour : contours) {
                // if polydp fails, switch to a local new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), polyDpResult, 3, true);
                Rect r = Imgproc.boundingRect(new MatOfPoint(polyDpResult.toArray()));
                if (r.area() > MAXIMUM_JUNCTION_AREA)
                    addCombineRectangle(bounds, r, bounds.size() - 1);
            }
        }

        private boolean overlaps(Rect a, Rect b) {
            return a.tl().inside(b) || a.br().inside(b) || b.tl().inside(a) || b.br().inside(a);
        }

        private Rect combineRect(Rect a, Rect b) {
            int topY = (int) Math.min(a.tl().y, b.tl().y);
            int leftX = (int) Math.min(a.tl().x, b.tl().x);
            int bottomY = (int) Math.max(a.br().y, b.br().y);
            int rightX = (int) Math.max(a.br().x, b.br().x);
            return new Rect(leftX, topY, rightX - leftX, bottomY - topY);
        }

        private void addCombineRectangle(List<Rect> list, Rect newRect, int ptr) {
            for (int i = ptr; i >= 0; i--) {
                Rect existing = list.get(i);
                if (overlaps(newRect, existing)) {
                    list.remove(i);
                    addCombineRectangle(list, combineRect(existing, newRect), i - 1);
                    return;
                }
            }
            list.add(newRect);
        }

    }
}
