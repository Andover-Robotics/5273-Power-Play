package org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.cone;

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

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HLS;

public class ConeDetection {

    private final OpenCvCamera camera;
    private final ConeDetectionPipeline pipeline = new ConeDetectionPipeline();
    private volatile Pair<Integer, Integer> result = null;
    private volatile boolean saveImageNext = true;
    private Telemetry telemetry;

    public ConeDetection(OpMode opMode, Telemetry telemetry) {

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
    static final double MAXIMUM_CONE_AREA = 10000;

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

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    class ConeDetectionPipeline extends OpenCvPipeline {
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

//            Imgproc.cvtColor(input, test, COLOR_RGB2HLS);
//            Core.inRange(test, lowerRange, upperRange, edgeDetector);
//            Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);
//
//            ArrayList<MatOfPoint> contours = new ArrayList<>();
//
//            Imgproc.findContours(smoothEdges, contours, contourDetector, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            extractRectBounds(contours);
//
//            for (Rect t: bounds) {
//                x += (t.x + (t.width / 2)) / bounds.size();
//                y += (t.y + (t.height / 2)) / bounds.size();
//            }
//
//            result = new Pair(x, y);

            //OUTPUTS
            Mat rgbThresholdOutput = new Mat();
            Mat cvErodeOutput = new Mat();
            ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
            ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

            //RGB Threshold: Segmenting the image based on color ranges
            Mat rgbThresholdInput = input;
            double[] rgbThresholdRed = {153.64208633093526, 255.0};
            double[] rgbThresholdGreen = {0.0, 71.0016977928693};
            double[] rgbThresholdBlue = {0.0, 255.0};
            rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);

            //CV Erode: Expands areas of lower values in an image
            Mat cvErodeSrc = rgbThresholdOutput;
            Mat cvErodeKernel = new Mat();
            Point cvErodeAnchor = new Point(-1, -1);
            double cvErodeIterations = 4.0;
            int cvErodeBordertype = Core.BORDER_CONSTANT;
            Scalar cvErodeBordervalue = new Scalar(-1);
            cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

            //Find Contours Detects contours in a binary image
            Mat findContoursInput = cvErodeOutput;
            boolean findContoursExternalOnly = false;
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

            //Filter Contours
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 0;
            double filterContoursMinPerimeter = 0;
            double filterContoursMinWidth = 0;
            double filterContoursMaxWidth = 5000.0;
            double filterContoursMinHeight = 0;
            double filterContoursMaxHeight = 1000;
            double[] filterContoursSolidity = {71.94244604316548, 100.0};
            double filterContoursMaxVertices = 1000000;
            double filterContoursMinVertices = 0;
            double filterContoursMinRatio = 0;
            double filterContoursMaxRatio = 1000;
            filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

            extractRectBounds(filterContoursOutput);

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
                if (r.area() > MAXIMUM_CONE_AREA)
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

        //Methods for CV
        private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
            Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
                    new Scalar(red[1], green[1], blue[1]), out);
        }

        private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                             int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1, -1);
            }
            if (borderValue == null) {
                borderValue = new Scalar(-1);
            }
            Imgproc.erode(src, dst, kernel, anchor, (int) iterations, borderType, borderValue);
        }

        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            } else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }

        private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                    double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                            maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                            minRatio, double maxRatio, List<MatOfPoint> output) {
            final MatOfInt hull = new MatOfInt();
            output.clear();
            //operation
            for (int i = 0; i < inputContours.size(); i++) {
                final MatOfPoint contour = inputContours.get(i);
                final Rect bb = Imgproc.boundingRect(contour);
                if (bb.width < minWidth || bb.width > maxWidth) continue;
                if (bb.height < minHeight || bb.height > maxHeight) continue;
                final double area = Imgproc.contourArea(contour);
                if (area < minArea) continue;
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int) hull.get(j, 0)[0];
                    double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100 * area / Imgproc.contourArea(mopHull);
                if (solid < solidity[0] || solid > solidity[1]) continue;
                if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount) continue;
                final double ratio = bb.width / (double) bb.height;
                if (ratio < minRatio || ratio > maxRatio) continue;
                output.add(contour);
            }
        }
    }
}
