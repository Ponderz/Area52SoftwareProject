package org.firstinspires.ftc.teamcode.Auto;

import java.util.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public VisionPipeline(Telemetry t) {telemetry = t;}
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(10, 100, 100);
        Scalar highHSV = new Scalar(30, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, kernel);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        MatOfPoint largestContour = null;
        int idx = -1;
        int i = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
                idx = i;
            }
            i++;
        }
        if (largestContour != null) {
            Imgproc.drawContours(input, contours, idx, new Scalar(255, 0, 0), 2);
            Moments moments = Imgproc.moments(largestContour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();
            double width = Imgproc.boundingRect(largestContour).width;
            String widthLabel = "Width: " + width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX - 50, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 1);
            double distance = (88.9 * 4) / width; //width of sample in mm * logitech c270 focal length in mm / pixel width
            String distanceLabel = "Distance: " + String.format("%.2f", distance) + " mm";
            Imgproc.putText(input, distanceLabel, new Point(cX -50, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 1);

            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.4, new Scalar(0, 255, 0), 1);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        }
        return input;
    }
}
