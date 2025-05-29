package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous()
public class AprilTags extends LinearOpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
        while (opModeInInit()) {
            List<AprilTagDetection> curDetections = aprilTagProcessor.getDetections();
            StringBuilder idsFound = new StringBuilder();
            for (AprilTagDetection detection : curDetections) {
                idsFound.append(detection.id);
                idsFound.append(" ");
            }
            telemetry.addData("April Tags", idsFound);
            telemetry.update();
            idle();
        }
        waitForStart();
        visionPortal.stopStreaming();
        while (opModeIsActive()) {
            sleep(20);
        }
    }


}
