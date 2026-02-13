package org.firstinspires.ftc.teamcode.TeleOpRoadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOpRoadrunner")
public class TeleOpRoadrunner extends LinearOpMode {

    DcMotor motor1, motor2, motor3, motor4;

    // VISION
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    // TUNING
    static final double DESIRED_DISTANCE = 8.0;
    static final double KP_FORWARD = 0.05;
    static final double KP_STRAFE = 0.04;
    static final double KP_TURN = 0.01;
    VoltageSensor batteryVoltageSensor;
    static final double V_REF = 11;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== APRILTAG =====
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= APRILTAG ALIGN (A) =================
            if (gamepad1.a) {

                List<AprilTagDetection> detections = aprilTag.getDetections();

                if (!detections.isEmpty()) {
                    AprilTagDetection tag = detections.get(0);

                    // Folosim funcția corectată
                    Pose2d alignPower = alignToTag(tag);
                    drive.setWeightedDrivePower(alignPower);

                    telemetry.addLine("ALIGNING TO TAG");
                    telemetry.addData("ID", tag.id);
                    telemetry.addData("X", tag.ftcPose.x);
                    telemetry.addData("Z", tag.ftcPose.z);
                    telemetry.addData("Yaw", tag.ftcPose.yaw);
                    telemetry.addLine("TAG");
                } else {
                    drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    telemetry.addLine("NO TAG");
                }

            } else {
                // ================= NORMAL TELEOP =================
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            drive.update();

            // ================= TELEMETRY POZITIE =================
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("RR x", poseEstimate.getX());
            telemetry.addData("RR y", poseEstimate.getY());
            telemetry.addData("RR heading", poseEstimate.getHeading());

            // ================= MOTOARE AUX =================
            motor1.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? gamepad2.left_stick_y : 0);
            motor2.setPower(Math.abs(gamepad2.right_stick_y) > 0.1 ? gamepad2.right_stick_y : 0);

            if (gamepad2.right_bumper) {
                motor3.setPower(compensatedPower(-0.65));
                motor4.setPower(compensatedPower(0.65));
            } else if (gamepad2.left_bumper) {
                motor3.setPower(compensatedPower(-0.53));
                motor4.setPower(compensatedPower(0.53    ));
            } else {
                motor3.setPower(0);
                motor4.setPower(0);
            }

            telemetry.update();
        }

        visionPortal.close();
    }

    // ================= ALIGN FUNCTION (corectata) =================
    Pose2d alignToTag(AprilTagDetection tag) {

        double forwardError = tag.ftcPose.z - DESIRED_DISTANCE;
        double strafeError = tag.ftcPose.x;

        // corectare sens rotatie in functie de pozitia laterala
        double turnError;
        if (strafeError > 0) { // tag-ul este in dreapta
            turnError = -Math.abs(tag.ftcPose.yaw);
        } else { // tag-ul este in stanga
            turnError = Math.abs(tag.ftcPose.yaw);
        }

        double forward = clip(forwardError * KP_FORWARD, -0.4, 0.4);
        double strafe = clip(strafeError * KP_STRAFE, -0.4, 0.4);
        double turn = clip(turnError * KP_TURN, -0.3, 0.3);

        return new Pose2d(forward, strafe, turn);
    }

    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    double compensatedPower(double powerDorita) {
        double voltage = batteryVoltageSensor.getVoltage();
        double power = powerDorita * (V_REF / voltage);
        telemetry.addData("voltaj", voltage);
        return clip(power, -1.0, 1.0);
    }

}
