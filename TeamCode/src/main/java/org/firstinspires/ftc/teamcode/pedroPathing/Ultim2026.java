package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Ultim2026")
public class Ultim2026 extends OpMode {

    DcMotorEx motor1, motor2, motor3, motor4;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    boolean autoAlign = false;

    // ===== TUNING =====
    double TURN_GAIN = 0.02;
    double STRAFE_GAIN = 0.025;
    double FORWARD_GAIN = 0.03;
    double TARGET_DISTANCE = 9.5; // inches from tag

    int TARGET_ID = -1;
    public double dif;// -1 = any tag

    VoltageSensor batteryVoltageSensor;
    static final double V_REF = 12.0; // voltaj referinta

    private Follower follower;
    @Override
    public void init() {
        follower= Constants.createFollower(hardwareMap);
        // starting pose Pedro (modifica daca vrei)
        follower.setStartingPose(new Pose(72,72));
        // AprilTag processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // camera
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        motor1 = hardwareMap.get(DcMotorEx.class,"m1");
        motor2 = hardwareMap.get(DcMotorEx.class,"m2");
        motor3 = hardwareMap.get(DcMotorEx.class,"m3");
        motor4 = hardwareMap.get(DcMotorEx.class,"m4");

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Pedro TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        if (gamepad1.a) autoAlign = true;   // start auto align
        if (gamepad1.b) autoAlign = false;


        if (autoAlign) {

            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;
            telemetry.addData("target tag", targetTag );

            // find desired tag
            for (AprilTagDetection tag : detections) {
                if (TARGET_ID == -1 || tag.id == TARGET_ID) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag != null && targetTag.ftcPose != null) {

                double x = targetTag.ftcPose.x;
                double yaw = targetTag.ftcPose.yaw;
                double dist = targetTag.ftcPose.z;
                telemetry.addData("X", x);
                telemetry.addData("X", dist);
                telemetry.addData("Yaw", yaw);


                double strafe = 0;
                double turn = 0;

                // ===== STRAFE ALIGN =====
                if (x > 2) {
                    strafe = -0.25;   // mergi stanga
                }
                else if (x < -2) {
                    strafe = 0.25;    // mergi dreapta
                }
                else {
                    strafe = 0;       // centrat
                }

                // ===== ROTATION ALIGN =====
                if(dist<-15) {
                    if (x > 1) {
                        turn =  -0.1;
                    } else if (x < 3) {
                        turn = 0.1;
                    } else {
                        turn = 0;
                    }
                }
                if(dist>-15) {
                    if (x > -2) {
                        turn = -0.1;
                    } else if (x < 2) {
                        turn = +0.1;
                    } else {
                        turn = 0;
                    }
                }

                follower.setTeleOpDrive(
                        0,
                        0,
                        turn,
                        true
                );

                telemetry.addLine("ALIGN MODE");

            } else {
                telemetry.addLine("NO TAG");

                follower.setTeleOpDrive(0,0,0,false);
            }
        }else {


            // ===== MANUAL DRIVE =====
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

        }




        // ===== DRIVE CONTROL =====
        follower.update();

        // ================= MOTOARE AUX =================
        motor1.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? gamepad2.left_stick_y : 0);
        motor2.setPower(Math.abs(gamepad2.right_stick_y) > 0.1 ? gamepad2.right_stick_y : 0);
        double vouttake = motor3.getVelocity();
        telemetry.addData("ticks", vouttake);
        // ===== MECANISME =====
        if (gamepad2.right_bumper) {
            motor3.setPower(compensatedPower(power1(-0.7,vouttake)));
            motor4.setPower(compensatedPower(power1(0.7,vouttake)));
        }
        else if (gamepad2.left_bumper) {
            motor3.setPower(compensatedPower(power2(-0.45,vouttake)));
            motor4.setPower(compensatedPower(power2(0.45,vouttake)));
        }
        else {
            motor3.setPower(0);
            motor4.setPower(0);
        }

        // ===== TELEMETRIE =====
        telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
        telemetry.update();


    }

    // ===== VOLTAGE COMPENSATION =====
    double compensatedPower(double power) {
        double voltage = batteryVoltageSensor.getVoltage();
        double adjusted = power * (V_REF / voltage);
        return clip(adjusted, -1, 1);
    }

    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    double power1(double power, double vouttake){
        double adjusted = power * (-1540 / vouttake);
        return clip(adjusted, -1, 1);
    }
    double power2(double power, double vouttake){
        double adjusted = power * (-1400 / vouttake);
        return clip(adjusted, -1, 1);
    }
}
