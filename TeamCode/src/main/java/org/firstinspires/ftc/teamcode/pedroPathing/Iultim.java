package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Ultim2026")
public class Iultim extends OpMode {

    // ===== HARDWARE =====
    private DcMotorEx motor1, motor2, motor3, motor4;

    private VoltageSensor batteryVoltageSensor;

    // ===== VISION =====
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ===== PEDRO PATHING =====
    private Follower follower;

    // ===== LOGICĂ =====
    boolean autoAlign = false;
    boolean isHolding = false; // Variabilă critică pentru frânare

    // ===== SETĂRI TAG =====
    int TARGET_ID = -1; // -1 = orice tag

    // ===== CONSTANTE =====
    static final double V_REF = 12.0;

    @Override
    public void init() {
        // Inițializare Follower Pedro Pathing
        follower = Constants.createFollower(hardwareMap);

        // Setează poziția de start (importantă pentru Field Centric)
        follower.setStartingPose(new Pose(72, 72, 0));

        // Inițializare AprilTag
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        // Inițializare Motoare Auxiliare
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor3 = hardwareMap.get(DcMotorEx.class, "m3");
        motor4 = hardwareMap.get(DcMotorEx.class, "m4");

        // Configurare motoare auxiliare (opțional, dar recomandat)
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Pedro TeleOp Ready - Active Hold Enabled");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Dacă muți robotul cu mâna înainte de start, follower-ul știe unde e
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Citire voltaj pentru compensare
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Toggle Auto Align
        if (gamepad1.a) autoAlign = true;
        if (gamepad1.b) autoAlign = false;

        // =========================================================
        //                 LOGICA DE DRIVE (PEDRO)
        // =========================================================

        if (autoAlign) {
            handleAutoAlign();
            isHolding = false; // Resetăm starea de hold când ieșim din manual
        } else {
            handleManualDrive();
        }

        // Actualizare PID-uri Pedro Pathing (Aici se întâmplă mișcarea fizică)
        follower.update();

        // =========================================================
        //                 MOTOARE AUXILIARE
        // =========================================================
        handleAuxMotors();

        // =========================================================
        //                 TELEMETRIE
        // =========================================================
        telemetry.addData("Mode", autoAlign ? "AUTO ALIGN" : (isHolding ? "HOLDING (Braking)" : "MANUAL DRIVING"));
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Battery", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }

    // ---------------------------------------------------------
    // METODĂ: MANUAL DRIVE CU INERTIA CANCELLATION
    // ---------------------------------------------------------
    private void handleManualDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Deadzone pentru a evita mișcarea fină accidentală
        boolean isStickMoved = Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(turn) > 0.1;
        telemetry.addData("mers", isStickMoved);
        if (isStickMoved) {
            // Șoferul controlează robotul
            isHolding = false;
            follower.setTeleOpDrive(forward, strafe, turn, true); // true = Robot Centric (schimbă în false pt Field Centric)
        } else {
            // Joystick-urile sunt la 0
            if (!isHolding) {
                // Executăm asta O SINGURĂ DATĂ exact când dăm drumul la stick

                // 1. Oprim orice vector de viteză anterior
                follower.breakFollowing();

                // 2. Setăm un punct de hold la coordonatele CURENTE
                follower.holdPoint(follower.getPose());

                // 3. Activăm flag-ul ca să nu resetăm holdPoint-ul la fiecare loop
                isHolding = true;
            }
            // Notă: Când isHolding e true, follower.update() din main loop va ține robotul pe loc
        }
    }

    // ---------------------------------------------------------
    // METODĂ: AUTO ALIGN (APRIL TAG)
    // ---------------------------------------------------------
    private void handleAutoAlign() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection targetTag = null;

        for (AprilTagDetection tag : detections) {
            if (TARGET_ID == -1 || tag.id == TARGET_ID) {
                targetTag = tag;
                break;
            }
        }

        if (targetTag != null && targetTag.ftcPose != null) {
            double x = targetTag.ftcPose.x;
            double dist = targetTag.ftcPose.z; // distanța față de cameră

            double strafe = 0;
            double turn = 0;

            // Simplificare logică aliniere
            if (x > 2) strafe = -0.25;
            else if (x < -2) strafe = 0.25;

            // Logică distanță/rotație
            if (dist < -15) {
                if (x > 1) turn = -0.1;
                else if (x < 3) turn = 0.1; // Aici pare o mică eroare în logica ta originală, dar am păstrat-o
            } else {
                if (x > -2) turn = -0.1;
                else if (x < 2) turn = 0.1;
            }

            follower.setTeleOpDrive(0, 0, turn, true);
        } else {
            telemetry.addLine("NO TAG FOUND");
            // Dacă nu vede tag, oprește-te (poți pune holdPoint aici dacă vrei frână bruscă)
            follower.setTeleOpDrive(0, 0, 0, false);
        }
    }

    // ---------------------------------------------------------
    // METODĂ: AUX MOTORS CONTROL
    // ---------------------------------------------------------
    private void handleAuxMotors() {
        // Control Joystick
        motor1.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? gamepad2.left_stick_y : 0);
        motor2.setPower(Math.abs(gamepad2.right_stick_y) > 0.1 ? gamepad2.right_stick_y : 0);
        double vouttake = motor3.getVelocity();
        telemetry.addData("ticks", vouttake);
        // Control Bumpers (Mecanisme)
        if (gamepad2.right_bumper) {
            motor3.setPower(compensatedPower(-0.6));
            motor4.setPower(compensatedPower(0.6));
        } else if (gamepad2.left_bumper) {
            motor3.setPower(compensatedPower(-0.55));
            motor4.setPower(compensatedPower(0.55));
        } else {
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }

    // ---------------------------------------------------------
    // UTILITARE
    // ---------------------------------------------------------
    double compensatedPower(double power) {
        double voltage = batteryVoltageSensor.getVoltage();
        // Evităm împărțirea la 0 sau valori aberante
        if (voltage < 7) return power;
        double adjusted = power * (V_REF / voltage);
        return Math.max(-1, Math.min(1, adjusted));
    }
}