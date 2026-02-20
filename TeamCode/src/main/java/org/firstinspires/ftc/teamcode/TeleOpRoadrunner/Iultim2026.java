package org.firstinspires.ftc.teamcode.TeleOpRoadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp_Centrare_Finala_2026")
public class Iultim2026 extends LinearOpMode {

    // Drive și Hardware
    private SampleMecanumDrive drive;
    private DcMotorEx motor1, motor2, motor3, motor4;
    private VoltageSensor batteryVoltageSensor;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    double d, x;

    // --- PARAMETRI CONTROL ---
    static final double KP_TURN = 0.018;      // Ajustează dacă rotația e prea lentă/rapidă
    static final double MAX_TURN_SPEED = 0.45; // Viteza maximă de rotație automată
    static final double V_REF = 12.0;          // Voltaj referință
    static final double a = 56;

    @Override
    public void runOpMode() throws InterruptedException {

        //Condus
        drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Motoare auxiliare (Intake/Outtake)
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor3 = hardwareMap.get(DcMotorEx.class, "m3");
        motor4 = hardwareMap.get(DcMotorEx.class, "m4");

        // Setăm motoarele să nu folosească encodere pentru TeleOp (mai mult cuplu)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configurare AprilTag cu procesare simplificată pentru viteză
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Sistem Inițializat. Gata de Start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            x = xd();

            if (gamepad1.a) {
                //LOGICĂ ALINIERE (DOAR ROTAȚIE)
                AprilTagDetection targetTag = getFirstDetection();

                if (targetTag != null) {
                    // Calculăm rotația bazată pe Bearing
                    // Dacă bearing e 0, tag-ul pe mijlocul imaginii.
                    double turnPower = targetTag.ftcPose.bearing * KP_TURN;

                    // Limităm viteza
                    turnPower = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, turnPower));

                    // Folosim setDrivePower(0, 0, turn) pentru a forța RR
                    // robotul rămâne  pe loc.
                    drive.setDrivePower(new Pose2d(0, 0, turnPower));

                    telemetry.addData("STATUS", "ALINIERE ACTIVĂ");
                    telemetry.addData("Eroare Unghi", targetTag.ftcPose.bearing);
                } else {
                    // Dacă nu vede tag-ul, nu mișcăm nimic
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    telemetry.addLine("STATUS: CĂUTARE TAG...");
                }
            } else {
                // CONDUS TELEOP NORMAL
                // Folosim WeightedDrivePower pentru condus
                drive.setWeightedDrivePower(new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                ));
            }

            // Actualizări obligatorii
            drive.update();
            controlAuxMotors();

            // Telemetrie pentru monitorizare voltaj
            telemetry.addData("Voltaj Baterie", batteryVoltageSensor.getVoltage());
            telemetry.update();
        }

        // Închidem camera la final
        visionPortal.close();
    }

    /**
     * Găsește prima detecție validă din lista de AprilTags
     */
    private AprilTagDetection getFirstDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                return detection;
            }
        }
        return null;
    }

    private double xd(){
      List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if (detection.id == 24 || detection.id ==20){
                d = detection.ftcPose.range;
            }
            telemetry.addData("distanta", d);
        }
        return d;
    }

    /**
     * Control motoare auxiliare
     */
    private void controlAuxMotors() {
        // Motoare controlate prin stick-uri gamepad 2
        motor1.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? gamepad2.left_stick_y : 0);
        motor2.setPower(Math.abs(gamepad2.right_stick_y) > 0.1 ? -gamepad2.right_stick_y : 0);

        double voutake = motor3.getVelocity();
        telemetry.addData("gjhadfghjds", voutake);

        // Control motoare 3 și 4 prin Bumpers cu compensare de voltaj
        if (gamepad2.right_bumper) {
             motor3.setPower(getCompensatedPower(baseAdjust(-0.58, voutake, -1400)));
            motor4.setPower(getCompensatedPower(baseAdjust(0.58, voutake, -1400)));
            //motor3.setPower(getCompensatedPower(-0.6));
            //motor4.setPower(getCompensatedPower(0.6));
        } else if (gamepad2.left_bumper) {
            motor3.setPower(getCompensatedPower(baseAdjust(-0.49, voutake, -1250)));
            motor4.setPower(getCompensatedPower(baseAdjust(0.49, voutake, -1250)));
        }else {

            motor3.setPower(getCompensatedPower (0));
            motor4.setPower(getCompensatedPower(0));
        }

    }

    private double getCompensatedPower(double power) {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        return Math.max(-1, Math.min(1, power * (V_REF / currentVoltage)));
    }

    private double baseAdjust(double power, double currentVel, double targetVel) {
        // Evităm împărțirea la zero sau calculele când motorul stă
        if (Math.abs(currentVel) < 1) return power;
        return Math.max(-1, Math.min(1, power * (targetVel / currentVel)));
    }
     private double VFD(double x){
        double v = 0;
        v = x/ Math.cos(56);
        telemetry.addData("cos", Math.cos(56));
        v = v*Math.sqrt(9.81/(Math.tan(56)*x-1));
        return v;
     }
}