package org.firstinspires.ftc.teamcode.TeleOpRoadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp_teo")
public class Iultim2026redteo extends LinearOpMode {

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
        //motor2.setPower(Math.abs(gamepad2.right_stick_y) > 0.1 ? -gamepad2.right_stick_y : 0);
        if (gamepad2.left_trigger > 0)
        {
            motor2.setPower(-0.8);
        }
        else if (gamepad2.right_trigger > 0)
        {
            motor2.setPower(0.8);
        }
        else
            motor2.setPower(0);

        double voutake = motor3.getVelocity();
        double voutake1 = motor4.getVelocity();
        telemetry.addData("gjhadfghjds", voutake);
        telemetry.addData("gjhadfghjds", voutake1);

        // Control motoare 3 și 4 prin Bumpers cu compensare de voltaj
        if (gamepad2.right_bumper) {
             motor3.setPower(getCompensatedPower(baseAdjust(-VFD(x), voutake, -tiks_teoretic(VFD(x)))));
            motor4.setPower(getCompensatedPower(baseAdjust1(VFD(x), voutake, -tiks_teoretic(VFD(x)))));
            //motor3.setPower(getCompensatedPower(-0.6));
            //motor4.setPower(getCompensatedPower(0.6));
        } else if (gamepad2.left_bumper) {
            motor3.setPower(getCompensatedPower(baseAdjust(-VFD(x), voutake, -tiks_teoretic(VFD(x)))));
            motor4.setPower(getCompensatedPower(baseAdjust1(VFD(x), voutake, -tiks_teoretic(VFD(x)))));
            telemetry.addData("compensedPower", getCompensatedPower(baseAdjust1(VFD(x), voutake, -tiks_teoretic(VFD(x)))));
            telemetry.addData("compensedPower", getCompensatedPower(baseAdjust(-VFD(x), voutake, -tiks_teoretic(VFD(x)))));
        }
        else {

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

    private double baseAdjust1(double power, double currentVel, double targetVel) {
        // Evităm împărțirea la zero sau calculele când motorul stă
        if (Math.abs(currentVel) < 1) return power;
        return Math.max(-1, Math.min(1, power * (targetVel / currentVel)));
    }
     private double VFD(double x){
        double v = 0;
        x = x * 0.0254;
        v = x / 0.5591;
        telemetry.addData("distanta", x);
        v = v*Math.sqrt(9.81/(1.48256*x-1));
         v = v/15.3 ;
        if (v>= 0.95)
            v = 0.95;
        if (v<=0)
            v = 0;
         telemetry.addData("viteza", v);
         return v;
     }

     public double tiks_teoretic(double v){
        double C = 0.2419;
        double TPM = 537.7 / C;
        double tiks = v*TPM;
        telemetry.addData("tiks", tiks);
        return tiks;
     }
}