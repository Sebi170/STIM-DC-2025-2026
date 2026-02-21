package org.firstinspires.ftc.teamcode.TeleOp;

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

@TeleOp(name = "TeleOp Main")
public class TeleOpMain extends LinearOpMode {

    // Drive și Hardware
    private SampleMecanumDrive drive;
    private DcMotorEx motor1, motor2, motor3, motor4;
    private VoltageSensor batteryVoltageSensor;
    boolean canShoot;
    private double RPM_TOLERANCE = 50;      // allowed error
    private double STABLE_TIME = 0.2;       // seconds required at speed
    private double atSpeedTimer = 0;
    static final double TICKS_PER_REV = 28;

    boolean shooterEnabled;
    double shooterTargetRPM;
    double masterVel;
    double slaveVel;
    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    double d, x;

    // --- PARAMETRI CONTROL ---
    static final double KP_TURN = 0.018;      // Ajustează dacă rotația e prea lentă/rapidă
    static final double MAX_TURN_SPEED = 0.25; // Viteza maximă de rotație automată
    static final double V_REF = 12.0;          // Voltaj referință
    static final double a = 56;
    public void Intake()
    {
        motor1.setPower(-1);
        motor2.setPower(-0.01);
    }
    public void StopIntake()
    {
        motor1.setPower(0);
        motor2.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {

        //Condus
        drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Motoare auxiliare (Intake/Outtake)
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor3 = hardwareMap.get(DcMotorEx.class, "m3");
        motor3.setDirection(DcMotor.Direction.REVERSE);
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

            // 320cm distanta maxima
            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                Intake();
            } else {
                StopIntake();
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
        motor1.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? -0.9 : 0);
        motor2.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? gamepad2.right_stick_y : 0 );

        double voutake = motor3.getVelocity();
        double voutake1 = motor4.getVelocity();
        telemetry.addData("Viteza motor 1", voutake * 60.0 / TICKS_PER_REV);
        telemetry.addData("Viteza motor 2", voutake1 * 60.0 / TICKS_PER_REV);

        masterVel = motor3.getVelocity();
        slaveVel = motor4.getVelocity();
        telemetry.addData("RPM Target", shooterTargetRPM);

        if (shooterEnabled) {
            double[] powers = baseAdjustSyncedRPM(masterVel, slaveVel, shooterTargetRPM);
            motor3.setPower(powers[0]);
            motor4.setPower(powers[1]);
        }
        else {
            motor3.setPower(0);
            motor4.setPower(0);
            if (shooterTargetRPM == 0) {
                canShoot = false;
                atSpeedTimer = 0;
            }
        }
    }

    private double KP = 0.004;
    private double KI = 0.002;
    private double KD = 0.000;
    private double KF = 0.4;
    private double KP_SYNC = 0.001;

    private double INTEGRAL_CLAMP = 50;

    private long prevTimeNs = -1;

    private double masterIntegral = 0;
    private double masterPrevError = 0;
    private double masterDeriv = 0;

    private double slaveIntegral = 0;
    private double slavePrevError = 0;
    private double slaveDeriv = 0;

    private double syncIntegral = 0;


    private double[] baseAdjustSyncedRPM(double masterTicksPerSec,
                                         double slaveTicksPerSec,
                                         double targetRPM) {

        double targetTicksPerSec = targetRPM * TICKS_PER_REV / 60.0;

        long nowNs = System.nanoTime();
        double dt = (prevTimeNs < 0) ? 0.02 : (nowNs - prevTimeNs) / 1e9;
        prevTimeNs = nowNs;
        dt = Math.max(1e-6, Math.min(dt, 0.5));

        // MASTER
        double masterError = targetTicksPerSec - masterTicksPerSec;
        masterIntegral += masterError * dt;
        masterIntegral = clip(masterIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

        double masterRawD = (masterError - masterPrevError) / dt;
        masterDeriv = 0.7 * masterDeriv + 0.3 * masterRawD;
        masterPrevError = masterError;

        double masterCorrection =
                KP * masterError +
                        KI * masterIntegral +
                        KD * masterDeriv;

        // SLAVE
        double slaveError = targetTicksPerSec - slaveTicksPerSec;
        slaveIntegral += slaveError * dt;
        slaveIntegral = clip(slaveIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

        double slaveRawD = (slaveError - slavePrevError) / dt;
        slaveDeriv = 0.7 * slaveDeriv + 0.3 * slaveRawD;
        slavePrevError = slaveError;

        double slaveCorrection =
                KP * slaveError +
                        KI * slaveIntegral +
                        KD * slaveDeriv;

        // SYNC
        double syncError = masterTicksPerSec - slaveTicksPerSec;
        syncIntegral += syncError * dt;
        syncIntegral = clip(syncIntegral, -1000, 1000);

        double syncNudge = KP_SYNC * syncError;

        double ff = KF * Math.signum(targetTicksPerSec);

        double masterPower = clip(ff + masterCorrection, -1, 1);
        double slavePower = clip(ff + slaveCorrection + syncNudge, -1, 1);

        double masterRPM = masterTicksPerSec * 60.0 / TICKS_PER_REV;
        double rpmError = targetRPM - masterRPM;

        // Check if within tolerance
        if (Math.abs(rpmError) < RPM_TOLERANCE) {
            atSpeedTimer += dt;
        } else {
            atSpeedTimer = 0;
        }

        // If stable long enough → allow shooting
        canShoot = atSpeedTimer >= STABLE_TIME;
        return new double[]{masterPower, slavePower};
    }

    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
     private double GetDistance(double x){
        double v = 0;
        x = x * 0.0254 + 0.02;
        telemetry.addData("distanta", x);
        return x;
     }

    public double calculateRequiredRPM(
            double distanceMeters,     // horizontal distance to goal (m)
            double launchAngleDeg,     // shooter angle (degrees)
            double shooterHeight,      // ball exit height (m)
            double goalHeight,         // goal center height (m)
            double wheelRadiusMeters,  // flywheel radius (m)
            double efficiency          // 0.85–0.95 realistic
    ) {

        final double g = 9.81;

        double theta = Math.toRadians(launchAngleDeg);
        double deltaH = goalHeight - shooterHeight;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denominator = 2 * cos * cos * (distanceMeters * tan - deltaH);

        // Prevent impossible shots (bad angle / geometry)
        if (denominator <= 0) {
            return 0;
        }

        double velocity = Math.sqrt((g * distanceMeters * distanceMeters) / denominator);

        // Account for slip losses
        velocity /= efficiency;

        // Convert linear velocity → RPM
        double rpm = (velocity * 60.0) / (2.0 * Math.PI * wheelRadiusMeters);

        return rpm;
    }

    public double GetDistanceRPM(double distance)
    {
        return calculateRequiredRPM(distance, 56, 0.35, 1, 0.0385, 0.49);
    }

}