package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Ultim2026")
public class TeleOPH extends OpMode {

    DcMotorEx motor1, motor2, motor3, motor4;
    VoltageSensor batteryVoltageSensor;

    static final double TICKS_PER_REV = 28.0;
    static final double V_REF = 12.0;

    private Follower follower;

    // ================= PID VARIABLES =================
    private double KP = 0.004;
    private double KI = 0.003;
    private double KD = 0.000;
    private double KF = 0.4;
    private double KP_SYNC = 0.001;

    private double INTEGRAL_CLAMP = 2000;

    private long prevTimeNs = -1;

    private double masterIntegral = 0;
    private double masterPrevError = 0;
    private double masterDeriv = 0;

    private double slaveIntegral = 0;
    private double slavePrevError = 0;
    private double slaveDeriv = 0;

    private double syncIntegral = 0;

    // ===== LIVE TUNING CONTROL =====
    double tuneStep = 0.00001;
    boolean lastUp, lastDown, lastLeft, lastRight, lastY, lastA;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72));

        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");

        motor3 = hardwareMap.get(DcMotorEx.class, "m3");
        motor4 = hardwareMap.get(DcMotorEx.class, "m4");

        motor3.setDirection(DcMotor.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // ================= DRIVE =================
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // ================= AUX MOTORS =================
        motor1.setPower(Math.abs(gamepad2.left_stick_y) > 0.1 ? gamepad2.left_stick_y : 0);
        motor2.setPower(Math.abs(gamepad2.right_stick_y) > 0.1 ? gamepad2.right_stick_y : 0);

        double masterVel = motor3.getVelocity();
        double slaveVel = motor4.getVelocity();

        // ================= LIVE PID TUNING =================
        if (gamepad2.dpad_up && !lastUp) KP += tuneStep;
        if (gamepad2.dpad_down && !lastDown) KP -= tuneStep;

        if (gamepad2.dpad_right && !lastRight) KI += tuneStep;
        if (gamepad2.dpad_left && !lastLeft) KI -= tuneStep;

        if (gamepad2.y && !lastY) KD += tuneStep;
        if (gamepad2.a && !lastA) KD -= tuneStep;

        KF += gamepad2.right_trigger * 0.001;
        KF -= gamepad2.left_trigger * 0.001;

        lastUp = gamepad2.dpad_up;
        lastDown = gamepad2.dpad_down;
        lastLeft = gamepad2.dpad_left;
        lastRight = gamepad2.dpad_right;
        lastY = gamepad2.y;
        lastA = gamepad2.a;

        // ================= OUTTAKE CONTROL =================
        if (gamepad2.right_bumper) {

            double[] powers = baseAdjustSyncedRPM(masterVel, slaveVel, 3500);
            motor3.setPower(powers[0]);
            motor4.setPower(powers[1]);

        } else if (gamepad2.left_bumper) {

            double[] powers = baseAdjustSyncedRPM(masterVel, slaveVel, 2000);
            motor3.setPower(powers[0]);
            motor4.setPower(powers[1]);

        } else {

            motor3.setPower(0);
            motor4.setPower(0);
        }

// ================= TELEMETRY =================

        double masterRPM = masterVel * 60.0 / TICKS_PER_REV;
        double slaveRPM  = slaveVel  * 60.0 / TICKS_PER_REV;

        double targetRPM = 0;
        if (gamepad2.right_bumper) targetRPM = 3500;
        if (gamepad2.left_bumper)  targetRPM = 2000;

        double rpmError = targetRPM - masterRPM;

        telemetry.addLine("========== OUTTAKE ==========");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Master RPM", "%.1f", masterRPM);
        telemetry.addData("Slave RPM", "%.1f", slaveRPM);
        telemetry.addData("RPM Error", "%.1f", rpmError);

        telemetry.addLine("========== PID ==========");
        telemetry.addData("KP", "%.6f", KP);
        telemetry.addData("KI", "%.6f", KI);
        telemetry.addData("KD", "%.6f", KD);
        telemetry.addData("KF", "%.4f", KF);
        telemetry.addData("KP_SYNC", "%.6f", KP_SYNC);

        telemetry.addLine("========== SYSTEM ==========");
        telemetry.addData("Battery Voltage", "%.2f", batteryVoltageSensor.getVoltage());
        telemetry.addData("Master Power", "%.3f", motor3.getPower());
        telemetry.addData("Slave Power", "%.3f", motor4.getPower());

        telemetry.update();
    }

    // ================= RPM PID FUNCTION =================
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

        return new double[]{masterPower, slavePower};
    }

    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}