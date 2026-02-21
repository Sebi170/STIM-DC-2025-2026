package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Far Red 2")
public class AutoFarRed2 extends OpMode {

    private Follower follower;
    private VoltageSensor batteryVoltageSensor;

    double masterVel;
    double slaveVel;

    private Timer pathTimer, actionTimer, opmodeTimer;

    static final double V_REF = 12.0;
    static final double TICKS_PER_REV = 28;

    boolean shooterEnabled;
    double shooterTargetRPM;
    DcMotorEx motor1, motor2, motor3, motor4;

    private int pathState;

    boolean canShoot;

    private double RPM_TOLERANCE = 50;      // allowed error
    private double STABLE_TIME = 0.2;       // seconds required at speed
    private double atSpeedTimer = 0;

    private Path scorePreload;
    private PathChain Path1, Path2, Path3, Path4, Path5;
    private final Pose startPose = new Pose(85.2, 8.201398601398598, Math.toRadians(-90));

    public boolean Outtake()
    {

        if (actionTimer.getElapsedTimeSeconds() < 1.f)
            return false;

        motor1.setPower(-0.75);
        motor2.setPower(-0.75);

        if (actionTimer.getElapsedTimeSeconds() < 3.5f)
            return false;

        motor3.setPower(0);
        motor4.setPower(0);
        motor2.setPower(0);
        motor1.setPower(0);
        shooterEnabled = false;
        shooterTargetRPM = 0;
        return true;
    }

    public void StartOutake(int target)
    {
        shooterEnabled = true;
        shooterTargetRPM = target;
    }

    public void Intake()
    {
        motor1.setPower(-1);
        motor2.setPower(-0.08);
    }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.620, 8.201),
                                new Pose(82.976, 19.938)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-120))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.976, 19.938),
                                new Pose(130, 22.959)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-65))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132, 22.959),
                                new Pose(132, 12.3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(-30))
                .build();
        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132, 13),
                                new Pose(132.4,11.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-90))
                .build();
        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.976, 19.938),
                                new Pose(84, 20)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-120))
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                StartOutake(3100);
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if (!Outtake())
                        return;
                    follower.followPath(Path2,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    Intake();

                    if (actionTimer.getElapsedTimeSeconds() < 5.f)
                        follower.followPath(Path3, 0.4, false);
                    else
                        setPathState(3);
                }
                break;
            case 3:
                if (actionTimer.getElapsedTimeSeconds() < 5.f) {
                    follower.followPath(Path4);
                    setPathState(4);
                }
                else
                    setPathState(4);
                break;
            case 4:
                if(!follower.isBusy()) {
                    motor2.setPower(0.2);
                    if (actionTimer.getElapsedTimeSeconds() < 0.5f)
                        return;
                    motor2.setPower(0);
                    if (actionTimer.getElapsedTimeSeconds() < 0.5f)
                        return;
                    StartOutake(3100);
                    follower.followPath(Path1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if (!Outtake())
                        return;
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        masterVel = motor3.getVelocity();
        slaveVel = motor4.getVelocity();

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
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Can Shoot", canShoot);
        telemetry.addData("Time At Speed", atSpeedTimer);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor1.setDirection(DcMotor.Direction.FORWARD);

        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor3 = hardwareMap.get(DcMotorEx.class, "m3");
        motor3.setDirection(DcMotor.Direction.REVERSE);

        motor4 = hardwareMap.get(DcMotorEx.class, "m4");
        motor4.setDirection(DcMotor.Direction.FORWARD);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    private double getCompensatedPower(double power) {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        return Math.max(-1, Math.min(1, power * (V_REF / currentVoltage)));
    }
    // ================= PID VARIABLES =================
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
}
