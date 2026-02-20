package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Testauto_BUN2026", group = "Examples")
public class Auto3 extends OpMode {

    private Follower follower;
    private VoltageSensor batteryVoltageSensor;

    private Timer pathTimer, actionTimer, opmodeTimer;

    static final double V_REF = 12.0;
    static final double TICKS_PER_REV = 28;

    DcMotorEx motor1, motor2, motor3, motor4;

    private int pathState;

    private Path scorePreload;
    private PathChain launchPose, ballPose, takePose, shootPose;
    private final Pose startPose = new Pose(54.5015, 8.28, Math.toRadians(-90));
    private final Pose scorePose = new Pose(61.6676, 13.5998, Math.toRadians(-65));
    private final Pose AballPose = new Pose(9.936567772511845, 23.7, Math.toRadians(-100));
    private final Pose takeBall = new Pose(8.76777251184834, 10.4, Math.toRadians(-100));

    public boolean Outtake()
    {

        motor1.setPower(-0.9);
        motor2.setPower(-0.6);

        if (actionTimer.getElapsedTimeSeconds() < 3.f)
            return false;

        motor3.setPower(0);
        motor4.setPower(0);
        motor2.setPower(0);
        motor1.setPower(0);

        return true;
    }
    public void PrepareOuttake() {
        double voutake = motor3.getVelocity();
        double voutake2 = motor4.getVelocity();
        double[] powers = baseAdjustSynced(voutake, voutake2, 1000 * TICKS_PER_REV / 60.0);
        motor3.setPower((powers[0]));
        motor4.setPower((powers[1]));
    }

    public void Intake()
    {
        follower.setMaxPower(0.2);
        motor1.setPower(-1);
        motor2.setPower(-0.2);
        if (actionTimer.getElapsedTimeSeconds() < 4.f) {
            motor1.setPower(-1);
            motor2.setPower(0);
        }
        follower.setMaxPower(1);
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        launchPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        ballPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, AballPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), AballPose.getHeading())
                .build();

        takePose = follower.pathBuilder()
                .addPath(new BezierLine(AballPose, takeBall))
                .setLinearHeadingInterpolation(AballPose.getHeading(), takeBall.getHeading())
                .build();

        shootPose = follower.pathBuilder()
                .addPath(new BezierLine(takeBall, scorePose))
                .setLinearHeadingInterpolation(takeBall.getHeading(), scorePose.getHeading())
                .build();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                double voutake = motor3.getVelocity();
                motor3.setPower(getCompensatedPower(-0.4));
                motor4.setPower(getCompensatedPower(-0.4));
                follower.followPath(launchPose);
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;
            case 1:
                motor1.setPower(getCompensatedPower(0.95));
                motor2.setPower(getCompensatedPower(0.95));
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Intake();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(takePose,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    PrepareOuttake();
                    follower.followPath(shootPose,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 1.f)
                        return;
                    if (!Outtake())
                        return;
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
    private double KP             = 0.0012;
    private double KI             = 0.00003;
    private double KD             = 0.0010;
    private double KF             = 0.4;     // base kick to overcome inertia
    private double INTEGRAL_CLAMP = 0.15;
    private double KP_SYNC        = 0.002;

    // Shared timestamp
    private long   prevTimeNs      = -1;

    // Master state
    private double masterIntegral  = 0.0;
    private double masterPrevError = 0.0;
    private double masterDeriv     = 0.0;

    // Slave state
    private double slaveIntegral   = 0.0;
    private double slavePrevError  = 0.0;
    private double slaveDeriv      = 0.0;

    // Sync state
    private double syncIntegral    = 0.0;

    private double[] baseAdjustSynced(double masterVel, double slaveVel, double targetVel) {

        // --- SHARED DT ---
        long   nowNs = System.nanoTime();
        double dt    = (prevTimeNs < 0) ? 0.02 : (nowNs - prevTimeNs) / 1e9;
        prevTimeNs   = nowNs;
        dt           = Math.max(1e-6, Math.min(dt, 0.5));

        // --- MASTER ---
        double masterError  = targetVel - masterVel;
        masterIntegral     += masterError * dt;
        masterIntegral      = Math.max(-INTEGRAL_CLAMP, Math.min(INTEGRAL_CLAMP, masterIntegral));
        double masterRawD   = (masterError - masterPrevError) / dt;
        masterDeriv         = 0.7 * masterDeriv + 0.3 * masterRawD;
        masterPrevError     = masterError;

        double masterCorrection = KP * masterError
                + KI * masterIntegral
                + KD * masterDeriv;

        // --- SLAVE ---
        double slaveError = targetVel - slaveVel;
        slaveIntegral      += slaveError * dt;
        slaveIntegral       = Math.max(-INTEGRAL_CLAMP, Math.min(INTEGRAL_CLAMP, slaveIntegral));
        double slaveRawD    = (slaveError - slavePrevError) / dt;
        slaveDeriv          = 0.7 * slaveDeriv + 0.3 * slaveRawD;
        slavePrevError      = slaveError;

        double slaveCorrection = KP * slaveError
                + KI * slaveIntegral
                + KD * slaveDeriv;

        // --- SYNC NUDGE ---
        double syncError = masterVel - slaveVel;
        syncIntegral    += syncError * dt;
        syncIntegral     = Math.max(-0.1, Math.min(0.1, syncIntegral));
        double syncNudge = KP_SYNC * syncError + 0.0001 * syncIntegral;

// MASTER
        double ff           = KF * Math.signum(targetVel);
        double masterPower = Math.max(-1, Math.min(1, ff + KP * masterError + KI * masterIntegral + KD * masterDeriv));
        double slavePower  = Math.max(-1, Math.min(1, ff + slaveCorrection + syncNudge));

        return new double[]{masterPower, slavePower};
    }

    private double baseAdjust(double power, double currentVel, double targetVel) {
        // Evităm împărțirea la zero sau calculele când motorul stă
        if (Math.abs(currentVel) < 1) return power;
        return Math.max(-1, Math.min(1, power * (targetVel / currentVel)));
    }
}

