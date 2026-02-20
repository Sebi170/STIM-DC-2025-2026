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

@Autonomous(name = "AutoFinal", group = "Examples")
public class AutoFinal extends OpMode {

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
    private final Pose scorePose = new Pose(61.6676, 13.5998, Math.toRadians(-70));
    private final Pose AballPose = new Pose(9.95, 23.7, Math.toRadians(-105));
    private final Pose takeBall = new Pose(8.76777251184834, 10.1, Math.toRadians(-100));

    public boolean Outtake()
    {
        if (actionTimer.getElapsedTimeSeconds() < 0.5f)
            return false;
        motor1.setPower(-0.9);
        motor2.setPower(-0.45);

        if (actionTimer.getElapsedTimeSeconds() < 4.f)
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
        telemetry.addData("voutake",voutake );
        telemetry.addData("voutake2", voutake2 );
        double power1 = baseAdjust(0.7, voutake, 1300);
        double power2 = baseAdjust(0.7, voutake2, 1300);
        motor3.setPower(getCompensatedPower(0.7));
        motor4.setPower(getCompensatedPower(0.7));
    }

    public void Intake()
    {
        follower.setMaxPower(0.1);
        follower.setYVelocity(0.4);
        motor1.setPower(-1);
        motor2.setPower(-0.1);
        if (actionTimer.getElapsedTimeSeconds() < 4.f) {
            motor1.setPower(-1);
            motor2.setPower(0);
        }
        follower.setMaxPower(1);
        follower.setYVelocity(1);
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
                PrepareOuttake();
                follower.followPath(launchPose);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 1.f) // lasam outake ul sa isi ia viteza
                        return;

                    if (!Outtake())
                        return;

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ballPose,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Intake();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(takePose,true);
                    if (actionTimer.getElapsedTimeSeconds() < 5.f)
                        return;
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
                    if (actionTimer.getElapsedTimeSeconds() < 3.f)
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

    private double baseAdjust(double power, double currentVel, double targetVel) {
        // Evităm împărțirea la zero sau calculele când motorul stă
        if (Math.abs(currentVel) < 1) return power;
        return Math.max(-1, Math.min(1, power * (targetVel / currentVel)));
    }
}

