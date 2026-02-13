package org.firstinspires.ftc.teamcode.OA;


import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name = "autoteo")
public class autoteo extends LinearOpMode {


    DcMotor motor1, motor2, motor3, motor4;
    VoltageSensor batteryVoltageSensor;
    static final double V_REF = 11;


    // ===== COORDONATE COS (Road Runner - inch) =====
    public static double BASKET_X = 0.0;
    public static double BASKET_Y = 0.0;


    // toleranta pentru auto-aim
    public static double AIM_TOLERANCE_DEG = 2.0;


    // coeficienti shooter (modificabili din Dashboard)
    public static double a = 0.009;
    public static double b = 0.19;


    @Override
    public void runOpMode() throws InterruptedException {


        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        // ===== TRAIECTORIILE TALE =====
        Trajectory preload = drive.trajectoryBuilder(startPose)
                .forward(53)
                .build();


        TrajectorySequence intoarcere = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(58.118, -35, Math.toRadians(-40)))
                .build();


        TrajectorySequence bile = drive.trajectorySequenceBuilder(intoarcere.end())
                .turn(Math.toRadians(-5))
                .setConstraints(
                        new MecanumVelocityConstraint(10, TRACK_WIDTH),
                        new ProfileAccelerationConstraint(30)
                )
                .lineToConstantHeading(new Vector2d(68.118, -30))
                .build();


        TrajectorySequence aruncare = drive.trajectorySequenceBuilder(bile.end())
                .lineToLinearHeading(new Pose2d(58.118, 0, Math.toRadians(40)))
                .build();


        TrajectorySequence iesire = drive.trajectorySequenceBuilder(aruncare.end())
                .strafeLeft(6)
                .build();


        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {


            // ======== PRELOAD: AUTO-AIM + POWER DIN DISTANTA ========
            Pose2d pose = drive.getPoseEstimate();


            autoAim(drive, pose);


            double dist = getDistanceToBasket(pose);
            double shooterPower = calculateShooterPower(dist);


            motor3.setPower(compensatedPower(-shooterPower));
            motor4.setPower(compensatedPower(shooterPower));


            drive.followTrajectory(preload);


            motor2.setPower(-0.75);
            motor1.setPower(-0.75);
            sleep(3000);


            motor3.setPower(compensatedPower(-0.2));
            motor4.setPower(compensatedPower(0.2));


            drive.followTrajectorySequence(intoarcere);


            motor1.setPower(-0.6);
            motor2.setPower(-0.1);


            drive.followTrajectorySequence(bile);


            // ======== A DOUA ARUNCARE: AUTO-AIM + POWER ========
            Pose2d pose2 = drive.getPoseEstimate();
            autoAim(drive, pose2);


            double dist2 = getDistanceToBasket(pose2);
            double shooterPower2 = calculateShooterPower(dist2);


            motor3.setPower(compensatedPower(-shooterPower2));
            motor4.setPower(compensatedPower(shooterPower2));


            drive.followTrajectorySequence(aruncare);
            sleep(3000);
            motor4.setPower(0);
            motor3.setPower(0);
            motor2.setPower(0);
            motor1.setPower(0);


            drive.followTrajectorySequence(iesire);


            // ===== TELEMETRIE DEBUG =====
            Pose2d debugPose = drive.getPoseEstimate();
            telemetry.addData("X", debugPose.getX());
            telemetry.addData("Y", debugPose.getY());
            telemetry.addData("Heading", Math.toDegrees(debugPose.getHeading()));
            telemetry.addData("Dist to Basket", getDistanceToBasket(debugPose));
            telemetry.addData("Shooter Power", calculateShooterPower(getDistanceToBasket(debugPose)));
            telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
            telemetry.update();


            break; // iesim din while dupa ce terminam traseul
        }
    }


    // ===== FUNCȚII UTILITARE =====


    void autoAim(SampleMecanumDrive drive, Pose2d pose) {
        double targetAngle = getAngleToBasket(pose);
        double angleError = targetAngle - pose.getHeading();


        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;


        if (Math.abs(Math.toDegrees(angleError)) > AIM_TOLERANCE_DEG) {
            drive.turn(angleError);
        }
    }


    double getDistanceToBasket(Pose2d robotPose) {
        double dx = BASKET_X - robotPose.getX();
        double dy = BASKET_Y - robotPose.getY();
        return Math.hypot(dx, dy);
    }


    double getAngleToBasket(Pose2d robotPose) {
        double dx = BASKET_X - robotPose.getX();
        double dy = BASKET_Y - robotPose.getY();
        return Math.atan2(dy, dx);
    }


    double calculateShooterPower(double distance) {
        double power = a * distance + b;
        return clip(power, 0.25, 1.0);
    }


    double compensatedPower(double powerDorita) {
        double voltage = batteryVoltageSensor.getVoltage();
        double power = powerDorita * (V_REF / voltage);
        return clip(power, -1.0, 1.0);
    }
}

