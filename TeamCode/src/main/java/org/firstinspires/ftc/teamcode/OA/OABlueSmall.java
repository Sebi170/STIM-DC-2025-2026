package org.firstinspires.ftc.teamcode.OA;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name= "OABlueSmall")

public class OABlueSmall extends LinearOpMode {

    DcMotor motor1, motor2, motor3, motor4;

    VoltageSensor batteryVoltageSensor;
    static final double V_REF = 11;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        Pose2d startPose = new Pose2d(0, 0, 0);
        telemetry.addLine("Ce faci,Sebi?");
        telemetry.update();
        drive.setPoseEstimate(startPose);

        waitForStart();
        if(isStopRequested())return;
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .back(4.72)
                .turn(Math.toRadians(21))
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-21.6535433,4.72,69))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder((traj2.end()))
                .setConstraints(new MecanumVelocityConstraint(8, TRACK_WIDTH),
                        new ProfileAccelerationConstraint(20))
                .forward(58.34644567)
                .build();
        TrajectorySequence traj4 =drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(4.72,0,-90))
                        .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder((traj4.end()))
                        .strafeLeft(15)
                                .build();

            motor3.setPower(compensatedPower(-0.65));
            motor4.setPower(compensatedPower(0.65));
            drive.followTrajectorySequence(traj1);
            motor1.setPower(-0.75);
            motor2.setPower(-0.75);
            sleep(2500);
            motor3.setPower(0);
            motor4.setPower(0);
            motor1.setPower(-0.5);
            motor2.setPower(-0.1);
            drive.followTrajectorySequence(traj2);
            motor3.setPower(compensatedPower(-0.65));
            motor4.setPower(compensatedPower(0.65));
            drive.followTrajectorySequence(traj3);
            sleep(2500);
            motor3.setPower(0);
            motor4.setPower(0);
            motor1.setPower(0);
            motor2.setPower(0);
            drive.followTrajectorySequence(traj4);
            sleep(25000);

    }
    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    double compensatedPower(double powerDorita) {
        double voltage = batteryVoltageSensor.getVoltage();
        double power = powerDorita * (V_REF / voltage);
        telemetry.addData("voltaj", voltage);
        return clip(power, -1.0, 1.0);
    }

}