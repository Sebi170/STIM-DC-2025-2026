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
/// plan de afaceri
/// merg 59.11inch, fac stanga, iau bile si arunc
@Config
@Autonomous(name = "OABlueBigv2")
public class OABlueBigv2 extends LinearOpMode {

    DcMotor motor1, motor2, motor3, motor4;
    VoltageSensor batteryVoltageSensor;

    static final double V_REF = 11.0;

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
        telemetry.addLine("Ce faci,Sebi?");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        // ================= TRAJECTORII =================

        // 1️⃣ Merg înainte ~59 inch
        drive.followTrajectory(
                drive.trajectoryBuilder(startPose)
                        .forward(59.11)
                        .build()
        );

        // --- aici poți activa motorul 3 și 4 pentru intake ---
        motor3.setPower(compensatedPower(-0.58));
        motor4.setPower(compensatedPower(0.58));
        sleep(1000); // timpul cât ia bile
        motor1.setPower(compensatedPower(-0.75));
        motor2.setPower(compensatedPower(-0.75));
        sleep(2500);
        motor3.setPower(0);
        motor4.setPower(0);
        motor1.setPower(-0.5);
        motor2.setPower(-0.1);

        // 2️⃣ Întoarcere și mers spre zona de aruncare
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-139))
                .forward(24)
                .waitSeconds(1)
                .setConstraints(
                        new MecanumVelocityConstraint(8, TRACK_WIDTH),
                        new ProfileAccelerationConstraint(20)
                )
                .forward(26)
                .resetConstraints()
                .waitSeconds(1.7)
                .build();

        drive.followTrajectorySequence(traj2);
        sleep(1000);
        // 3️⃣ Întoarcere către poziția inițială
        Pose2d returnPose = new Pose2d(59.11, 0, 0); // exact ce ai definit
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(returnPose)
                .build();

        drive.followTrajectorySequence(traj3);

        // 4️⃣ Strafing dreapta pentru aliniere finală
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(17)
                .build();

        drive.followTrajectorySequence(traj4);

        // ================= FINAL =================
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        sleep(10000); // pauză ca să vezi finalul
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
