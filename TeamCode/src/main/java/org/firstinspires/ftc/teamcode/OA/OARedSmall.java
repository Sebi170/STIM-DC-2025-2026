package org.firstinspires.ftc.teamcode.OA;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name= "OARedSmall")

public class OARedSmall extends LinearOpMode {

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
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                        .back(4.72)
                .turn(Math.toRadians(-20.95))
                                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(-25.591,0,Math.toRadians(20.95)))
                        .build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(traj1);
            motor3.setPower(compensatedPower(-0.65));
            motor4.setPower(compensatedPower(0.65));
            sleep(2000);
            motor1.setPower(-0.75);
            motor2.setPower(-0.75);
            sleep(3000);        /// stam 3 secunde pentru a arunca toate bilele
        drive.followTrajectorySequence(traj2);
            motor3.setPower(0);            /// oprim motoarele
            motor4.setPower(0);            /// de outake
            motor1.setPower(0.5);          /// aruncam bilele care nu au fost aruncate
            motor2.setPower(0.5);          /// pentru a nu primi penalizare
            sleep(2000);        /// timp 2 secunde
            motor1.setPower(0);
            motor2.setPower(0);
            sleep(25000);
        }
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