package org.firstinspires.ftc.teamcode.OA;

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

/* PLAN AFACERI
    -Plecam de la albastru, triunghiul mare, robotul drept si pe mijloc cu linia alba a triunghiului, lipit de cos
    -Mergem inainte 51 in = ~130 cm , apoi aruncam 3 bile preincarcate
    -Se intoarce -140 grade
    -Merge inainte mai lent pentru a lua cele 3 bile de pe prima linie
    -Merge cu spatele, se intoarce 140 grade
    -Arunca bilele
    -Iesim din triunghi
 */



@Config
@Autonomous(name= "OARedBiginit")
public class OARedBiginit extends LinearOpMode {
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
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(52)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(139))
                .setConstraints (new MecanumVelocityConstraint(10, TRACK_WIDTH), // Viteza maximă dorită (în loc de 10, am pus 30 ca exemplu)
                        new ProfileAccelerationConstraint(30)           // Accelerația maximă dorită
                )
                .lineToConstantHeading(new Vector2d(28 ,38))
                .resetConstraints()
                .waitSeconds(1.5)
                .build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            motor3.setPower(compensatedPower(-0.55));  ///pregatim
            motor4.setPower(compensatedPower(0.55));
            drive.followTrajectory(traj1); ///traiectoria 1 merge inainte 51 inch
            motor1.setPower(compensatedPower(-0.75));         /// incepem sa aruncam bilele
            motor2.setPower(compensatedPower(-0.75));         /// preincarcate
            sleep(2500);        /// stam 3 secunde pentru a arunca toate bilele
            motor3.setPower(0);            /// oprim motoarele
            motor4.setPower(0);            /// de outake
            motor1.setPower(0.5);          /// aruncam bilele care nu au fost aruncate
            motor2.setPower(0.5);          /// pentru a nu primi penalizare
            sleep(1000);        /// timp 2 secunde
            motor1.setPower(-0.5);         /// pornim motoarele
            motor2.setPower(-0.1);
            sleep(2000);
            drive.followTrajectorySequence(traj2);
            motor3.setPower(0);
            motor4.setPower(0);
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
