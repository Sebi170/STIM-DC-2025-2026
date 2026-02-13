package org.firstinspires.ftc.teamcode.OA;

/*
 * This is an example of a more complex path to really test the tuning.
 */
/* @Autonomous(name = "ULTIMAUTO ",group = "drive")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(5, 5, Math.toRadians(30)))
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        /*drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    //}
//}
