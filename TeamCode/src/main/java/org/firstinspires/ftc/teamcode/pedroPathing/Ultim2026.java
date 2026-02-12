package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Ultim2026")
public class Ultim2026 extends OpMode {

    DcMotor motor1, motor2, motor3, motor4;

    VoltageSensor batteryVoltageSensor;
    static final double V_REF = 12.0; // voltaj referinta

    private Follower follower;
    @Override
    public void init() {
        follower= Constants.createFollower(hardwareMap);
        // starting pose Pedro (modifica daca vrei)
        follower.setStartingPose(new Pose(72,72));

        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        motor2 = hardwareMap.get(DcMotor.class,"motor2");
        motor3 = hardwareMap.get(DcMotor.class,"motor3");
        motor4 = hardwareMap.get(DcMotor.class,"motor4");

        telemetry.addLine("Pedro TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // ===== DRIVE CONTROL =====
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, turn, true);
        follower.update();

        // ===== MECANISME =====
        if (gamepad2.right_bumper) {
            motor3.setPower(compensatedPower(-0.7));
            motor4.setPower(compensatedPower(0.7));
        }
        else if (gamepad2.left_bumper) {
            motor3.setPower(compensatedPower(-0.5));
            motor4.setPower(compensatedPower(0.5));
        }
        else {
            motor3.setPower(0);
            motor4.setPower(0);
        }

        // ===== TELEMETRIE =====
        telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
        telemetry.update();

        draw();
    }

    // ===== VOLTAGE COMPENSATION =====
    double compensatedPower(double power) {
        double voltage = batteryVoltageSensor.getVoltage();
        double adjusted = power * (V_REF / voltage);
        return clip(adjusted, -1, 1);
    }

    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
