package org.firstinspires.ftc.teamcode.drive.opmode.teleop.BS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Shooting Regression Tuner", group = "Tuning")
public class ShootingRegressionTuner extends OpMode {

    private Follower follower;
    private Turret turret;

    private static final Pose TARGET_POSE = new Pose(0, 144, 0);
    private static final double OFFSET = Math.toRadians(180.0);

    private double targetRPM = 2500.0;
    private double hoodPosition = 0.5;

    private static final double RPM_STEP = 50.0;
    private static final double HOOD_STEP = 0.01;

    // -----------------------------------------------------------------------
    // REGRESSION LINES — fill these in after collecting data points
    // Input (x) = distance to goal in inches
    // -----------------------------------------------------------------------
    private double regressionRPM(double distance) {
        // TODO: replace with RPM regression line
        // Example: return m * distance + b;
        return targetRPM;
    }

    private double regressionHood(double distance) {
        // TODO: replace with hood position regression line
        // Example: return m * distance + b;
        return hoodPosition;
    }
    // -----------------------------------------------------------------------

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", false, false);

        if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
        } else {
            follower.setPose(new Pose(72, 72, 0));
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.on();
        turret.setHoodPosition(hoodPosition);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET
        );

        // --- RPM control: bumpers ---
        if (gamepad1.rightBumperWasPressed()) {
            targetRPM += RPM_STEP;
        }
        if (gamepad1.leftBumperWasPressed()) {
            targetRPM -= RPM_STEP;
        }
        targetRPM = Math.max(0.0, targetRPM);

        // --- Hood control: dpad up/down ---
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            hoodPosition = Math.min(1.0, hoodPosition + HOOD_STEP);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            hoodPosition = Math.max(0.39, hoodPosition - HOOD_STEP);
        }

        turret.setShooterRPM(targetRPM);
        turret.on();
        turret.setHoodPosition(hoodPosition);
        turret.trackTarget(follower.getPose(), TARGET_POSE, 0);

        Pose pose = follower.getPose();
        double dx = TARGET_POSE.getX() - pose.getX();
        double dy = TARGET_POSE.getY() - pose.getY();
        double distance = Math.hypot(dx, dy);

        telemetry.addLine("=== Shooting Regression Tuner ===");
        telemetry.addLine("Drive: left stick | RB/LB: RPM +/- | DPAD U/D: Hood +/-");
        telemetry.addLine();
        telemetry.addData("Distance to Goal (in)", String.format(java.util.Locale.US, "%.2f", distance));
        telemetry.addLine();
        telemetry.addData("Target RPM", String.format(java.util.Locale.US, "%.0f", targetRPM));
        telemetry.addData("Measured RPM", String.format(java.util.Locale.US, "%.0f", turret.getShooterRPM()));
        telemetry.addData("Hood Position", String.format(java.util.Locale.US, "%.3f", hoodPosition));
        telemetry.addLine();
        telemetry.addData("Robot Pose", String.format(java.util.Locale.US, "(%.1f, %.1f)", pose.getX(), pose.getY()));
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.off();
    }
}
