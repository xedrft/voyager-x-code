package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Shooter Velocity Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    // Target control
    private double targetRPM = 2500.0;
    private double rpmStep = 50.0;
    private double gainStep = 0.00001;

    // Auto-step
    private boolean autoStep = false;
    private double rpmA = 2000.0;
    private double rpmB = 3000.0;
    private double autoPeriodS = 2.0;
    private final ElapsedTime autoTimer = new ElapsedTime();

    private boolean shooterOn = false;

    private enum Param { KP, KI, KD, KV, KS }
    private Param selected = Param.KP;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);
        turret.setShooterRPM(targetRPM);

        autoTimer.reset();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                shooterOn = !shooterOn;
            }

            if (gamepad1.bWasPressed()) {
                autoStep = !autoStep;
                autoTimer.reset();
            }

            if (gamepad1.xWasPressed()) {
                gainStep = nextGainStep(gainStep);
            }

            if (gamepad1.yWasPressed()) {
                selected = next(selected);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                rpmStep = Math.max(1.0, rpmStep / 2.0);
            }
            if (gamepad1.dpadRightWasPressed()) {
                rpmStep = Math.min(2000.0, rpmStep * 2.0);
            }

            if (gamepad1.dpadUpWasPressed()) {
                targetRPM += rpmStep;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetRPM -= rpmStep;
            }
            targetRPM = Math.max(0.0, targetRPM);
            turret.setShooterRPM(targetRPM);

            if (gamepad1.rightBumperWasPressed()) {
                adjustSelected(+gainStep);
            }
            if (gamepad1.leftBumperWasPressed()) {
                adjustSelected(-gainStep);
            }

            if (autoStep && autoTimer.seconds() >= autoPeriodS) {
                autoTimer.reset();
                targetRPM = (Math.abs(targetRPM - rpmA) < 1e-6) ? rpmB : rpmA;
                turret.setShooterRPM(targetRPM);
            }

            if (shooterOn) {
                turret.on();
            } else {
                turret.off();
            }

            telemetry.addLine("=== Shooter Velocity Tuner ===");
            telemetry.addData("Shooter (A)", shooterOn ? "ON" : "OFF");
            telemetry.addData("Auto-step (B)", autoStep);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("RPM step (Dpad L/R)", "%.1f", rpmStep);

            telemetry.addLine();
            telemetry.addData("Measured RPM", "%.1f", turret.getShooterRPM());
            telemetry.addData("Error RPM", "%.1f", turret.getShooterRpmError());
            telemetry.addData("Power Command", "%.3f", turret.getShooterPowerCommand());

            telemetry.addLine();
            telemetry.addData("Selected (Y)", selected);
            telemetry.addData("Gain Step (X)", "%.6f", gainStep);
            telemetry.addData("KP", "%.6f", Turret.SHOOTER_KP);
            telemetry.addData("KI", "%.6f", Turret.SHOOTER_KI);
            telemetry.addData("KD", "%.6f", Turret.SHOOTER_KD);
            telemetry.addData("KV", "%.6f", Turret.SHOOTER_KV);
            telemetry.addData("KS", "%.4f", Turret.SHOOTER_KS);

            telemetry.addLine();
            telemetry.addLine("RB = increase selected");
            telemetry.addLine("LB = decrease selected");
            telemetry.addLine("X = cycle gain step");
            telemetry.update();
        }

        turret.off();
    }

    private void adjustSelected(double delta) {
        switch (selected) {
            case KP:
                Turret.SHOOTER_KP = clampNonNeg(Turret.SHOOTER_KP + delta);
                break;
            case KI:
                Turret.SHOOTER_KI = clampNonNeg(Turret.SHOOTER_KI + delta);
                break;
            case KD:
                Turret.SHOOTER_KD = clampNonNeg(Turret.SHOOTER_KD + delta);
                break;
            case KV:
                Turret.SHOOTER_KV = clampNonNeg(Turret.SHOOTER_KV + delta);
                break;
            case KS:
                Turret.SHOOTER_KS = clampNonNeg(Turret.SHOOTER_KS + delta);
                break;
        }
    }

    private static double nextGainStep(double current) {
        if (current < 0.00002) return 0.00002;
        if (current < 0.00005) return 0.00005;
        if (current < 0.0001) return 0.0001;
        if (current < 0.0002) return 0.0002;
        if (current < 0.0005) return 0.0005;
        if (current < 0.001) return 0.001;
        if (current < 0.002) return 0.002;
        if (current < 0.005) return 0.005;
        if (current < 0.01) return 0.01;
        if (current < 0.02) return 0.02;
        if (current < 0.05) return 0.05;
        if (current < 0.1) return 0.1;
        if (current < 0.2) return 0.2;
        if (current < 0.5) return 0.5;
        if (current < 1.0) return 1.0;
        if (current < 5.0) return 5.0;
        if (current < 10.0) return 10.0;
        return 0.00001;
    }

    private static Param next(Param p) {
        switch (p) {
            case KP:
                return Param.KI;
            case KI:
                return Param.KD;
            case KD:
                return Param.KV;
            case KV:
                return Param.KS;
            default:
                return Param.KP;
        }
    }

    private static double clampNonNeg(double v) {
        return Math.max(0.0, v);
    }
}
