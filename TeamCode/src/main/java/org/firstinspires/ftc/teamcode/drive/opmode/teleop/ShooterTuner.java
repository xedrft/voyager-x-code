package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter Velocity Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";
    private static final double COUNTS_PER_REV = 28.0;

    // Target control
    private double targetRPM = 2500.0;
    private double rpmStep = 50.0;

    // Auto-step
    private boolean autoStep = false;
    private double rpmA = 2000.0;
    private double rpmB = 3000.0;
    private double autoPeriodS = 2.0;
    private final ElapsedTime autoTimer = new ElapsedTime();

    // Velocity PIDF coefficients
    private double kP, kI, kD, kF;

    private boolean shooterOn = false;

    private enum Param { P, I, D, F }
    private Param selected = Param.P;

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients coeffs = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
        kF = coeffs.f;

        autoTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            // Toggle shooter
            if (gamepad1.aWasPressed()) {
                shooterOn = !shooterOn;
            }

            // Toggle auto-step
            if (gamepad1.bWasPressed()) {
                autoStep = !autoStep;
                autoTimer.reset();
            }

            // Cycle selected PIDF parameter
            if (gamepad1.yWasPressed()) {
                selected = next(selected);
            }

            // Change RPM step size
            if (gamepad1.dpadLeftWasPressed()) {
                rpmStep = Math.max(1.0, rpmStep / 2.0);
            }
            if (gamepad1.dpadRightWasPressed()) {
                rpmStep = Math.min(2000.0, rpmStep * 2.0);
            }

            // Change target RPM
            if (gamepad1.dpadUpWasPressed()) {
                targetRPM += rpmStep;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetRPM -= rpmStep;
            }
            targetRPM = Math.max(0.0, targetRPM);

            // Edit selected coefficient
            if (gamepad1.rightBumperWasPressed()) {
                adjustSelected(+getStepFor(selected));
            }
            if (gamepad1.leftBumperWasPressed()) {
                adjustSelected(-getStepFor(selected));
            }

            // Auto-step target
            if (autoStep && autoTimer.seconds() >= autoPeriodS) {
                autoTimer.reset();
                targetRPM = (Math.abs(targetRPM - rpmA) < 1e-6) ? rpmB : rpmA;
            }

            // Apply coefficients
            shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            // Command shooter
            if (shooterOn) {
                double targetTPS = targetRPM * COUNTS_PER_REV / 60.0;
                shooter.setVelocity(targetTPS);
            } else {
                shooter.setPower(0.0);
            }

            // Telemetry
            double measuredTPS = shooter.getVelocity();
            double measuredRPM = measuredTPS * 60.0 / COUNTS_PER_REV;
            double errRPM = targetRPM - measuredRPM;

            telemetry.addLine("=== Shooter Velocity Tuner ===");
            telemetry.addData("Shooter (A)", shooterOn ? "ON" : "OFF");
            telemetry.addData("Auto-step (B)", autoStep);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("RPM step (Dpad L/R)", "%.1f", rpmStep);

            telemetry.addLine();
            telemetry.addData("Measured RPM", "%.1f", measuredRPM);
            telemetry.addData("Error RPM", "%.1f", errRPM);
            telemetry.addData("Velocity (ticks/s)", "%.1f", measuredTPS);

            telemetry.addLine();
            telemetry.addData("Selected (Y)", selected);
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.4f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("kF", "%.4f", kF);

            telemetry.addLine();
            telemetry.addLine("RB = increase selected");
            telemetry.addLine("LB = decrease selected");

            telemetry.update();
        }

        shooter.setPower(0.0);
    }

    private void adjustSelected(double delta) {
        switch (selected) {
            case P:
                kP = clampNonNeg(kP + delta);
                break;
            case I:
                kI = clampNonNeg(kI + delta);
                break;
            case D:
                kD = clampNonNeg(kD + delta);
                break;
            case F:
                kF = clampNonNeg(kF + delta);
                break;
        }
    }

    private double getStepFor(Param p) {
        switch (p) {
            case P: return 0.1;
            case I: return 0.1;
            case D: return 0.1;
            case F: return 0.1;
            default: return 1.0;
        }
    }

    private static Param next(Param p) {
        switch (p) {
            case P: return Param.I;
            case I: return Param.D;
            case D: return Param.F;
            default: return Param.P;
        }
    }

    private static double clampNonNeg(double v) {
        return Math.max(0.0, v);
    }
}


//f 13.0305
//p