package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private final DcMotorImplEx shooterMotor;

    private static final double COUNTS_PER_WHEEL_REV = 28.0;

    private final DcMotorImplEx transferMotor;
    private final Servo turretServo;
    private final AnalogInput turretEncoder;

    private double shooterRPM = 2500.0;
    private double farRPM = 3000.0;
    private double transferPower = 1;

    private static final double ANALOG_MAX_VOLTAGE = 3.3;
    private double lastCommandedAngle = 0.0;

    // PID Coefficients for trackTarget
    public static double Kp = 0.02;
    public static double Ki = 0.0;
    public static double Kd = 0.001;
    public static double kStatic = 0.1;

    // Shooter RPM controller tuning
    public static double SHOOTER_KP = 0.019;
    public static double SHOOTER_KI = 0.00;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KS = 0.03;
    // 6000 RPM motor: leave room for kS so feedforward doesn't start too high.
    public static double SHOOTER_KV = (1.0 - SHOOTER_KS) / 5500.0;
    public static double SHOOTER_INTEGRAL_MAX = 2500.0;

    private final ElapsedTime shooterPidTimer = new ElapsedTime();
    private boolean shooterPidInitialized = false;
    private double shooterIntegral = 0.0;
    private double lastShooterError = 0.0;
    private double lastShooterPower = 0.0;

    public Turret(HardwareMap hardwareMap, String shooterName, String turretName, String turretEncoderName,
                  String transferName, boolean shooterReversed, boolean transferReversed) {
        shooterMotor = hardwareMap.get(DcMotorImplEx.class, shooterName);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (shooterReversed) {
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretServo = hardwareMap.get(Servo.class, turretName);

        transferMotor = hardwareMap.get(DcMotorImplEx.class, transferName);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (transferReversed) {
            transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        turretEncoder = hardwareMap.get(AnalogInput.class, turretEncoderName);
        resetShooterPid();
    }

    public void on() {
        runShooterPid(shooterRPM);
    }

    public void onFar() {
        runShooterPid(farRPM);
    }

    public void off() {
        resetShooterPid();
        shooterMotor.setPower(0);
    }

    public void transferOn() {
        transferMotor.setPower(transferPower);
    }

    public void transferOff() {
        transferMotor.setPower(1);
    }

    public void setShooterRPM(double RPM) {
        double clampedRpm = Math.max(0.0, RPM);
        if (Math.abs(clampedRpm - shooterRPM) > 100.0) {
            resetShooterPid();
        }
        this.shooterRPM = clampedRpm;
    }

    public double getShooterRPM() {
        return shooterMotor.getVelocity() * 60.0 / COUNTS_PER_WHEEL_REV;
    }

    public double getSetShooterRPM() {
        return this.shooterRPM;
    }

    public double getShooterPowerCommand() {
        return lastShooterPower;
    }

    public double getShooterRpmError() {
        return shooterRPM - getShooterRPM();
    }

    public double getTurretVoltage() {
        return turretEncoder.getVoltage();
    }

    public void trackTarget(Pose robotPose, Pose targetPose, int offset) {
        double x = targetPose.getX() - robotPose.getX();
        double y = targetPose.getY() - robotPose.getY();
        double targetAngle = Math.atan2(y, x);
        targetAngle += Math.toRadians(offset);

        double robotHeading = robotPose.getHeading();
        double desiredRelativeAngle = Math.toDegrees(targetAngle - robotHeading);
        desiredRelativeAngle = normalizeAngle(desiredRelativeAngle);
        desiredRelativeAngle = Math.max(80, Math.min(280, desiredRelativeAngle));

        goToPosition(desiredRelativeAngle);
    }

    public void goToPosition(double targetAngleDegrees) {
        lastCommandedAngle = targetAngleDegrees;
        double mapped = targetAngleDegrees * (255.0 / 360.0);
        turretServo.setPosition(1 - (mapped / 255.0));
    }

    public double getEncoderAngle() {
        double v = turretEncoder.getVoltage();
        if (v < 0) v = 0;
        if (v > ANALOG_MAX_VOLTAGE) v = ANALOG_MAX_VOLTAGE;
        return (v / ANALOG_MAX_VOLTAGE) * 360.0;
    }

    public double getEncoderOffset() {
        double diff = lastCommandedAngle - getEncoderAngle();
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    public double getCurrentError() {
        return getEncoderOffset();
    }

    public void turretServo(double position) {
        turretServo.setPosition(position);
    }

    private void runShooterPid(double targetRpm) {
        if (targetRpm <= 0.0) {
            off();
            return;
        }

        double measuredRpm = getShooterRPM();
        double error = targetRpm - measuredRpm;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();

        if (!shooterPidInitialized || dt <= 1e-4 || dt > 0.25) {
            shooterPidInitialized = true;
            shooterIntegral = 0.0;
            lastShooterError = error;
            dt = 0.0;
        }

        double integralCandidate = shooterIntegral + error * dt;
        integralCandidate = clamp(integralCandidate, -SHOOTER_INTEGRAL_MAX, SHOOTER_INTEGRAL_MAX);

        double derivative = (dt > 0.0) ? ((error - lastShooterError) / dt) : 0.0;
        double baseOutput = computeFeedforward(targetRpm) + SHOOTER_KP * error + SHOOTER_KD * derivative;
        double outputWithCandidateIntegral = baseOutput + SHOOTER_KI * integralCandidate;
        double clampedCandidateOutput = clamp(outputWithCandidateIntegral, 0.0, 1.0);

        boolean allowIntegral = clampedCandidateOutput == outputWithCandidateIntegral
                || Math.signum(error) != Math.signum(integralCandidate);
        if (allowIntegral) {
            shooterIntegral = integralCandidate;
        }

        double output = baseOutput + SHOOTER_KI * shooterIntegral;
        output = clamp(output, 0.0, 1.0);

        shooterMotor.setPower(output);
        lastShooterPower = output;
        lastShooterError = error;
    }

    private double computeFeedforward(double targetRpm) {
        if (targetRpm <= 0.0) return 0.0;
        return SHOOTER_KS + SHOOTER_KV * targetRpm;
    }

    private void resetShooterPid() {
        shooterPidTimer.reset();
        shooterPidInitialized = false;
        shooterIntegral = 0.0;
        lastShooterError = 0.0;
        lastShooterPower = 0.0;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void stop() {
        off();
    }
}
