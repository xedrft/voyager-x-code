package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

/**
 * TeleOp-friendly shooting controller.
 *
 * Owns the state machine for full outtake + single-color outtake,
 * turret aiming, RPM calculation from distance, and velocity-based RPM compensation.
 *
 * Call {@link #update(Pose)} once per loop after your pose estimate updates.
 */
public class Shooting {

    public static final class Config {
        /** RPM change per (inch/sec) of radial velocity. */
        public double rpmPerIps = 4.0;
        /** Ignore tiny velocity noise. */
        public double radialVelDeadbandIps = 1.0;
        /** Clamp velocity compensation so it can't run away. */
        public double maxRpmVelComp = 250.0;

        /** Default outtake delay (ms). TeleOp may overwrite dynamically. */
        public double outtakeDelayMs = 300.0;

        /** If enabled, clamp RPM to closeCapRpm. */
        public boolean rpmCapEnabled = true;
        /** RPM cap value. */
        public int closeCapRpm = 2600;

        /** Degrees added inside {@link Turret#trackTarget(Pose, Pose, int)}. */
        public int turretOffsetDeg = 0;

        /** If true, forces shooter index to 1 whenever spindexer is full and idle. */
        public boolean forceShootIndexOneWhenFullIdle = true;

        /** Divisor for the first advance during a full-outtake routine. */
        public double firstAdvanceDelayDivisor = 3.0;

        /** If true, apply the TeleOp full-and-idle pulse behavior. */
        public boolean manageIdleFullPulse = true;

        /** If true, stop the bar intake while a full outtake is in progress. */
        public boolean stopBarIntakeDuringOuttake = true;

        /** If true, spin the bar intake inward when a full outtake finishes. */
        public boolean spinBarIntakeOnOuttakeFinish = true;


        /** Target pose for aiming / distance calculations. */
        public Pose targetPose = new Pose(0, 144, 0);

        /** Number of spindexer advances to perform during a full outtake. */
        public int advancesPerOuttake = 2;

        /** If true, perform one advance immediately when full outtake starts. */
        public boolean immediateAdvanceOnOuttakeStart = false;

        /** If true use advanceShoot(); otherwise use advanceIntake(). */
        public boolean useShootAdvanceMethod = true;

        /** If true, turn transfer off when a full outtake finishes. */
        public boolean transferOffOnOuttakeFinish = false;
    }

    private final Turret turret;
    private final KickerServo kickerServo;
    private final Spindexer spindexer;
    private final BarIntake barIntake;

    private final Config config;

    // Outtake routine state
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private boolean singleOuttakeInProgress = false;
    private boolean singleAtPosition = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTimeMs = 0.0;

    // Full idle pulse
    private int spinInterval = 0;

    // RPM state
    private double currentRpmTarget = 2500.0;

    // velocity estimation
    private Pose lastPose = null;
    private double lastPoseTimeSec = 0.0;
    private double radialVelocityIps = 0.0;

    private final ElapsedTime monotonicTimer = new ElapsedTime();

    // just finished an outtake?
    private boolean outtakeJustFinished = false;

    public Shooting(Turret turret, KickerServo kickerServo, Spindexer spindexer, BarIntake barIntake, Config config) {
        this.turret = turret;
        this.kickerServo = kickerServo;
        this.spindexer = spindexer;
        this.barIntake = barIntake;
        this.config = (config == null) ? new Config() : config;

        monotonicTimer.reset();
        lastPoseTimeSec = monotonicTimer.seconds();
    }

    /** Call from OpMode.start() for shooter-related startup actions. */
    public void onStart() {
        turret.on();
        turret.transferOn();
    }

    /**
     * Call once per loop after pose updates.
     *
     * @param robotPose current estimate of robot pose
     */
    public void update(Pose robotPose) {
        if (robotPose == null) return;

        estimateRadialVelocity(robotPose);
        turret.trackTarget(robotPose, config.targetPose, config.turretOffsetDeg);

        double distance = Math.hypot(config.targetPose.getX() - robotPose.getX(), config.targetPose.getY() - robotPose.getY());
        currentRpmTarget = rpmFromDistance(distance);

        double velComp = config.rpmPerIps * radialVelocityIps;
        velComp = Math.max(-config.maxRpmVelComp, Math.min(config.maxRpmVelComp, velComp));
        currentRpmTarget += velComp;

        if (config.rpmCapEnabled && currentRpmTarget > config.closeCapRpm) {
            currentRpmTarget = config.closeCapRpm;
        }

        turret.setShooterRPM(currentRpmTarget);
        turret.on();

        runOuttakeStateMachines();

        if (config.manageIdleFullPulse && spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress) {
            if (config.forceShootIndexOneWhenFullIdle) {
                spindexer.setShootIndex(1);
            }
            spinInterval++;
            if (spinInterval > 30 && spinInterval < 50) {
                barIntake.spinOuttake();
            } else {
                barIntake.stop();
            }
        }

        if (config.stopBarIntakeDuringOuttake && outtakeInProgress) {
            barIntake.stop();
        }
    }

    /** Auto-friendly fixed shot update: keep RPM + turret angle ready and run any active outtake routine. */
    public void updateFixedShot(double shooterRpm, double targetAngleDeg) {
        currentRpmTarget = shooterRpm;
        turret.setShooterRPM(shooterRpm);
        turret.on();
        turret.goToPosition(targetAngleDeg);

        runOuttakeStateMachines();

        if (config.stopBarIntakeDuringOuttake && outtakeInProgress) {
            barIntake.stop();
        }
    }

    /** Call when the driver triggers a full outtake. */
    public void requestFullOuttake() {
        if (outtakeInProgress) return;
        startOuttakeRoutine();
    }

    /** Call when the driver triggers a single-color outtake. */
    public void requestSingleOuttake(char color) {
        if (singleOuttakeInProgress) return;
        startSingleOuttake(color);
    }

    public boolean isOuttakeInProgress() {
        return outtakeInProgress;
    }

    public boolean isAnyOuttakeBusy() {
        return outtakeInProgress || singleOuttakeInProgress;
    }

    /** TeleOp can use this to engage its drive LockMode while shooting. */
    public boolean wantsDriveLock() {
        return outtakeInProgress;
    }

    public double getCurrentRpmTarget() {
        return currentRpmTarget;
    }

    public double getRadialVelocityIps() {
        return radialVelocityIps;
    }

    public void setTurretOffsetDeg(int offsetDeg) {
        config.turretOffsetDeg = offsetDeg;
    }

    public void setRpmCapEnabled(boolean enabled) {
        config.rpmCapEnabled = enabled;
    }

    public void setOuttakeDelayMs(double delayMs) {
        config.outtakeDelayMs = delayMs;
    }

    public void setCloseCapRpm(int closeCapRpm) {
        config.closeCapRpm = closeCapRpm;
    }

    public void setTargetPose(Pose targetPose) {
        if (targetPose != null) config.targetPose = targetPose;
    }

    public void onFieldReset(Pose poseAfterReset, double nowSec) {
        lastPose = poseAfterReset;
        lastPoseTimeSec = nowSec;
        radialVelocityIps = 0.0;
        spinInterval = 0;
        outtakeInProgress = false;
        singleOuttakeInProgress = false;
        singleAtPosition = false;
        outtakeAdvanceCount = 0;
        lastAdvanceTimeMs = 0.0;
        kickerServo.normal();
    }

    private void estimateRadialVelocity(Pose currentPose) {
        double nowSec = secondsNow();
        double dt = nowSec - lastPoseTimeSec;

        if (lastPose != null && dt > 1e-3) {
            double dx = currentPose.getX() - lastPose.getX();
            double dy = currentPose.getY() - lastPose.getY();

            double vx = dx / dt;
            double vy = dy / dt;

            double toTargetX = config.targetPose.getX() - currentPose.getX();
            double toTargetY = config.targetPose.getY() - currentPose.getY();
            double distToTarget = Math.hypot(toTargetX, toTargetY);

            if (distToTarget > 1e-6) {
                double ux = toTargetX / distToTarget;
                double uy = toTargetY / distToTarget;

                // Positive means moving toward target; negative means moving away.
                double closingSpeedIps = vx * ux + vy * uy;

                // We want + = away, - = toward.
                radialVelocityIps = -closingSpeedIps;

                if (Math.abs(radialVelocityIps) < config.radialVelDeadbandIps) {
                    radialVelocityIps = 0.0;
                }
            } else {
                radialVelocityIps = 0.0;
            }
        }

        lastPose = currentPose;
        lastPoseTimeSec = nowSec;
    }

    private double rpmFromDistance(double distanceIn) {
        return 0.0151257 * distanceIn * distanceIn + 10.03881 * distanceIn + 1382.4428;
    }

    private double secondsNow() {
        return monotonicTimer.seconds();
    }

    private void runOuttakeStateMachines() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }
        if (singleOuttakeInProgress) {
            handleSingleOuttake();
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeJustFinished = false;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTimeMs = 0.0;

        turret.transferOn();
        kickerServo.kick();
        lastAdvanceTimeMs = outtakeTimer.milliseconds();

        if (config.immediateAdvanceOnOuttakeStart) {
            performConfiguredAdvance();
            outtakeAdvanceCount++;
            lastAdvanceTimeMs = outtakeTimer.milliseconds();
        }
    }

    private void handleOuttakeRoutine() {
        double currentTimeMs = outtakeTimer.milliseconds();

        if (outtakeAdvanceCount < config.advancesPerOuttake) {
            double waitMs = (outtakeAdvanceCount == 0)
                    ? (config.outtakeDelayMs / Math.max(1.0, config.firstAdvanceDelayDivisor))
                    : config.outtakeDelayMs;

            if (currentTimeMs - lastAdvanceTimeMs >= waitMs) {
                performConfiguredAdvance();
                outtakeAdvanceCount++;
                lastAdvanceTimeMs = currentTimeMs;
            }
        } else {
            if (currentTimeMs - lastAdvanceTimeMs >= config.outtakeDelayMs) {
                kickerServo.normal();
                spindexer.clearTracking();
                if (config.transferOffOnOuttakeFinish) {
                    turret.transferOff();
                }
                if (config.spinBarIntakeOnOuttakeFinish) {
                    barIntake.spinIntake();
                }
                spinInterval = 0;
                spindexer.setIntakeIndex(0);
                outtakeInProgress = false;
                outtakeJustFinished = true;
            }
        }
    }

    private void startSingleOuttake(char color) {
        int index = -1;
        char[] filled = spindexer.getFilled();
        for (int i = 0; i < 3; i++) {
            if (filled[i] == color) {
                index = i;
                break;
            }
        }
        if (index == -1) return;

        singleOuttakeInProgress = true;
        singleAtPosition = false;

        turret.transferOn();
        spindexer.setShootIndex(index);
    }

    private void handleSingleOuttake() {
        if (!singleAtPosition) {
            if (spindexer.isAtTarget(5.0)) {
                singleAtPosition = true;
                outtakeTimer.reset();
                kickerServo.kick();
            }
        } else {
            if (outtakeTimer.milliseconds() > config.outtakeDelayMs) {
                kickerServo.normal();
                spindexer.setColorAtPos('_', spindexer.getShootIndex());
                singleOuttakeInProgress = false;
                if (!spindexer.isFull()) {
                    barIntake.spinIntake();
                }
            }
        }
    }

    /** Call this to see if an outtake just finished (for autos to consume). */
    public boolean consumeOuttakeFinished() {
        boolean finished = outtakeJustFinished;
        outtakeJustFinished = false;
        return finished;
    }

    public void setFirstAdvanceDelayDivisor(double divisor) {
        config.firstAdvanceDelayDivisor = Math.max(1.0, divisor);
    }

    private void performConfiguredAdvance() {
        if (config.useShootAdvanceMethod) {
            spindexer.advanceShoot();
        } else {
            spindexer.advanceIntake();
        }
    }
}
