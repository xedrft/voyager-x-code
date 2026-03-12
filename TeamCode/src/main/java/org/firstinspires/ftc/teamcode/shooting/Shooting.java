package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

/**
 * TeleOp-friendly shooting controller.
 *
 * Owns the state machine for full outtake + single-color outtake,
 * turret aiming, RPM calculation from distance, and close/far shot behavior.
 *
 * Call {@link #update(Pose)} once per loop after your pose estimate updates.
 */
public class Shooting {

    private static final int CLOSE_CAP_RPM = 2600;
    private static final double CLOSE_OUTTAKE_DELAY_MS = 300.0;
    private static final double FAR_OUTTAKE_DELAY_MS = 600.0;
    private static final double FIRST_ADVANCE_DELAY_DIVISOR = 3.0;

    private final Turret turret;
    private final KickerServo kickerServo;
    private final Spindexer spindexer;
    private final BarIntake barIntake;
    private final Pose targetPose;

    private boolean farShootingEnabled = false;

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

    public Shooting(Turret turret, KickerServo kickerServo, Spindexer spindexer, BarIntake barIntake, Pose targetPose) {
        this.turret = turret;
        this.kickerServo = kickerServo;
        this.spindexer = spindexer;
        this.barIntake = barIntake;
        this.targetPose = (targetPose == null) ? new Pose(0, 144, 0) : targetPose;
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

        turret.trackTarget(robotPose, targetPose, 0);

        double distance = Math.hypot(targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
        currentRpmTarget = rpmFromDistance(distance);

        if (!farShootingEnabled && currentRpmTarget > CLOSE_CAP_RPM) {
            currentRpmTarget = CLOSE_CAP_RPM;
        }

        turret.setShooterRPM(currentRpmTarget);
        turret.on();

        runOuttakeStateMachines();

        if (spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress) {
            spinInterval++;
            if (spinInterval > 30 && spinInterval < 50) {
                barIntake.spinOuttake();
            } else {
                barIntake.stop();
            }
        }

        if (outtakeInProgress) {
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

    public double getCurrentRpmTarget() {
        return currentRpmTarget;
    }

    /** Switches to the opposite preconfigured shooting mode. */
    public void toggleFarShootingMode() {
        farShootingEnabled = !farShootingEnabled;
    }

    private double rpmFromDistance(double distanceIn) {
        return 0.0151257 * distanceIn * distanceIn + 10.03881 * distanceIn + 1382.4428;
    }

    private double getOuttakeDelayMs() {
        return farShootingEnabled ? FAR_OUTTAKE_DELAY_MS : CLOSE_OUTTAKE_DELAY_MS;
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
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTimeMs = 0.0;

        turret.transferOn();
        kickerServo.kick();
        lastAdvanceTimeMs = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTimeMs = outtakeTimer.milliseconds();
        double outtakeDelayMs = getOuttakeDelayMs();

        if (outtakeAdvanceCount < 2) {
            double waitMs = (outtakeAdvanceCount == 0)
                    ? (outtakeDelayMs / FIRST_ADVANCE_DELAY_DIVISOR)
                    : outtakeDelayMs;

            if (currentTimeMs - lastAdvanceTimeMs >= waitMs) {
                spindexer.advanceShoot();
                outtakeAdvanceCount++;
                lastAdvanceTimeMs = currentTimeMs;
            }
        } else {
            if (currentTimeMs - lastAdvanceTimeMs >= outtakeDelayMs) {
                kickerServo.normal();
                spindexer.clearTracking();
                barIntake.spinIntake();
                spinInterval = 0;
                spindexer.setIntakeIndex(0);
                outtakeInProgress = false;
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
            if (outtakeTimer.milliseconds() > getOuttakeDelayMs()) {
                kickerServo.normal();
                spindexer.setColorAtPos('_', spindexer.getShootIndex());
                singleOuttakeInProgress = false;
                if (!spindexer.isFull()) {
                    barIntake.spinIntake();
                }
            }
        }
    }
}
