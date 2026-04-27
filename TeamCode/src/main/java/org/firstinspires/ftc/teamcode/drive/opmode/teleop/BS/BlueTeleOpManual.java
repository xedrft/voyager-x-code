package org.firstinspires.ftc.teamcode.drive.opmode.teleop.BS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.intake.IntakeFlap;
import org.firstinspires.ftc.teamcode.intake.IntakeServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Blue TeleOp Manual", group = "TeleOp")
public class BlueTeleOpManual extends OpMode {
    private Follower follower;
    private LockMode lockMode;
    private boolean isLocked = false;

    private BarIntake barIntake;
    private IntakeFlap intakeFlap;

    private Spindexer spindexer;
    private int offset_turret = 0;
    private IntakeServo intakeServo;
    private Turret turret;
    private ColorSensor colorSensor;
    private ElapsedTime loopTimer;
    private ElapsedTime outtakeTimer;
    private static final double OFFSET = Math.toRadians(180.0);
    private Pose targetPose = new Pose(0, 144, 0);

    private boolean outtakeInProgress = false;
    private boolean singleOuttakeInProgress = false;
    private boolean singleAtPosition = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;
    private static double OUTTAKE_DELAY_MS = 500;

    private int spinInterval = 0;
    private boolean goingToPosition = false;
    private static Pose GO_TO_TARGET = new Pose(18.53, 58.42, 2.67);

    // Manual RPM and hood controls
    private double manualRPM = 2500.0;
    private double manualHoodPosition = 0.6;

    private static final double RPM_ADJUST_RATE = 15.0;   // RPM per loop at full stick
    private static final double HOOD_ADJUST_RATE = 0.003; // servo units per loop at full stick

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        lockMode = new LockMode(follower);
        barIntake = new BarIntake(hardwareMap, "barIntake", false);
        intakeFlap = new IntakeFlap(hardwareMap, "intakeFlapServo");
        intakeServo = new IntakeServo(hardwareMap, "intakeServo");
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor, intakeFlap);
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", true, false);
        loopTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();

        if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
        } else {
            follower.setPose(new Pose(0, 0, 0));
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.on();
        barIntake.spinIntake();
        intakeFlap.on();
        intakeServo.intake();
        turret.transferOff();
        spindexer.setIntakeIndex(0);
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        boolean colorScanInProgress = spindexer.isAccurateColorScanInProgress();

        if (colorScanInProgress) {
            intakeFlap.off();
            intakeServo.outtake();
            barIntake.stop();
        } else if (!spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress) {
            intakeFlap.on();
            intakeServo.intake();
        } else {
            intakeFlap.off();
            intakeServo.outtake();
        }

        follower.update();

        if (isLocked && gamepad1.left_trigger > 0.5) {
            lockMode.lockPosition();
        } else {
            lockMode.unlockPosition();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false,
                    OFFSET
            );
        }

        if (gamepad1.aWasPressed()) {
            Pose cur = follower.getPose();
            PathChain goToPath = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(cur.getX(), cur.getY(), cur.getHeading()),
                            GO_TO_TARGET))
                    .setLinearHeadingInterpolation(cur.getHeading(), GO_TO_TARGET.getHeading())
                    .build();
            follower.followPath(goToPath, 0.5, false);
            goingToPosition = true;
        }
        if (goingToPosition) {
            boolean stickMoved = Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1;
            if (!follower.isBusy() || stickMoved) {
                goingToPosition = false;
                follower.setMaxPower(1.0);
                follower.startTeleopDrive();
            }
        }

        if (gamepad1.bWasPressed()) {
            GO_TO_TARGET = follower.getPose();
        }

        if (gamepad1.shareWasPressed()) {
            follower.setPose(new Pose(136.5, 7.75, Math.toRadians(180)));
            turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", true, false);
            isLocked = false;
            lockMode.unlockPosition();
        }

        if (gamepad2.yWasPressed() && !colorScanInProgress && !outtakeInProgress && !singleOuttakeInProgress) {
            spindexer.startAccurateColorScan();
            colorScanInProgress = spindexer.isAccurateColorScanInProgress();
            if (colorScanInProgress) {
                barIntake.stop();
                intakeFlap.off();
                intakeServo.outtake();
                gamepad2.rumble(200);
            }
        }

        if (!colorScanInProgress && gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (!colorScanInProgress && gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }

        if (!colorScanInProgress && gamepad1.xWasPressed()) {
            spindexer.clearTracking();
            barIntake.spinIntake();
        }

        if (!colorScanInProgress && gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            turret.on();
            startOuttakeRoutine();
        }

        turret.trackTarget(follower.getPose(), targetPose, offset_turret);

        // --- Manual RPM adjustment (gamepad2 dpad up/down) ---
        if (gamepad2.dpad_up) manualRPM = Math.min(4500.0, manualRPM + RPM_ADJUST_RATE);
        if (gamepad2.dpad_down) manualRPM = Math.max(500.0, manualRPM - RPM_ADJUST_RATE);

        // --- Manual hood adjustment (gamepad2 dpad left/right) ---
        if (gamepad2.dpad_right) manualHoodPosition = Math.min(1.0, manualHoodPosition + HOOD_ADJUST_RATE);
        if (gamepad2.dpad_left) manualHoodPosition = Math.max(0.39, manualHoodPosition - HOOD_ADJUST_RATE);
        turret.setHoodPosition(manualHoodPosition);

        turret.setShooterRPM(manualRPM);
        turret.on();

        double distance = Math.sqrt(
                (targetPose.getX() - follower.getPose().getX()) * (targetPose.getX() - follower.getPose().getX())
                + (targetPose.getY() - follower.getPose().getY()) * (targetPose.getY() - follower.getPose().getY()));

        if (!colorScanInProgress && gamepad1.leftStickButtonWasPressed()) {
            startSingleOuttake('P');
        }
        if (!colorScanInProgress && gamepad1.rightStickButtonWasPressed()) {
            startSingleOuttake('G');
        }

        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }
        if (singleOuttakeInProgress) {
            handleSingleOuttake();
        }

        if (!colorScanInProgress && spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress) {
            spindexer.setShootIndex(2);
            spinInterval++;
            barIntake.stop();
        }

        if (outtakeInProgress) {
            barIntake.stop();
        }

        spindexer.update();

        telemetry.addData("Distance to Goal (in)", String.format(java.util.Locale.US, "%.2f", distance));
        telemetry.addData("Target RPM", String.format(java.util.Locale.US, "%.0f", manualRPM));
        telemetry.addData("Hood Position", String.format(java.util.Locale.US, "%.4f", turret.getHoodPosition()));
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        intakeFlap.off();
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        if (spindexer.isFull()) {
            turret.transferOn();
        }
        isLocked = true;
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        if (outtakeAdvanceCount < 2) {
            if (currentTime - lastAdvanceTime >= (outtakeAdvanceCount == 0 ? OUTTAKE_DELAY_MS / 2 : OUTTAKE_DELAY_MS)) {
                turret.transferOn();
                spindexer.retreatShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS * 3) {
                barIntake.spinIntake();
                spindexer.clearTracking();
                turret.transferOff();
                intakeFlap.on();
                spinInterval = 0;
                spindexer.setIntakeIndex(0);
                outtakeInProgress = false;
                isLocked = false;
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
        spindexer.setShootIndex(index);
    }

    private void handleSingleOuttake() {
        if (!singleAtPosition) {
            if (spindexer.isAtTarget(5.0)) {
                singleAtPosition = true;
                outtakeTimer.reset();
                turret.transferOn();
            }
        } else {
            if (outtakeTimer.milliseconds() > OUTTAKE_DELAY_MS) {
                turret.transferOff();
                spindexer.setColorAtPos('_', spindexer.getShootIndex());
                singleOuttakeInProgress = false;
                if (spindexer.isEmpty()) {
                    barIntake.spinIntake();
                    spindexer.setIntakeIndex(0);
                }
            }
        }
    }
}
