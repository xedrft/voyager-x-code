package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Turret Regression Test", group = "TeleOp")
public class TurretRegressionTeleOp extends OpMode {

    private Follower follower;
    private static final Pose startingPose = new Pose(7.5, 7.75, Math.toRadians(0));
    private static final Pose targetPose = new Pose(0, 144, 0);

    private BarIntake barIntake;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;
    private LynxModule expansionHub;

    private ElapsedTime outtakeTimer;
    private static final double OUTTAKE_DELAY_MS = 150;
    private static final int TURRET_OFFSET_DEG = 0;

    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;
    private int spinInterval = 0;
    private boolean outtakeWaitingForReset = false;
    private double resetWaitStartTime = 0;

    private double currentRPM = 2500.0;

    private static final double OFFSET = Math.toRadians(180.0);

    @Override
    public void init() {
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.createFollower(hardwareMap);
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        ColorSensor colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);

        outtakeTimer = new ElapsedTime();

        follower.setStartingPose(startingPose);
        turret.setShooterRPM(currentRPM);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.on();
        barIntake.spinIntake();
    }

    @Override
    public void loop() {
        expansionHub.clearBulkCache();
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET);

        if (gamepad1.dpadUpWasPressed()) {
            currentRPM += 50;
        }
        if (gamepad1.dpadDownWasPressed()) {
            currentRPM -= 50;
        }
        if (gamepad1.dpadRightWasPressed()) {
            currentRPM += 10;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            currentRPM -= 10;
        }
        currentRPM = Math.max(0.0, currentRPM);

        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        }

        if (gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }

        if (gamepad1.xWasPressed()) {
            spindexer.clearTracking();
            barIntake.spinIntake();
        }

        Pose robotPose = follower.getPose();
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double distance = Math.hypot(dx, dy);

        turret.trackTarget(robotPose, targetPose, TURRET_OFFSET_DEG);
        turret.setShooterRPM(currentRPM);
        turret.on();

        if (spindexer.isFull() && !outtakeInProgress) {
            spindexer.setShootIndex(1);
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
        spindexer.update();

        if (gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            startOuttakeRoutine();
        }

        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }

        telemetry.addData("Distance to Goal (in)", String.format(java.util.Locale.US, "%.2f", distance));
        telemetry.addData("Target RPM", String.format(java.util.Locale.US, "%.1f", currentRPM));
        telemetry.addData("Actual RPM", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM()));
        telemetry.addData("Data Point", String.format(java.util.Locale.US, "(%.2f, %.1f)", distance, currentRPM));
        telemetry.addData("Robot Pose", String.format(java.util.Locale.US, "(%.2f, %.2f, %.2f)", robotPose.getX(), robotPose.getY(), robotPose.getHeading()));
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.addData("Controls", "Dpad U/D: +/-50 RPM, Dpad L/R: +/-10 RPM");
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeWaitingForReset = false;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        turret.transferOn();
        kickerServo.kick();
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        if (outtakeWaitingForReset) {
            if (currentTime - resetWaitStartTime >= 300) {
                barIntake.spinIntake();
                kickerServo.normal();
                spindexer.clearTracking();
                spinInterval = 0;
                spindexer.setIntakeIndex(0);
                outtakeInProgress = false;
                outtakeWaitingForReset = false;
            }
            return;
        }

        if (outtakeAdvanceCount < 2) {
            if (currentTime - lastAdvanceTime >= (outtakeAdvanceCount == 0 ? OUTTAKE_DELAY_MS / 3.0 : OUTTAKE_DELAY_MS)) {
                spindexer.advanceShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                outtakeWaitingForReset = true;
                resetWaitStartTime = currentTime;
            }
        }
    }
}
