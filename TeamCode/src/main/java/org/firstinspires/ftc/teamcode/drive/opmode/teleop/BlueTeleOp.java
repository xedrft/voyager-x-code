package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Shooting;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {
    private Follower follower;

    private BarIntake barIntake;
    private Servo ledHeadlight;
    private Servo ledHeadlight2;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;
    private ElapsedTime loopTimer;
    private LynxModule expansionHub;
    private GoBildaPinpointDriver pinpoint;

    private Shooting shooting;

    private static final double OFFSET = Math.toRadians(180.0);
    private final Pose targetPose = new Pose(0, 144, 0);

    private boolean lastFull = false;

    private boolean goingToPosition = false;
    private static Pose GO_TO_TARGET = new Pose(18.53, 58.42, 2.67);

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
        loopTimer = new ElapsedTime();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        ledHeadlight = hardwareMap.get(Servo.class, "ledLight");
        ledHeadlight2 = hardwareMap.get(Servo.class, "ledLight2");
        ledHeadlight.setPosition(0.0);
        ledHeadlight2.setPosition(0.0);
        pinpoint.recalibrateIMU();

        if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
        } else {
            follower.setPose(new Pose(0, 0, 0));
        }

        shooting = createShooting();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        barIntake.spinIntake();
        shooting.onStart();
    }

    @Override
    public void loop() {
        expansionHub.clearBulkCache();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET
        );

        if (gamepad1.aWasPressed()) {
            Pose cur = follower.getPose();
            PathChain goToPath = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(cur.getX(), cur.getY(), cur.getHeading()), GO_TO_TARGET))
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
            follower.setPose(new Pose(136.5, 7.75, Math.toRadians(0)));
            turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);
            shooting = createShooting();
            shooting.onStart();
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

        if (gamepad1.dpadDownWasPressed()) {
            shooting.toggleFarShootingMode();
            gamepad1.rumble(200);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            gamepad1.rumble(200);
        }

        if (gamepad1.left_trigger > 0.5 && !shooting.isOuttakeInProgress()) {
            shooting.requestFullOuttake();
        }
        if (gamepad1.leftStickButtonWasPressed()) {
            shooting.requestSingleOuttake('P');
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            shooting.requestSingleOuttake('G');
        }

        shooting.update(follower.getPose());

        if (spindexer.isFull()) {
            ledHeadlight.setPosition(1.0);
            ledHeadlight2.setPosition(1.0);
            if (!lastFull) gamepad2.rumble(2000);
            lastFull = true;
        } else {
            ledHeadlight.setPosition(0.0);
            ledHeadlight2.setPosition(0.0);
            lastFull = false;
        }

        spindexer.update();

         telemetry.addData("Spindexer Index", spindexer.getIntakeIndex());
         telemetry.addData("Robot Pose: ", "(" + follower.getPose().getX() + ", " + follower.getPose().getY() + ", " + follower.getPose().getHeading() + ")");
         telemetry.addData("Adaptive Tolerance", String.format(java.util.Locale.US, "%.2f", spindexer.getLastAdaptiveTol()));
         telemetry.addData("Turret RPM Error", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM() - turret.getSetShooterRPM()));
         telemetry.addData("Target RPM", shooting.getCurrentRpmTarget());
         telemetry.addData("Radial Vel (ips)", shooting.getRadialVelocityIps());
         telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
         char[] filled = spindexer.getFilled();
         telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
         telemetry.update();
    }

    private Shooting createShooting() {
        Shooting.Config config = new Shooting.Config();
        config.targetPose = targetPose;
        return new Shooting(turret, kickerServo, spindexer, barIntake, config);
    }
}
