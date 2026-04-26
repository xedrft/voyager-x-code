package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.intake.IntakeFlap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@Autonomous(name = "Red Far Playoff", group = "Autonomous")
@Configurable
public class RedFarPlayoff extends OpMode {

    // -------------------- Panels + Pedro --------------------
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // -------------------- Subsystems --------------------
    private BarIntake barIntake;
    private IntakeFlap intakeFlap;
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    private Turret turret;
    private boolean spitInit = false;
    double  currentRPM;


    // -------------------- Timers --------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime spitTimer = new ElapsedTime();

    // -------------------- State machine --------------------
    private int pathState = 0;
    Pose targetPose = new Pose(132, 132, 0); // Fixed Red Target
    private int lastState = -1;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
        }
        pathState = s;
    }

    // -------------------- Config --------------------
    public static double OUTTAKE_DELAY_MS = 450;
    public static final int FIXED_RPM = 3935;
    private double targetAngle = 77.94;
    private int shotCount = 0;

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Starting pose from path: (100, 9, 0)
        Pose startPose = new Pose(100.000, 9.000, Math.toRadians(0));
        follower.setStartingPose(startPose);

        // Subsystems (mimicking RedCloseRackAuto)
        barIntake = new BarIntake(hardwareMap, "barIntake", false);
        intakeFlap = new IntakeFlap(hardwareMap, "intakeFlapServo");
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(
                hardwareMap,
                "spindexerMotor",
                "spindexerAnalog",
                "distanceSensor",
                colorSensor,
                intakeFlap
        );
        turret = new Turret(
                hardwareMap,
                "shooter",
                "turret",
                "turretEncoder",
                "transferMotor",
                "hoodServo",
                true,
                false
        );
        spindexer.filled = new char[]{'X', 'X', 'X'};

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        matchTimer.reset();
        stateTimer.reset();
        outtakeInProgress = false;
        shotCount = 0;

        turret.on();
        turret.transferOff();
        intakeFlap.on();
        spindexer.setShootIndex(1);
        barIntake.spinIntake();

        setState(0);
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        // Subsystem updates
        turret.trackTarget(follower.getPose(), targetPose, 0);
        double distance = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
        currentRPM = 17.1 * distance + 1700;
        turret.setShooterRPM(currentRPM);
        turret.setHoodPosition(0.58);
        turret.on();

        if (spindexer.isFull() && !outtakeInProgress) {
            if (!spitInit) {
                spitTimer.reset();
                spitInit = true;
            }
            spindexer.goToOuttakePosition();
            double spitElapsed = spitTimer.milliseconds();
            if (spitElapsed > 125 && spitElapsed < 225) {
                barIntake.spinOuttake();
            }
            else if (spitElapsed >= 225) {
                spindexer.setShootIndex(2);
                barIntake.stop();
            }
            else {
                barIntake.stop();
            }
        } else {
            spitInit = false;
        }

        spindexer.update();

        // Spit logic from RedCloseRackAuto
        if (spindexer.isFull() && !outtakeInProgress) {
            // Simplification of spit logic for auto if needed, or keep exactly same
            spindexer.goToOuttakePosition();
            // In the RedCloseRackAuto there was a timer based spit, omitted here for brevity or can be added
        }

        autonomousUpdate();
        PoseStorage.currentPose = currentPose;

        // Telemetry
        panelsTelemetry.debug("Match Time", matchTimer.seconds());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", currentPose.getX());
        panelsTelemetry.debug("Y", currentPose.getY());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.update(telemetry);
    }

    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {
            case 0: // Wait 3 seconds for spin up
                if (stateTimer.seconds() > 3.0) {
                    startOuttakeRoutine();
                    setState(1);
                }
                break;

            case 1: // Wait for preset shot to finish
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup1);
                    setState(2);
                }
                break;

            case 2: // Arrive at Pickup1
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    setState(3);
                }
                break;

            case 3: // Wait 0.5s for pickup
                if (waitTimer.seconds() > 0.5) {
                    follower.followPath(paths.Shoot1);
                    setState(4);
                }
                break;

            case 4: // Arrive at Shoot1
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(5);
                }
                break;

            case 5: // After Shoot1 outtake, check loop exit
                if (!outtakeInProgress) {
                    if (matchTimer.seconds() > 27.0) {
                        follower.followPath(paths.Park);
                        setState(10);
                    } else {
                        follower.followPath(paths.Pickup2);
                        setState(6);
                    }
                }
                break;

            case 6: // Arrive at Pickup2
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    setState(7);
                }
                break;

            case 7: // Wait 0.5s for pickup
                if (waitTimer.seconds() > 0.5) {
                    follower.followPath(paths.Shoot2);
                    setState(8);
                }
                break;

            case 8: // Arrive at Shoot2
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(9);
                }
                break;

            case 9: // After Shoot2 outtake, loop back to check time
                if (!outtakeInProgress) {
                    setState(5);
                }
                break;

            case 10: // Done
                if (!follower.isBusy()) {
                    setState(11);
                }
                break;
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        intakeFlap.off();
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = outtakeTimer.milliseconds();
        turret.transferOn();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();
        if (outtakeAdvanceCount < 2) {
            if (currentTime - lastAdvanceTime >= (outtakeAdvanceCount == 0 ? OUTTAKE_DELAY_MS / 1.5 : OUTTAKE_DELAY_MS)) {
                shotCount++;
                spindexer.retreatShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS * 2) {
                barIntake.spinIntake();
                spindexer.clearTracking();
                turret.transferOff();
                intakeFlap.on();
                shotCount = 0;
                spindexer.setIntakeIndex(0);
                outtakeInProgress = false;
            }
        }
    }

    public static class Paths {
        public PathChain Pickup1;
        public PathChain Shoot1;
        public PathChain Pickup2;
        public PathChain Shoot2;
        public PathChain Park;

        public Paths(Follower follower) {
            // Pickup 1: (100, 9) -> (134, 9)
            Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(100.000, 9.000), new Pose(134.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Shoot 1: (134, 9) -> (100, 9)
            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.000, 9.000), new Pose(100.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Pickup 2: (100, 9) -> (130.572, 20.034)
            Pickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(100.000, 9.000), new Pose(122.024, 9.116), new Pose(130.572, 20.034)))
                    .setTangentHeadingInterpolation()
                    .build();

            // Shoot 2: (130.572, 20.034) -> (100, 9)
            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(130.572, 20.034), new Pose(100.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))
                    .build();

            // Park: (100, 9) -> (100, 15)
            Park = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(100.000, 9.000), new Pose(100.000, 15.000)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}