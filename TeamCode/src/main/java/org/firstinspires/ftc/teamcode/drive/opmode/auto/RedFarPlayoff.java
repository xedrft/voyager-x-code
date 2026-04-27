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


    // -------------------- Timers --------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime spitTimer = new ElapsedTime();

    // -------------------- State machine --------------------
    private int pathState = 0;
    Pose targetPose = new Pose(132, 132, 0); // Fixed Red Target
    private int lastState = -1;
    private int loopCountBalls = 0;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
        }
        pathState = s;
    }

    // -------------------- Config --------------------
    public static double OUTTAKE_DELAY_MS = 350;
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
        spindexer.setShootIndex(2);
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

        double currentRPM = 12.98196 * distance + 2102.57653;
        double rampUpFactor = 0.5 * distance;
        currentRPM += shotCount * (200 + rampUpFactor);

        double currentHood = 0.58;

        turret.setShooterRPM(currentRPM);
        turret.setHoodPosition(currentHood);
        turret.on();

        spindexer.update();

        // Spit out logic
        boolean isShootingPath = (pathState == 4 || pathState == 8 || pathState == 12) && currentPose.getX() < 115;
        if ((spindexer.isFull() && !outtakeInProgress) || isShootingPath) {
            if (!spitInit) {
                spitTimer.reset();
                spitInit = true;
            }
            spindexer.goToOuttakePosition();
            double spitElapsed = spitTimer.milliseconds();
            if (spitElapsed > 150 && spitElapsed < 250) {
                barIntake.spinOuttake();
            } else if (spitElapsed >= 250) {
                spindexer.setShootIndex(2);
                barIntake.stop();
            } else {
                barIntake.stop();
            }
        } else {
            if (spitInit) {
                barIntake.spinIntake();
            }
            spitInit = false;
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
                    follower.followPath(paths.PickUpHumanPlayer);
                    setState(2);
                }
                break;

            case 2: // Arrive at PickUpHumanPlayer
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(99.0, 13.0))).setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build());
                    setState(4);
                } else if (!follower.isBusy()) {
                    waitTimer.reset();
                    setState(3);
                }
                break;

            case 3: // Wait 80ms for pickup
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(99.0, 13.0))).setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build());
                    setState(4);
                } else if (waitTimer.milliseconds() > 0) {
                    follower.followPath(paths.ShootHumanPlayer);
                    setState(4);
                }
                break;

            case 4: // Arrive at ShootHumanPlayer
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(5);
                }
                break;

            case 5: // After ShootHumanPlayer outtake, route to PickUpRack1 at 0.9 speed
                if (!outtakeInProgress) {
                    follower.followPath(paths.PickUpRack1, 0.9, false);
                    setState(6);
                }
                break;

            case 6: // Arrive at PickUpRack1
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(99.846, 11.537))).setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build());
                    setState(8);
                } else if (!follower.isBusy()) {
                    waitTimer.reset();
                    setState(7);
                }
                break;

            case 7: // Settle and ShootRack1
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(99.846, 11.537))).setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build());
                    setState(8);
                } else if (waitTimer.milliseconds() > 0.0) {
                    follower.followPath(paths.ShootRack1);
                    setState(8);
                }
                break;

            case 8: // Arrive at ShootRack1
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(9);
                }
                break;

            case 9: // Setup 2 loops for PickUpBalls
                if (!outtakeInProgress) {
                    loopCountBalls = 0;
                    follower.followPath(paths.PickUpBalls);
                    setState(10);
                }
                break;

            case 10: // Arrive at PickUpBalls
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(99.846, 13.229))).setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build());
                    setState(12);
                } else if (!follower.isBusy()) {
                    waitTimer.reset();
                    setState(11);
                }
                break;

            case 11: // Wait 80ms for pickup
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(99.846, 13.229))).setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build());
                    setState(12);
                } else if (waitTimer.milliseconds() > 0) {
                    follower.followPath(paths.ShootBalls);
                    setState(12);
                }
                break;

            case 12: // Arrive at ShootBalls
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(13);
                }
                break;

            case 13: // After ShootBalls outtake, repeat up to 2 times
                if (!outtakeInProgress) {
                    loopCountBalls++;
                    if (loopCountBalls < 2) {
                        follower.followPath(paths.PickUpBalls);
                        setState(10);
                    } else {
                        follower.followPath(paths.Park);
                        setState(14);
                    }
                }
                break;

            case 14: // Done, Park
                if (!follower.isBusy()) {
                    setState(15);
                }
                break;

            case 15:
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
        if (outtakeAdvanceCount < 3) {
            if (currentTime - lastAdvanceTime >= (outtakeAdvanceCount == 0 ? OUTTAKE_DELAY_MS / 1.5 : OUTTAKE_DELAY_MS)) {
                char[] filled = spindexer.getFilled();
                if (filled[spindexer.getShootIndex()] != '_') {
                    shotCount++;
                }
                spindexer.retreatShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
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
        public PathChain PickUpHumanPlayer;
        public PathChain ShootHumanPlayer;
        public PathChain PickUpRack1;
        public PathChain ShootRack1;
        public PathChain PickUpBalls;
        public PathChain ShootBalls;
        public PathChain Park;

        public Paths(Follower follower) {
            PickUpHumanPlayer = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(105.000, 10.000),

                                    new Pose(132.500, 10.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ShootHumanPlayer = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.500, 10.000),

                                    new Pose(99.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            PickUpRack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(99.000, 13.000),
                                    new Pose(100.105, 36.912),
                                    new Pose(125.417, 36.169)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootRack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.417, 36.169),

                                    new Pose(99.846, 11.537)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            PickUpBalls = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(99.846, 11.537),
                                    new Pose(127.329, 7.515),
                                    new Pose(131.921, 25.182)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.921, 25.182),

                                    new Pose(99.846, 13.229)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.846, 13.229),

                                    new Pose(100.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }
}

