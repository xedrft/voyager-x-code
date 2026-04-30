package org.firstinspires.ftc.teamcode.drive.opmode.auto.sort;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import java.util.Objects;

@Autonomous(name = "Red Side Sorted Auto", group = "Autonomous")
@Configurable
public class RedSideSortedAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // Subsystems
    private BarIntake barIntake;
    private IntakeFlap intakeFlap;
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    private Turret turret;
    private ElapsedTime spitTimer = new ElapsedTime();
    private boolean spitInit = false;

    // Limelight / motif sort (from RedTwelveBallAuto)
    private Limelight3A limelight;
    private int scannedTagId = 0;
    private int[] order = null;
    private int currentOrderIndex = 0;
    private boolean fallback = false;
    private final ElapsedTime scanTimer = new ElapsedTime();

    private static int[] getMotifForTag(int tagId) {
        switch (tagId) {
            case 21: return new int[]{2, 2, 1, 0}; // GPP
            case 22: return new int[]{0, 0, 2, 1}; // PGP
            case 23: return new int[]{1, 1, 0, 2}; // PPG
            default: return null;
        }
    }

    // Config
    public static double OUTTAKE_DELAY_MS = 350;
    public static double PARK_SPEED = 1.0;
    public static double SCAN_TURRET_DEG = 150;
    private static final long SETTLE_DELAY_MS = 250;

    // Shooting (from RedCloseRackAuto)
    private Pose targetPose = new Pose(132, 132, 0); // Fixed target
    private int shotCount = 0;

    // State machine
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
            isSettling = false;
        }
        pathState = s;
    }

    // Outtake
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 21.5, 121.5, Math.toRadians(0)));

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();

        spindexer.filled = new char[]{'X', 'X', 'X'};

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        outtakeInProgress = false;
        setState(0);
        stateTimer.reset();
        scanTimer.reset();

        turret.on();
        turret.transferOff();
        intakeFlap.on();
        spindexer.setShootIndex(2);
        barIntake.spinIntake();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        // While tag not yet found, point turret to scan angle; after, track target
        if (scannedTagId == 0) {
            turret.goToPosition(SCAN_TURRET_DEG);
        } else {
            Vector vel = follower.getVelocity();
            if (vel != null) {
                turret.trackTarget(currentPose, targetPose, 0);
            } else {
                double flightTime = 0.6;
                double adjustX = vel.getXComponent() * flightTime;
                double adjustY = vel.getYComponent() * flightTime;
                Pose adjustedTarget = new Pose(
                        targetPose.getX() - adjustX,
                        targetPose.getY() - adjustY,
                        targetPose.getHeading()
                );
                turret.trackTarget(currentPose, adjustedTarget, 0);
            }
        }

        // Distance-based RPM and hood (from RedCloseRackAuto)
        double distance = Math.hypot(
                targetPose.getX() - currentPose.getX(),
                targetPose.getY() - currentPose.getY()
        );
        double currentRPM = 12.98196 * distance + 2072.57653;
        double currentHood = (1.07947e-7) * Math.pow(distance, 4)
                - 0.0000376157 * Math.pow(distance, 3)
                + 0.00473038  * Math.pow(distance, 2)
                - 0.256541    * distance + 5.77716;

        double rampUpFactor = 0.3 * distance;
        //currentRPM += shotCount * (100 + rampUpFactor);
        currentHood = turret.clamp(currentHood, 0, 1.0);

        turret.setShooterRPM(currentRPM);
        turret.setHoodPosition(currentHood);
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
                if (order != null) spindexer.setShootIndex(order[currentOrderIndex]);
                else spindexer.setShootIndex(2);
                barIntake.stop();
            }
            else {
                barIntake.stop();
            }
        } else {
            spitInit = false;
        }

        // Spindexer + sort logic (from RedTwelveBallAuto)
        spindexer.update();




        autonomousUpdate();
        PoseStorage.currentPose = currentPose;

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", currentPose.getX());
        panelsTelemetry.debug("Y", currentPose.getY());
        panelsTelemetry.debug("Heading", currentPose.getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", spindexer.getBalls());
        panelsTelemetry.debug("Scanned Tag ID", scannedTagId);
        panelsTelemetry.debug("Order Index", currentOrderIndex);
        panelsTelemetry.debug("RPM", currentRPM);
        panelsTelemetry.debug("Distance", distance);
        panelsTelemetry.update(telemetry);
    }

    // -----------------------------------------------------------------------------------------
    // State machine (from RedTwelveBallAuto)
    // -----------------------------------------------------------------------------------------

    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        // Limelight tag scan
        if (scannedTagId == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getFiducialResults() != null
                    && !result.getFiducialResults().isEmpty()
                    && scanTimer.milliseconds() > 1000) {
                scannedTagId = result.getFiducialResults().get(0).getFiducialId();
                order = getMotifForTag(scannedTagId);
            } else if (scanTimer.milliseconds() > 2000) {
                fallback = true;
                scannedTagId = 21;
                order = getMotifForTag(scannedTagId);
            }
        }

        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreset);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy() && stateTimer.milliseconds() > 2500) {
                    startOuttakeRoutine();
                    setState(2);
                    currentOrderIndex = 1;
                }
                break;

            case 2:
                if (!outtakeInProgress) {
                    if (fallback) order = getMotifForTag(21);
                    follower.followPath(paths.Pickup1, 0.9, false);
                    setState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        follower.followPath(paths.Overflow);
                        setState(4);
                    }
                }
                // fall through to case 4

            case 4:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > 1000) {
                        follower.followPath(paths.Shoot1);
                        setState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(6);
                        currentOrderIndex = 2;
                    }
                }
                break;

            case 6:
                if (!outtakeInProgress) {
                    if (fallback) order = getMotifForTag(22);
                    follower.followPath(paths.Pickup2, 0.95, false);
                    setState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        follower.followPath(paths.Shoot2);
                        setState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(9);
                        currentOrderIndex = 3;
                    }
                }
                break;

            case 9:
                if (!outtakeInProgress) {
                    if (fallback) order = getMotifForTag(23);
                    follower.followPath(paths.Pickup3, 0.95, false);
                    setState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        follower.followPath(paths.Shoot3);
                        setState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(12);
                    }
                }
                break;

            case 12:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(13);
                }
                break;

            case 13:
                break;
        }
    }

    // -----------------------------------------------------------------------------------------
    // Outtake (from RedCloseRackAuto)
    // -----------------------------------------------------------------------------------------

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        intakeFlap.off();
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();

        // Step 2: Set kicker servo to kick
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call
        if (outtakeAdvanceCount < 3) {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {

                // Only increase shotCount if there's actually a ball in the current shoot index
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

    // -----------------------------------------------------------------------------------------
    // Paths (from RedTwelveBallAuto)
    // -----------------------------------------------------------------------------------------

    public static class Paths {
        public PathChain ShootPreset;
        public PathChain Pickup1;
        public PathChain Overflow;
        public PathChain Shoot1;
        public PathChain Pickup2;
        public PathChain Shoot2;
        public PathChain Pickup3;
        public PathChain Shoot3;
        public PathChain Park;

        public Paths(Follower follower) {
            ShootPreset = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 21.500, 122.500),
                            new Pose(144- 42, 105)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Pickup1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144- 42, 105),
                            new Pose(144 - 67.500,  79.000),
                            new Pose(144 - 15.000,  85.500)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Overflow = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144 - 15.000, 85.500),
                            new Pose(144 - 27.000, 80.000),
                            new Pose(144 - 13.500, 78.000)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Shoot1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 13.500, 78.000),
                            new Pose(144- 42, 105)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Pickup2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144- 42, 105),
                            new Pose(144 - 70.000,  55.000),
                            new Pose(144 -  5.500,  60.500)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Shoot2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144 -  5.500,  60.500),
                            new Pose(144 - 61.000,  52.000),
                            new Pose(144- 42, 105)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Pickup3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144- 42, 105),
                            new Pose(144 - 86.000,  27.500),
                            new Pose(144 -  5.500,  37.500)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Shoot3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144 -  5.500,  37.500),
                            new Pose(144 - 40.000,  48.501),
                            new Pose(144 -  3.191,  75.649),
                            new Pose(144 - 52.419,  68.354),
                            new Pose(144 - 32.928,  93.919),
                            new Pose(144- 42, 105)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Park = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144- 42, 105),
                            new Pose(144- 42,  75.000)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
        }
    }
}
