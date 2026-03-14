package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

import java.util.Objects;


@Autonomous(name = "Blue 9 Ball Auto", group = "Autonomous")
@Configurable
public class Blue9Auto extends OpMode {

    // -------------------- Panels + Pedro --------------------
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // -------------------- Subsystems --------------------
    private BarIntake barIntake;
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;

    // -------------------- Motifs --------------------
    private Limelight3A limelight;
    private int scannedTagId = 0;

    private static int[] getMotifForTag(int tagId) {
        switch (tagId) {
            case 21: return new int[]{2, 2, 1, 0};
            case 22: return new int[]{1, 1, 0, 2};
            case 23: return new int[]{0, 0, 2, 1};
            default: return null;
        }
    }

    private int[] order = null;
    private int currentOrderIndex = 0;
    private String currentBarIntakeState = "stop";

    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 250;
    public static double SHOOT_DEG = 318.8;
    public static double SHOOT_RPM = 2125;

    public static double PARK_SPEED = 1.0;

    private final ElapsedTime scanTimer = new ElapsedTime();

    public static double OUTTAKE_DELAY_MS = 450;        // gap between kicker going normal and next kick
    public static double KICK_DWELL_MS = 150;            // how long kicker stays up per ball
    public static double PICKUP_SETTLE_MS = 2500;        // wait after pickup before driving to shoot position
    public static double SHOOT_POSITION_SETTLE_MS = 1500; // wait after arriving at shoot position before firing
    private double targetAngle = SCAN_TURRET_DEG;

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;
    private static final long SETTLE_DELAY_MS = 250;
    boolean fallback = false;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
            isSettling = false;
        }
        pathState = s;
    }

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private int outtakeShotCount = 0;
    private boolean kickerUp = false;
    private double lastAdvanceTime = 0.0;
    private int spinInterval = 60;


    // -----------------------------------------------------------------------------------------
    // init / start / loop
    // -----------------------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.5, 122.5, Math.toRadians(180)));

        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(
                hardwareMap,
                "spindexerMotor",
                "spindexerAnalog",
                "distanceSensor",
                colorSensor
        );
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(
                hardwareMap,
                "shooter",
                "turret",
                "turretEncoder",
                "transferMotor",
                false,
                false
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();
        spindexer.filled = new char[]{'X', 'X', 'X'};

        paths = new Paths(follower);

        kickerServo.normal();
        turret.setShooterRPM(SHOOT_RPM);

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
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);
        spindexer.setShootIndex(1);
    }

    @Override
    public void loop() {
        follower.update();

        turret.on();
        turret.setShooterRPM(SHOOT_RPM);
        turret.goToPosition(targetAngle);

        spindexer.update();
        if (spindexer.isFull() && !outtakeInProgress && follower.getPose().getX() > 12) {
            spinInterval++;
            if (spinInterval > 30 && spinInterval < 40)
                currentBarIntakeState = "out";
            else
                currentBarIntakeState = "stop";
        }

        if (follower.getPose().getX() > 20 && !outtakeInProgress && (pathState == 5 || pathState == 8)) {
            if (order != null) spindexer.setShootIndex(order[currentOrderIndex]);
        }

        if (spindexer.isFull() && !outtakeInProgress) {
            if (order != null) spindexer.setShootIndex(order[currentOrderIndex]);
        }

        autonomousUpdate();
        PoseStorage.currentPose = follower.getPose();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", spindexer.getBalls());
        panelsTelemetry.debug("Scanned Tag ID", scannedTagId);

        if (currentBarIntakeState.equals("in")) {
            barIntake.spinIntake();
        } else if (currentBarIntakeState.equals("out")) {
            barIntake.spinOuttake();
        } else {
            barIntake.stop();
        }

        panelsTelemetry.update(telemetry);
    }


    // -----------------------------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------------------------

    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        if (scannedTagId == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                scannedTagId = result.getFiducialResults().get(0).getFiducialId();
                order = getMotifForTag(scannedTagId);
            } else if (scanTimer.milliseconds() > 1700) {
                fallback = true;
                scannedTagId = 21;
                order = getMotifForTag(scannedTagId);
            }
        }

        if (order != null && currentOrderIndex < order.length && !outtakeInProgress) {
            targetAngle = SHOOT_DEG;
        }

        switch (pathState) {
            // ------------------------------------------------------------
            // 0) Follow ShootPreset
            // ------------------------------------------------------------
            case 0:
                follower.followPath(paths.ShootPreset);
                setState(1);
                break;

            // ------------------------------------------------------------
            // 1) Wait for path + settle, then shoot volley 1
            // ------------------------------------------------------------
            case 1:
                if (!follower.isBusy() && stateTimer.milliseconds() > 2000) {
                    startOuttakeRoutine();
                    setState(2);
                    currentOrderIndex = 1;
                }
                break;

            // ------------------------------------------------------------
            // 2) After volley 1, go pickup1
            // ------------------------------------------------------------
            case 2:
                if (fallback) {
                    order = getMotifForTag(21);
                }
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup1);
                    setState(3);
                }
                break;

            // ------------------------------------------------------------
            // 3) After pickup1, settle then go to Shoot1
            // ------------------------------------------------------------
            case 3:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_SETTLE_MS) {
                        spinInterval = 10;
                        follower.followPath(paths.Shoot1);
                        setState(5);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 5) After Shoot1 path, shoot volley 2
            // ------------------------------------------------------------
            case 5:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SHOOT_POSITION_SETTLE_MS) {
                        startOuttakeRoutine();
                        setState(6);
                        currentOrderIndex = 2;
                    }
                }
                break;

            // ------------------------------------------------------------
            // 6) After volley 2, go pickup2
            // ------------------------------------------------------------
            case 6:
                if (fallback) {
                    order = getMotifForTag(22);
                }
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup2, 0.95, false);
                    setState(7);
                }
                break;

            // ------------------------------------------------------------
            // 7) After pickup2, settle then go to Shoot2
            // ------------------------------------------------------------
            case 7:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_SETTLE_MS) {
                        spinInterval = 10;
                        follower.followPath(paths.Shoot2);
                        setState(8);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 8) After Shoot2 path, shoot volley 3
            // ------------------------------------------------------------
            case 8:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SHOOT_POSITION_SETTLE_MS) {
                        startOuttakeRoutine();
                        setState(9);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 9) After volley 3, leave
            // ------------------------------------------------------------
            case 9:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Leave, PARK_SPEED, false);
                    setState(10);
                }
                break;

            // ------------------------------------------------------------
            // 10) Idle
            // ------------------------------------------------------------
            case 10:
                break;
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeShotCount = 1;
        kickerUp = true;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        turret.transferOn();
        currentBarIntakeState = "stop";

        kickerServo.kick();
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        if (kickerUp) {
            // Kicker is up — wait for dwell, then bring it down
            if (currentTime - lastAdvanceTime >= KICK_DWELL_MS) {
                kickerServo.normal();
                kickerUp = false;
                lastAdvanceTime = currentTime;

                if (outtakeShotCount >= 3) {
                    // All 3 shots done — clean up
                    spindexer.clearTracking();
                    currentBarIntakeState = "in";
                    spindexer.setIntakeIndex(0);
                    spinInterval = 0;
                    outtakeInProgress = false;
                } else {
                    // Advance to next ball
                    spindexer.advanceShoot();
                    outtakeAdvanceCount++;
                }
            }
        } else {
            // Kicker is down — wait for advance to settle, then kick next ball
            if (outtakeShotCount < 3 && currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                kickerServo.kick();
                kickerUp = true;
                outtakeShotCount++;
                lastAdvanceTime = currentTime;
            }
        }
    }


    // -----------------------------------------------------------------------------------------
    // Paths
    // -----------------------------------------------------------------------------------------

    public static class Paths {

        public PathChain ShootPreset;
        public PathChain Pickup1;
        public PathChain Overflow;
        public PathChain Shoot1;
        public PathChain Pickup2;
        public PathChain Shoot2;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.500, 122.500),
                                    new Pose(38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(38.000, 108.000),
                                    new Pose(67.500, 79.000),
                                    new Pose(13.000, 84.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Overflow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.000, 84.500),
                                    new Pose(37.000, 76.000),
                                    new Pose(14.500, 76.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.000, 84.500),
                                    new Pose(38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(38.000, 108.000),
                                    new Pose(88.000, 55.000),
                                    new Pose(5.500, 59.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(5.500, 59.500),
                                    new Pose(61.000, 52.000),
                                    new Pose(38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(38.000, 108.000),
                                    new Pose(38.000, 75.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}
