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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;

@Autonomous(name = "Red Far Side Auto (OpMode)", group = "Autonomous")
@Configurable
public class RedFarSideAuto extends OpMode {

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

    // -------------------- Timers --------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
            isSettling = false;
            shootIndexSet = false;
            lastDrivetrainPose = null;
            drivetrainStuckTimer.reset();
        }
        pathState = s;
    }

    // -------------------- Drivetrain stuck recovery --------------------
    private Pose lastDrivetrainPose = null;
    private final ElapsedTime drivetrainStuckTimer = new ElapsedTime();
    private static final double DRIVETRAIN_STUCK_DIST_IN = 2.0;
    public static double DRIVETRAIN_STUCK_MS = 1500;

    // -------------------- Spindexer stall recovery --------------------
    // 0 = normal, 1 = retreating to closest intake pos, 2 = returning to original target
    private int stallRecoveryState = 0;
    private double stallSavedTarget = 0.0;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private static final double STALL_POWER_THRESHOLD = 0.35;
    private static final double STALL_VELOCITY_THRESHOLD = 8.0;
    private static final double STALL_ERROR_THRESHOLD = 5.0;
    private static final double STALL_DETECT_MS = 350;

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0.0;

    public static double OUTTAKE_DELAY_MS = 900;
    public static double FAR_OUTTAKE_DELAY_MS = 900;

    private boolean farOuttakeMode = false;

    // Stop going to the shoot position when this many seconds remain in the 30s auto
    public static double LOOP_STOP_TIME_S = 27.0;

    private int spinInterval = 0;

    // Retry logic: alternate Y positions when 0 balls are picked up
    private int retryCount = 0; // increments each time 0 balls detected in loop
    // retryCount % 2 == 1 → 9.0, retryCount % 2 == 0 → 35.0

    // Shoot queue: only shoot filled slots, in order
    private int[] shootQueue = new int[0];
    private int shootQueueIndex = 0;

    /** Returns indexes (0,1,2) of filled slots in ascending order. */
    private int[] buildShootQueue() {
        int count = 0;
        for (char c : spindexer.filled) if (c != '_') count++;
        int[] q = new int[count];
        int i = 0;
        for (int j = 0; j < 3; j++) if (spindexer.filled[j] != '_') q[i++] = j;
        return q;
    }

    /** Returns the first filled slot index, or 0 as fallback. */
    private int getFirstFilledIndex() {
        for (int i = 0; i < 3; i++) if (spindexer.filled[i] != '_') return i;
        return 0;
    }

    // -------------------- Loop timing --------------------
    public static int LOOP_WAIT_MS = 50;

    public static int SETTLE_TIME = 400;
    public static double SHOOT_INDEX_DELAY_MS = 200;
    private boolean isSettling = false;
    private boolean shootIndexSet = false;

    // -------------------- Dynamic destinations --------------------
    // Park pose: X mirrored (144 - 44 = 100), heading flipped 180° → 0°
    public static final Pose PARK_POSE = new Pose(
            100.0,
            14.69,
            Math.toRadians(0)
    );

    // Shoot pose: X mirrored (144 - 61 = 83), heading flipped 180° → 0°
    public static final Pose SHOOT_POSE = new Pose(
            83.000, 14.000, Math.toRadians(0)
    );

    // -------------------- Shooter config --------------------
    public static double currentRPM = 3300;
    public static double targetAngleDeg = 66;


    // ------------------------------------------------------------------------
    // init / start / loop (OpMode style)
    // ------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Starting pose: X mirrored (144 - 64 = 80), heading flipped 180° → 0°
        Pose startPose = new Pose(80.000, 8.000, Math.toRadians(0));
        follower.setStartingPose(startPose);

        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(
                hardwareMap,
                "spindexerMotor",
                "spindexerAnalog",
                "distanceSensor",
                colorSensor
        );
        spindexer.setColorAtPos('X', 0);
        spindexer.setColorAtPos('X', 1);
        spindexer.setColorAtPos('X', 2);
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

        paths = new Paths(follower);

        kickerServo.normal();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        matchTimer.reset();
        waitTimer.reset();
        stateTimer.reset();

        outtakeInProgress = false;

        turret.on();
        barIntake.spinIntake();

        setState(0);
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // --- Drivetrain stuck detection ---
        // If the follower is actively driving but the robot hasn't moved, go back to shoot position
//        if (follower.isBusy() && !outtakeInProgress) {
//            Pose cur = follower.getPose();
//            if (lastDrivetrainPose == null) {
//                lastDrivetrainPose = cur;
//                drivetrainStuckTimer.reset();
//            } else {
//                double dist = Math.hypot(cur.getX() - lastDrivetrainPose.getX(),
//                                         cur.getY() - lastDrivetrainPose.getY());
//                if (dist > DRIVETRAIN_STUCK_DIST_IN) {
//                    lastDrivetrainPose = cur;
//                    drivetrainStuckTimer.reset();
//                } else if (drivetrainStuckTimer.milliseconds() > DRIVETRAIN_STUCK_MS) {
//                    // Stuck — abort current path and head to shoot position
//                    paths.buildToShoot(follower, follower.getPose());
//                    follower.followPath(paths.toShoot);
//                    setState(90);
//                }
//            }
//        } else {
//            lastDrivetrainPose = null;
//            drivetrainStuckTimer.reset();
//        }

        // Keep shooter ready
        turret.setShooterRPM(currentRPM);
        turret.goToPosition(targetAngleDeg);
        turret.on();

        // Spindexer update
        spindexer.update();

        // --- Spindexer stall recovery ---
//        if (!outtakeInProgress) {
//            switch (stallRecoveryState) {
//                case 0: // Normal — watch for stall
//                    boolean motorStraining = Math.abs(spindexer.getLastOutput()) > STALL_POWER_THRESHOLD;
//                    boolean notMoving      = Math.abs(spindexer.getLastVelocity()) < STALL_VELOCITY_THRESHOLD;
//                    boolean notAtTarget    = Math.abs(spindexer.getLastError()) > STALL_ERROR_THRESHOLD;
//                    if (motorStraining && notMoving && notAtTarget) {
//                        if (stallTimer.milliseconds() > STALL_DETECT_MS) {
//                            stallSavedTarget = spindexer.getReferenceAngle();
//                            double cur = spindexer.getCalibratedAngle();
//                            int closest = 0;
//                            double minDiff = Double.MAX_VALUE;
//                            for (int i = 0; i < Spindexer.INTAKE_ANGLES.length; i++) {
//                                double d = Math.abs(Spindexer.INTAKE_ANGLES[i] - cur);
//                                if (d > 180) d = 360 - d;
//                                if (d < minDiff) { minDiff = d; closest = i; }
//                            }
//                            spindexer.startMoveToAngle(Spindexer.INTAKE_ANGLES[closest]);
//                            stallRecoveryState = 1;
//                        }
//                    } else {
//                        stallTimer.reset();
//                    }
//                    break;
//
//                case 1: // Retreating to closest intake position
//                    if (spindexer.isAtTarget(5.0)) {
//                        spindexer.startMoveToAngle(stallSavedTarget);
//                        stallRecoveryState = 2;
//                    }
//                    break;
//
//                case 2: // Returning to original target
//                    if (spindexer.isAtTarget(5.0)) {
//                        stallTimer.reset();
//                        stallRecoveryState = 0;
//                    }
//                    break;
//            }
//        } else {
//            // Reset during outtake so we don't false-trigger after
//            stallTimer.reset();
//            stallRecoveryState = 0;
//        }

        // Intake logic
        // Activate when full OR when traveling to shoot with any balls loaded
        boolean travelingToShoot = pathState == 3 || pathState == 6 || pathState == 90;
        if ((spindexer.isFull() || (travelingToShoot && spindexer.getBalls() > 0)) && !outtakeInProgress) {
            spinInterval++;
            if (spinInterval < 40)
                barIntake.spinIntake();       // let last ball settle in
            else if (spinInterval < 50)
                barIntake.spinOuttake();      // push overflow back out
            else
                barIntake.stop();
        }

        if (outtakeInProgress) {
            barIntake.stop();
        }

        // Run auto state machine
        autonomousUpdate();

        PoseStorage.currentPose = follower.getPose();

        // Telemetry
        panelsTelemetry.debug("Match Time (s)", matchTimer.seconds());
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Target RPM", currentRPM);
        panelsTelemetry.debug("Turret Angle (deg)", targetAngleDeg);
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Full", spindexer.isFull());
        panelsTelemetry.debug("Stall Recovery", stallRecoveryState == 0 ? "Normal" : stallRecoveryState == 1 ? "Retreating" : "Returning");
        panelsTelemetry.update(telemetry);
    }

    // ------------------------------------------------------------------------
    // Outtake routine (kick + advance x3 on delay)
    // ------------------------------------------------------------------------

    private void startFarOuttakeRoutine() {
        farOuttakeMode = true;
        startOuttakeRoutine();
    }

    private void startOuttakeRoutine() {
        // Build queue from currently filled slots only
        shootQueue = buildShootQueue();
        shootQueueIndex = 0;
        outtakeAdvanceCount = 0;

        outtakeInProgress = true;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        turret.transferOn();

        if (shootQueue.length == 0) {
            // No balls to shoot — reset spindexer back to intake so the next pickup works
            spindexer.setIntakeIndex(0);
            barIntake.spinIntake();
            spinInterval = 0;
            farOuttakeMode = false;
            outtakeInProgress = false;
            return;
        }

        // Position to first filled slot, then kick
        spindexer.setShootIndex(shootQueue[0]);
        kickerServo.kick();
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        int advancesNeeded = shootQueue.length - 1; // one advanceShoot per additional filled slot

        if (outtakeAdvanceCount < advancesNeeded) {

            double delay = farOuttakeMode ? FAR_OUTTAKE_DELAY_MS : OUTTAKE_DELAY_MS;
            if (currentTime - lastAdvanceTime >= delay) {
                spindexer.advanceShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            // All filled slots shot — clean up
            double cleanupDelay = farOuttakeMode ? FAR_OUTTAKE_DELAY_MS : OUTTAKE_DELAY_MS;
            if (currentTime - lastAdvanceTime >= cleanupDelay) {
                kickerServo.normal();
                spindexer.clearTracking();
                barIntake.spinIntake();
                spindexer.setIntakeIndex(0);
                spinInterval = 0;
                farOuttakeMode = false;
                outtakeInProgress = false;
            }
        }
    }

    // ------------------------------------------------------------------------
    // State machine (far-side strategy)
    // ------------------------------------------------------------------------

    private void autonomousUpdate() {
        // If outtaking, block path transitions
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {

            // ------------------------------------------------------------
            // 0: Preset Shot (shoot in place)
            // ------------------------------------------------------------
            case 0:
                spindexer.setShootIndex(0);
                if (turret.getShooterRPM() > 3400) {
                    startFarOuttakeRoutine();
                    setState(1);
                }
                break;

            case 1:
                // After preset volley completes, go pickup1
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickup1);
                    waitTimer.reset();
                    setState(2);
                }
                break;

            // ------------------------------------------------------------
            // Cycle 1: pickup1 -> short wait -> shoot1 -> volley -> pickup2
            // ------------------------------------------------------------
            case 2:
                if (!follower.isBusy() && stateTimer.milliseconds() > 2500) {
                    follower.followPath(paths.shoot1);
                    setState(3);
                }
                break;

            case 3:
                // Pre-position to first filled slot while en route (not before path starts)
                if (!shootIndexSet && stateTimer.milliseconds() > SHOOT_INDEX_DELAY_MS) {
                    spindexer.setShootIndex(getFirstFilledIndex());
                    shootIndexSet = true;
                }
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        waitTimer.reset();
                    }
                    if (waitTimer.milliseconds() > SETTLE_TIME) {
                        startOuttakeRoutine();
                        setState(4);
                    }
                }
                break;

            case 4:
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickup2);
                    waitTimer.reset();
                    setState(5);
                }
                break;

            // ------------------------------------------------------------
            // Cycle 2: pickup2 -> wait -> shoot2 -> volley -> path5
            // ------------------------------------------------------------
            case 5:
                if (!follower.isBusy() && stateTimer.milliseconds() > 2500) {
                    follower.followPath(paths.shoot2);
                    setState(6);
                }
                break;

            case 6:
                if (!shootIndexSet && stateTimer.milliseconds() > SHOOT_INDEX_DELAY_MS) {
                    spindexer.setShootIndex(getFirstFilledIndex());
                    shootIndexSet = true;
                }
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        waitTimer.reset();
                    }
                    if (waitTimer.milliseconds() > SETTLE_TIME) {
                        startOuttakeRoutine();
                        setState(7);
                    }
                }
                break;

            case 7:
                if (!outtakeInProgress) {
                    // Begin loop: go to path5 first
                    follower.followPath(paths.path5);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // Loop: path5 <-> path6 with priority checks
            // ------------------------------------------------------------
            case 8:
                if (!follower.isBusy() || stateTimer.milliseconds() > 3000) { // if stuck for 3s, re-evaluate (prevents softlock if something goes wrong in the loop)

                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    // Time check: stop heading to shoot if time is nearly up, stay outside
                    if (matchTimer.seconds() > LOOP_STOP_TIME_S) {
                        setState(99);
                        break;
                    }

                    // If balls to shoot, go to shoot position
                    if (spindexer.getBalls() > 0) {
                        retryCount = 0;
                        paths.buildToShoot(follower, follower.getPose());
                        follower.followPath(paths.toShoot);
                        setState(90);
                        break;
                    }

                    // Otherwise continue loop to path6
                    follower.followPath(paths.path6);
                    waitTimer.reset();
                    setState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {

                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    // Time check: stop heading to shoot if time is nearly up, stay outside
                    if (matchTimer.seconds() > LOOP_STOP_TIME_S) {
                        setState(99);
                        break;
                    }

                    // If balls to shoot, go to shoot position
                    if (spindexer.getBalls() > 0) {
                        retryCount = 0;
                        paths.buildToShoot(follower, follower.getPose());
                        follower.followPath(paths.toShoot);
                        setState(90);
                        break;
                    }

                    // 0 balls — retry at alternate Y (9 on odd retry, 35 on even retry)
                    retryCount++;
                    double retryY = (retryCount % 2 == 1) ? 9.0 : 35.0;
                    paths.buildRetryIntakePath(follower, follower.getPose(), retryY);
                    // IMPORTANT: also rebuild path6 so it starts from the same Y as the retry intake point
                    paths.buildPath6(follower, retryY);
                    follower.followPath(paths.path55);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // Go to shoot because full
            // ------------------------------------------------------------
            case 90:
                if (!shootIndexSet && stateTimer.milliseconds() > SHOOT_INDEX_DELAY_MS) {
                    spindexer.setShootIndex(getFirstFilledIndex());
                    shootIndexSet = true;
                }
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        waitTimer.reset();
                    }
                    if (waitTimer.milliseconds() > SETTLE_TIME) {
                        startOuttakeRoutine();
                        setState(91);
                    }
                }
                break;

            case 91:
                if (!outtakeInProgress) {
                    // If time is nearly up, drive back to X > 94 rather than re-entering the loop
                    if (matchTimer.seconds() > LOOP_STOP_TIME_S) {
                        follower.followPath(paths.driveBack);
                        setState(98);
                        break;
                    }
                    follower.followPath(paths.path5);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // Drive back to safe zone then idle
            // ------------------------------------------------------------
            case 98:
                if (!follower.isBusy()) {
                    setState(99);
                }
                break;

            case 99:
                // End
                break;
        }
    }

    // ------------------------------------------------------------------------
    // Paths class (X coordinates mirrored: 144 - blueX; headings flipped 180° → 0°)
    // ------------------------------------------------------------------------

    public static class Paths {
        public PathChain pickup1, shoot1, pickup2, shoot2, path5, path55, path6;

        // Dynamic paths (built at runtime)
        public PathChain toShoot;

        // End-of-match escape: drives from shoot pose back to X > 94
        public PathChain driveBack;

        public Paths(Follower follower) {
            // pickup1: blue (64,8)→(63,41)→(10,35) | red (80,8)→(81,41)→(134,35)
            pickup1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(80.000, 8.000),
                            new Pose(81.000, 41.000),
                            new Pose(134.000, 35.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot1: blue (10,35)→(61,14) | red (134,35)→(83,14)
            shoot1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, 35.000),
                            new Pose(83.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pickup2: blue (61,14)→(13.555,4.401)→(23.76,19.89)→(10,7.746)
            //           red  (83,14)→(130.445,4.401)→(120.24,19.89)→(134,7.746)
            pickup2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83.000, 14.000),
                            new Pose(130.445, 4.401),
                            new Pose(120.240, 19.890),
                            new Pose(134.000, 7.746)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot2: blue (10,7.746)→(61,14) | red (134,7.746)→(83,14)
            shoot2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, 7.746),
                            new Pose(83.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // path5: blue (61,14)→(10,18) | red (83,14)→(134,18)
            path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(83.000, 14.000),
                            new Pose(134.000, 18.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // path55: blue (25,24)→(10,18) | red (119,24)→(134,18)
            path55 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.000, 24.000),
                            new Pose(134.000, 18.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // path6: blue (10,18)→(25,14) | red (134,18)→(119,14)
            path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, 18.000),
                            new Pose(119.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // driveBack: blue (61,14)→(35,14) | red (83,14)→(109,14)
            driveBack = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(83.000, 14.000),
                            new Pose(109.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }

        /** Rebuilds path55 to target the intake at a different Y (for 0-ball retries). */
        public void buildRetryIntakePath(Follower follower, Pose currentPose, double endY) {
            path55 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            currentPose,
                            new Pose(134.000, endY)  // blue 10 → red 134
                    ))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(0))
                    .build();
        }

        /** Rebuilds path6 to connect from the dynamic intake Y (used in retries). */
        public void buildPath6(Follower follower, double startY) {
            // blue: (10, startY) -> (25, 14)
            // red : (134, startY) -> (119, 14)
            path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, startY),
                            new Pose(119.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }

        public void buildToShoot(Follower follower, Pose currentPose) {
            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, SHOOT_POSE))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), SHOOT_POSE.getHeading())
                    .build();
        }
    }
}
