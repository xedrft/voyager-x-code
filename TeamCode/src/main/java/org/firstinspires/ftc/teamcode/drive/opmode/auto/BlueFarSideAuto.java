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

@Autonomous(name = "Blue Far Side Auto (OpMode)", group = "Autonomous")
@Configurable
public class BlueFarSideAuto extends OpMode {

    // -------------------- Panels + Pedro --------------------
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // -------------------- Subsystems (match your OpMode-based BlueAuto) --------------------
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
    private static final double DRIVETRAIN_STUCK_DIST_IN = 2.0; // inches of movement required to not be "stuck"
    public static double DRIVETRAIN_STUCK_MS = 1500;             // ms of no movement before triggering

    // -------------------- Spindexer stall recovery --------------------
    // 0 = normal, 1 = retreating to closest intake pos, 2 = returning to original target
    private int stallRecoveryState = 0;
    private double stallSavedTarget = 0.0;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private static final double STALL_POWER_THRESHOLD = 0.35;
    private static final double STALL_VELOCITY_THRESHOLD = 8.0;
    private static final double STALL_ERROR_THRESHOLD = 5.0;
    private static final double STALL_DETECT_MS = 350;

    // -------------------- Outtake routine (ported pattern from your OpMode BlueAuto) --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0.0;

    // You can tune this; in your excerpt you used ~650-700ms depending on shot
    public static double OUTTAKE_DELAY_MS = 900;
    public static double FAR_OUTTAKE_DELAY_MS = 900; // longer delay for preset far shot

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
    public static double SHOOT_INDEX_DELAY_MS = 200; // how far into the path before repositioning spindexer
    private boolean isSettling = false;
    private boolean shootIndexSet = false;

    // -------------------- Dynamic destinations --------------------
    // Park pose (same as your original far-side code)
    public static final Pose PARK_POSE = new Pose(
            44.0,
            14.69,
            Math.toRadians(180)
    );

    // Shoot pose during loop (same as original far-side code)
    public static final Pose SHOOT_POSE = new Pose(
            61.000, 14.000, Math.toRadians(180)
    );

    // -------------------- “Auto shoot power” equivalents in this framework --------------------
    // In your OpMode auto, shooter power is “RPM” + turret angle
    public static double currentRPM = 3265;
    public static double targetAngleDeg = 291;


    // ------------------------------------------------------------------------
    // init / start / loop (OpMode style)
    // ------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // IMPORTANT: starting pose should match the first path start (pickup1 starts at 64,8)
        Pose startPose = new Pose(64.000, 8.000, Math.toRadians(180));
        follower.setStartingPose(startPose);

        // Subsystems (match your OpMode BlueAuto names/signatures)
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

        // Build paths
        paths = new Paths(follower);

        // Startup config
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
        if (follower.isBusy() && !outtakeInProgress) {
            Pose cur = follower.getPose();
            if (lastDrivetrainPose == null) {
                lastDrivetrainPose = cur;
                drivetrainStuckTimer.reset();
            } else {
                double dist = Math.hypot(cur.getX() - lastDrivetrainPose.getX(),
                                         cur.getY() - lastDrivetrainPose.getY());
                if (dist > DRIVETRAIN_STUCK_DIST_IN) {
                    lastDrivetrainPose = cur;
                    drivetrainStuckTimer.reset();
                } else if (drivetrainStuckTimer.milliseconds() > DRIVETRAIN_STUCK_MS) {
                    // Stuck — abort current path and head to shoot position
                    paths.buildToShoot(follower, follower.getPose());
                    follower.followPath(paths.toShoot);
                    setState(90);
                }
            }
        } else {
            lastDrivetrainPose = null;
            drivetrainStuckTimer.reset();
        }

        // Keep shooter ready
        turret.setShooterRPM(currentRPM);
        turret.goToPosition(targetAngleDeg);
        turret.on();

        // Spindexer update
        spindexer.update();

        // --- Spindexer stall recovery ---
        if (!outtakeInProgress) {
            switch (stallRecoveryState) {
                case 0: // Normal — watch for stall
                    boolean motorStraining = Math.abs(spindexer.getLastOutput()) > STALL_POWER_THRESHOLD;
                    boolean notMoving      = Math.abs(spindexer.getLastVelocity()) < STALL_VELOCITY_THRESHOLD;
                    boolean notAtTarget    = Math.abs(spindexer.getLastError()) > STALL_ERROR_THRESHOLD;
                    if (motorStraining && notMoving && notAtTarget) {
                        if (stallTimer.milliseconds() > STALL_DETECT_MS) {
                            stallSavedTarget = spindexer.getReferenceAngle();
                            double cur = spindexer.getCalibratedAngle();
                            int closest = 0;
                            double minDiff = Double.MAX_VALUE;
                            for (int i = 0; i < Spindexer.INTAKE_ANGLES.length; i++) {
                                double d = Math.abs(Spindexer.INTAKE_ANGLES[i] - cur);
                                if (d > 180) d = 360 - d;
                                if (d < minDiff) { minDiff = d; closest = i; }
                            }
                            spindexer.startMoveToAngle(Spindexer.INTAKE_ANGLES[closest]);
                            stallRecoveryState = 1;
                        }
                    } else {
                        stallTimer.reset();
                    }
                    break;

                case 1: // Retreating to closest intake position
                    if (spindexer.isAtTarget(5.0)) {
                        spindexer.startMoveToAngle(stallSavedTarget);
                        stallRecoveryState = 2;
                    }
                    break;

                case 2: // Returning to original target
                    if (spindexer.isAtTarget(5.0)) {
                        stallTimer.reset();
                        stallRecoveryState = 0;
                    }
                    break;
            }
        } else {
            // Reset during outtake so we don't false-trigger after
            stallTimer.reset();
            stallRecoveryState = 0;
        }

        // Intake logic (mirrors BlueTeleOp)
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
                // Turret tracked-angle helpers were removed from Turret.
                // Treat the turret as "ready" when the shooter is up to speed.
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
                if (!follower.isBusy()) {

                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    // Time check: stop heading to shoot if < 1.5s remain, stay outside
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

                    // Time check: stop heading to shoot if < 1.5s remain, stay outside
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
                    // If time is nearly up, drive back to X < 50 rather than re-entering the loop
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
            // Park because time > 25
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
    // Paths class (matches your original far-side geometry + dynamic build)
    // ------------------------------------------------------------------------

    public static class Paths {
        public PathChain pickup1, shoot1, pickup2, shoot2, path5, path55, path6;

        // Dynamic paths (built at runtime)
        public PathChain toShoot;

        // End-of-match escape: drives from shoot pose back to X < 50
        public PathChain driveBack;

        public Paths(Follower follower) {
            pickup1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(64.000, 8.000),
                            new Pose(63.000, 41.000),
                            new Pose(10.000, 35.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 35.000),
                            new Pose(61.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickup2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(61.000, 14.000),
                            new Pose(13.555, 4.401),
                            new Pose(23.76, 19.89),
                            new Pose(10.000, 7.746)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 7.746),
                            new Pose(61.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.000, 14.000),
                            new Pose(10.000, 18)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            path55 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(25.00, 24.00),
                            new Pose(10.000, 18)

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 18),
                            new Pose(25, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            driveBack = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.000, 14.000),
                            new Pose(35.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }

        /** Rebuilds path55 to target the intake at a different Y (for 0-ball retries). */
        public void buildRetryIntakePath(Follower follower, Pose currentPose, double endY) {
            path55 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            currentPose,
                            new Pose(10.000, endY)
                    ))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(180))
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
