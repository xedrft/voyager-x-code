package org.firstinspires.ftc.teamcode.drive.opmode.auto.far;

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

@Autonomous(name = "Blue 18 Ball Far Auto (Gay)", group = "Autonomous")
@Configurable
public class BlueGayFarAuto extends OpMode {

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

    // -------------------- Config (tune in Panels) --------------------
    public static double OUTTAKE_DELAY_MS = 350;
    Pose targetPose = new Pose(12, 132, 0); // Fixed Blue Target
    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;
    private static final long SETTLE_DELAY_MS = 0;
    private static final int PICKUP_DELAY_MS = 200;


    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
            isSettling = false;
        }
        pathState = s;
    }

    private void abortToShoot(Pose shootPose, double targetHeadingDeg) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), shootPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(targetHeadingDeg))
                .build());
    }

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int shotCount = 0;
    private int targetAngle = 289;

    private ElapsedTime spitTimer = new ElapsedTime();
    private boolean spitInit = false;

    // --- Shot/outtake state variables ---
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(39.000, 9.000, Math.toRadians(180)));

        // Subsystems
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

        // Paths
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        targetAngle = 289;
    }

    @Override
    public void start() {
        outtakeInProgress = false;
        setState(0); // shoot presets immediately
        stateTimer.reset();

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

        double distance = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());

        double currentRPM = 12.98196 * distance + 2102.57653;
        double rampUpFactor = 0.5 * distance;
        currentRPM += shotCount * (200 + rampUpFactor);

        double currentHood = 0.50;

        turret.setShooterRPM(currentRPM);
        turret.setHoodPosition(currentHood);
        turret.goToPosition(targetAngle);
        turret.on();

        spindexer.update();

        // Spit out logic
        boolean isShootingPath = (pathState == 3 || pathState == 6 || pathState == 11 || pathState == 16 || pathState == 21) && currentPose.getX() > 24;
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
    }

    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {
            case 0: // Shoot presets immediately
                if (stateTimer.milliseconds() > 2750) { // short delay for turret
                    startOuttakeRoutine();
                    setState(1);
                }
                break;

            case 1:
                targetAngle = 7;
                follower.followPath(paths.PickupCorner);
                setState(2);
                break;

            case 2:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootCorner);
                        setState(3);
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(4);
                    }
                }
                break;

            case 4:
                targetAngle = 338;
                follower.followPath(paths.PickupSpike);
                setState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        follower.followPath(paths.ShootSpike);
                        setState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(7);
                    }
                }
                break;

            case 7:
                targetAngle = 289;
                follower.followPath(paths.PickupStray1);
                setState(8);
                break;

            case 8:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(11);
                } else if(!follower.isBusy()) {
                    follower.followPath(paths.PickupStray2);
                    setState(9);
                }
                break;

            case 9:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(11);
                } else if(!follower.isBusy()) {
                    follower.followPath(paths.PickupStray3);
                    setState(10);
                }
                break;

            case 10:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(11);
                } else if(!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(12); // Loop 2 starts
                    }
                }
                break;

            case 12:
                follower.followPath(paths.PickupStray1);
                setState(13);
                break;

            case 13:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(16);
                } else if(!follower.isBusy()) {
                    follower.followPath(paths.PickupStray2);
                    setState(14);
                }
                break;

            case 14:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(16);
                } else if(!follower.isBusy()) {
                    follower.followPath(paths.PickupStray3);
                    setState(15);
                }
                break;

            case 15:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(16);
                } else if(!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if(settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(16);
                    }
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(17); // Loop 3 starts
                    }
                }
                break;

            case 17:
                follower.followPath(paths.PickupStray1);
                setState(18);
                break;

            case 18:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(21);
                } else if(!follower.isBusy()) {
                    follower.followPath(paths.PickupStray2);
                    setState(19);
                }
                break;

            case 19:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(21);
                } else if(!follower.isBusy()) {
                    follower.followPath(paths.PickupStray3);
                    setState(20);
                }
                break;

            case 20:
                if (spindexer.isFull()) {
                    abortToShoot(new Pose(39.0, 9.0), 140);
                    setState(21);
                } else if(!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if(settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(21);
                    }
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(22); // Leave
                    }
                }
                break;

            case 22:
                follower.followPath(paths.Leave);
                setState(23);
                break;

            case 23:
                // done
                break;
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        intakeFlap.off();
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        turret.transferOn();
        lastAdvanceTime = outtakeTimer.milliseconds();
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
        public PathChain PickupCorner;
        public PathChain ShootCorner;
        public PathChain PickupSpike;
        public PathChain ShootSpike;
        public PathChain PickupStray1;
        public PathChain PickupStray2;
        public PathChain PickupStray3;
        public PathChain ShootStray;
        public PathChain Leave;

        public Paths(Follower follower) {
            PickupCorner = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(39.000, 9.000),
                    new Pose(9.000, 9.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            ShootCorner = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(9.000, 9.000),
                    new Pose(58.000, 20.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110)).build();

            PickupSpike = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(58.000, 20.000),
                    new Pose(29.000, 9.000),
                    new Pose(22.500, 29.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90)).build();

            ShootSpike = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(22.500, 29.000),
                    new Pose(58.000, 20.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140)).build();

            PickupStray1 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(58.000, 20.000),
                    new Pose(49.000, 11.000),
                    new Pose(9.000, 9.000)
                )
            ).setTangentHeadingInterpolation().build();

            PickupStray2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(9.000, 9.000),
                    new Pose(15.000, 19.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();

            PickupStray3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(15.000, 19.000),
                    new Pose(13.000, 34.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(120)).build();

            ShootStray = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(13.000, 34.000),
                    new Pose(39.000, 9.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(140)).build();

            Leave = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(39.000, 9.000),
                    new Pose(25.000, 9.000)
                )
            ).setTangentHeadingInterpolation().build();
        }
    }
}
