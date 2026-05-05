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

@Autonomous(name = "Blue 18 Ball Auto (Diagonal)", group = "Autonomous")
@Configurable
public class BlueDiagonalAuto extends OpMode {

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
    private Pose targetPose = new Pose(12, 132, 0); // Fixed target
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
        targetAngle = 288;
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        double distance = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());

        double currentRPM = 12.98196 * distance + 2192.57653;
        double rampUpFactor = 0.5 * distance;
        currentRPM += shotCount * (200 + rampUpFactor);

        double currentHood = 0.50;

        turret.setShooterRPM(currentRPM);
        turret.setHoodPosition(currentHood);
        turret.goToPosition(targetAngle);
        turret.on();

        spindexer.update();

        // Spit out logic
        boolean isShootingPath = (pathState == 9 || pathState == 12 || pathState == 15 || pathState == 18) && currentPose.getX() < 120;
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
                if (stateTimer.milliseconds() > 2750) {
                    startOuttakeRoutine();
                    setState(1);
                }
                break;

            case 1:
                targetAngle = 8;
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
                targetAngle = 340;
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
                targetAngle = 288;
                follower.followPath(paths.PickupStray);
                setState(8);
                break;

            case 8:
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(104.0, 9.0))).setTangentHeadingInterpolation().setReversed().build());
                    setState(9);
                } else if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(10);
                    }
                }
                break;

            case 10:
                follower.followPath(paths.PickupStray);
                setState(11);
                break;

            case 11:
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(104.0, 9.0))).setTangentHeadingInterpolation().setReversed().build());
                    setState(12);
                } else if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(13);
                    }
                }
                break;

            case 13:
                follower.followPath(paths.PickupStray);
                setState(14);
                break;

            case 14:
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(104.0, 9.0))).setTangentHeadingInterpolation().setReversed().build());
                    setState(15);
                } else if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(15);
                    }
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(16);
                    }
                }
                break;

            case 16:
                follower.followPath(paths.PickupStray);
                setState(17);
                break;

            case 17:
                if (spindexer.isFull()) {
                    follower.followPath(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), new Pose(104.0, 9.0))).setTangentHeadingInterpolation().setReversed().build());
                    setState(18);
                } else if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PICKUP_DELAY_MS) {
                        follower.followPath(paths.ShootStray);
                        setState(18);
                    }
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(19);
                    }
                }
                break;

            case 19:
                follower.followPath(paths.Leave);
                setState(20);
                break;

            case 20:
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
            if (currentTime - lastAdvanceTime > OUTTAKE_DELAY_MS) {
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
        public PathChain PickupStray;
        public PathChain ShootStray;
        public PathChain Leave;

        public Paths(Follower follower) {
            PickupCorner = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(105.000, 9.000),

                                    new Pose(135.000, 9.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ShootCorner = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 9.000),

                                    new Pose(86.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))

                    .build();

            PickupSpike = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 20.000),
                                    new Pose(88.720, 36.970),
                                    new Pose(131.000, 35.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootSpike = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.000, 35.500),

                                    new Pose(86.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                    .build();

            PickupStray = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 20.000),

                                    new Pose(135.000, 20.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootStray = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 20.000),

                                    new Pose(88.000, 20.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 20.000),

                                    new Pose(100.000, 20.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }

}
 
