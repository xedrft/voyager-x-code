package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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

@Autonomous(name = "New Rack Auto", group = "Autonomous")
@Configurable
public class RedCloseRackAuto extends OpMode {

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
    public static double OUTTAKE_DELAY_MS = 225;

    public static long PRESET_SETTLE_DELAY_MS = 0;
    public static long SETTLE_DELAY_MS = 0;
    public static long GATE_WAIT_MS = 80; // Optional hold delay during gate intake

    Pose targetPose = new Pose(132, 132, 0); // Fixed Blue Target
    // -------------------- State machine --------------------
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

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;

    private int shotCount = 0;

    private ElapsedTime spitTimer = new ElapsedTime();
    private boolean spitInit = false;

    // --- Shot/outtake state variables (from BlueTeleOp) ---
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.396, 120.422, Math.toRadians(0)));

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
        setState(0);
        stateTimer.reset();

        turret.on();
        turret.transferOff();
        intakeFlap.on();
        spindexer.setShootIndex(1);
        barIntake.spinIntake();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        Vector vel = follower.getVelocity();
        if (vel == null) {
            turret.trackTarget(follower.getPose(), targetPose, 0);
        } else {
            double flightTime = 0.6; // .2 second constant as requested
            double adjustX = vel.getXComponent() * flightTime;
            double adjustY = vel.getYComponent() * flightTime;
            Pose adjustedTarget = new Pose(targetPose.getX() - adjustX, targetPose.getY() - adjustY, targetPose.getHeading());
            turret.trackTarget(follower.getPose(), adjustedTarget, 0);
            telemetry.addData("CompAdjustX", adjustX);
            telemetry.addData("CompAdjustY", adjustY);
            telemetry.addData("AdjustedTarget", "(" + adjustedTarget.getX() + ", " + adjustedTarget.getY() + ")");
        }

        double distance = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());

        double currentRPM = 12.98196 * distance + 2192.57653;
        double currentHood = (1.07947*Math.pow(10,-7))*Math.pow(distance, 4) - 0.0000376157*Math.pow(distance, 3) + 0.00473038*Math.pow(distance, 2) - 0.256541*distance + 5.77716;

        double rampUpFactor = (distance > 100) ? 0.5 * distance : 0.3 * distance;
        currentRPM += shotCount * (250 + rampUpFactor);
        currentHood = turret.clamp(currentHood, 0, 1.0);

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

        turret.setShooterRPM(currentRPM);
        turret.setHoodPosition(currentHood);
        turret.on();

        spindexer.update();

        // Spit out logic from BlueTeleOp
        if (spindexer.isFull() && !outtakeInProgress) {
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

//        if (currentPose.getX() < 108 && !outtakeInProgress) {
//            spindexer.setShootIndex(2);
//        }

        autonomousUpdate();
        PoseStorage.currentPose = currentPose;
    }

    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {
            case 0:
                follower.followPath(paths.PresetShoot);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > PRESET_SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(2);
                    }
                }
                break;

            case 2:
                follower.followPath(paths.PickupRack2);
                setState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootRack2);
                    setState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(5);
                    }
                }
                break;

            // GateIntake 1
            case 5:
                follower.followPath(paths.GateIntake);
                setState(6);
                break;
                
            case 6:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > GATE_WAIT_MS) {
                        follower.followPath(paths.ShootGateIntake);
                        setState(7);
                    }
                }
                break;
                
            case 7:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(8);
                    }
                }
                break;
                
            // GateIntake 2
            case 8:
                follower.followPath(paths.GateIntake);
                setState(9);
                break;
                
            case 9:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > GATE_WAIT_MS) {
                        follower.followPath(paths.ShootGateIntake);
                        setState(10);
                    }
                }
                break;
                
            case 10:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(11);
                    }
                }
                break;

            // GateIntake 3
            case 11:
                follower.followPath(paths.GateIntake);
                setState(12);
                break;
                
            case 12:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > GATE_WAIT_MS) {
                        follower.followPath(paths.ShootGateIntake);
                        setState(13);
                    }
                }
                break;
                
            case 13:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(14);
                    }
                }
                break;

            // PickupRack1
            case 14:
                follower.followPath(paths.PickupRack1);
                setState(15);
                break;
                
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootRack1);
                    setState(16);
                }
                break;
                
            case 16:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(17);
                    }
                }
                break;

            // Park
            case 17:
                follower.followPath(paths.Park);
                setState(18);
                break;
                
            case 18:
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


        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();

        // Step 2: Set kicker servo to kick
        lastAdvanceTime = outtakeTimer.milliseconds();
    }


    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call
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
        public PathChain PresetShoot;
        public PathChain PickupRack2;
        public PathChain ShootRack2;
        public PathChain GateIntake;
        public PathChain ShootGateIntake;
        public PathChain PickupRack1;
        public PathChain ShootRack1;
        public PathChain Park;
        
        public static Pose shootPose = new Pose(86.000, 74.500);
        public static Pose gateIntakePose = new Pose(134.911, 59.700);
        public static double gateIntakeAngle = Math.toRadians(29);

        public Paths(Follower follower) {
            PresetShoot = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(123.396, 122.422),
                    shootPose
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            PickupRack2 = follower.pathBuilder().addPath(
                new BezierCurve(
                    shootPose,
                    new Pose(91.328, 58.654),
                    new Pose(123.713, 58.993)
                )
            ).setTangentHeadingInterpolation().build();

            ShootRack2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(123.713, 58.993),
                    shootPose
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            GateIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                    shootPose,
                    new Pose(113.717, 47.984),
                    gateIntakePose
                )
            ).setTangentHeadingInterpolation().build();

            ShootGateIntake = follower.pathBuilder().addPath(
                new BezierLine(
                    gateIntakePose,
                    shootPose
                )
            ).setTangentHeadingInterpolation().setReversed().build();

            PickupRack1 = follower.pathBuilder().addPath(
                new BezierCurve(
                    shootPose,
                    new Pose(98.720, 84.594),
                    new Pose(126.857, 84.155)
                )
            ).setLinearHeadingInterpolation(gateIntakeAngle, Math.toRadians(0)).build();

            ShootRack1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(126.857, 84.155),
                    new Pose(92.832, 83.469)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Park = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.832, 83.469),
                    new Pose(92.793, 64.741)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
        }
    }
}

