package org.firstinspires.ftc.teamcode.drive.opmode.teleop.competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.intake.IntakeFlap;
import org.firstinspires.ftc.teamcode.intake.IntakeServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Lights;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

import com.pedropathing.math.Vector; // added for velocity compensation

@TeleOp(name = "!! Red TeleOp", group = "TeleOp")
public class RedTeleOp extends OpMode {
    private Follower follower;
    private LockMode lockMode;
    private boolean isLocked = false;
    private static final Pose startingPose = PoseStorage.currentPose;

    private BarIntake barIntake;
    private IntakeFlap intakeFlap;


    private Spindexer spindexer;
    private int offset_turret = 0;
    private IntakeServo intakeServo;
    private Turret turret;
    private ColorSensor colorSensor;
    //private KickerServo kickerServo;
    private ElapsedTime loopTimer;
    private ElapsedTime outtakeTimer;
    private LynxModule expansionHub;
    private static final double OFFSET = Math.toRadians(0.0);
    private Pose targetPose = new Pose(132, 132, 0); // Fixed target

    // Outtake routine state
    private boolean outtakeInProgress = false;

    boolean rpmCap = true;
    private boolean singleOuttakeInProgress = false;
    private boolean singleAtPosition = false;
    private static double OUTTAKE_DELAY_MS = 300;

    private ElapsedTime spitTimer = new ElapsedTime();
    private boolean spitInit = false;
//    private boolean goingToPosition = false;
//    private static Pose GO_TO_TARGET = new Pose(18.53, 58.42, 2.67);


    private double currentRPM = 2500.0;
    private double currentHood = 0.5;
    private int shotCount = 0;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;

    // --- velocity-based RPM compensation ---
    private Pose lastPose = null;
    private double lastPoseTimeSec = 0.0;

    /**
     * Inches/sec. Positive = robot moving away from target (distance increasing),
     * negative = robot moving toward target (distance decreasing).
     */
    private double radialVelocityIps = 0.0;


    /** Tune: ignore tiny velocity noise. */
    private static final double RADIAL_VEL_DEADBAND_IPS = 1.0;

    Lights lights;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        lockMode = new LockMode(follower);
        barIntake = new BarIntake(hardwareMap, "barIntake", false);
        intakeFlap = new IntakeFlap(hardwareMap, "intakeFlapServo");
        intakeServo = new IntakeServo(hardwareMap, "intakeServo");
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor, intakeFlap);
//        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", true, false);
        loopTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        lights = new Lights(hardwareMap);


        //turret.goToPosition(180);
//        ledHeadlight = hardwareMap.get(Servo.class, "ledLight");
//        ledHeadlight.setPosition(0.0);
//        ledHeadlight2 = hardwareMap.get(Servo.class, "ledLight2");
//        ledHeadlight2.setPosition(0.0);


        if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
        } else {
            // Default starting position if Auto wasn't run
            follower.setPose(new Pose(0, 0, 0));
        }

        // Initialize velocity estimator
        lastPose = follower.getPose();
        lastPoseTimeSec = getRuntime();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.on();
        barIntake.spinIntake();
        intakeFlap.on();
        intakeServo.intake();
        turret.transferOff();
        spindexer.setIntakeIndex(0);
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        boolean colorScanInProgress = spindexer.isAccurateColorScanInProgress();

        if (colorScanInProgress) {
            intakeFlap.off();
            intakeServo.outtake();
            barIntake.stop();
        } else if (!spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress) {
            lights.turnAllOff();
            intakeFlap.on();
            intakeServo.intake();
        }
        else {
            lights.turnAllOn();
            intakeFlap.off();
            intakeServo.outtake();
        }

        // Update follower first
        follower.update();
        // --- lock mode drive control ---
        // When locked, LockMode runs a tiny oscillation path to keep translational/heading PIDs engaged.
        // Otherwise, ensure we are in normal teleop drive.
//        if (isLocked && gamepad1.left_trigger > 0.5) {
//            lockMode.lockPosition();
//        } else {
//            lockMode.unlockPosition();
//            follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    false,
//                    OFFSET
//            );
//        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET
        );

        // --- estimate robot velocity (radial relative to target) ---
        Pose currentPose = follower.getPose();
        double nowSec = getRuntime();
        double dt = nowSec - lastPoseTimeSec;
        if (lastPose != null && dt > 1e-3) {
            double dx = currentPose.getX() - lastPose.getX();
            double dy = currentPose.getY() - lastPose.getY();

            // Robot velocity vector (inches/sec)
            double vx = dx / dt;
            double vy = dy / dt;

            // Unit vector from robot -> target
            double toTargetX = targetPose.getX() - currentPose.getX();
            double toTargetY = targetPose.getY() - currentPose.getY();
            double distToTarget = Math.hypot(toTargetX, toTargetY);

            if (distToTarget > 1e-6) {
                double ux = toTargetX / distToTarget;
                double uy = toTargetY / distToTarget;

                // Positive means moving toward target; negative means moving away
                double closingSpeedIps = vx * ux + vy * uy;

                // We want a sign convention where + = away, - = toward (distance rate).
                radialVelocityIps = -closingSpeedIps;

                if (Math.abs(radialVelocityIps) < RADIAL_VEL_DEADBAND_IPS) {
                    radialVelocityIps = 0.0;
                }
            } else {
                radialVelocityIps = 0.0;
            }
        }
        lastPose = currentPose;
        lastPoseTimeSec = nowSec;
        OUTTAKE_DELAY_MS = (currentPose.getY() < 25) ? 400 : 225;


        // Field Reset
        if (gamepad1.shareWasPressed()) {
            follower.setPose(new Pose(9, 9, Math.toRadians(0)));
            turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", true, false);
            // Ensure LockMode doesn't keep stale state across reset
            isLocked = false;
            lockMode.unlockPosition();
        }

        if (gamepad2.yWasPressed() && !colorScanInProgress && !outtakeInProgress && !singleOuttakeInProgress) {
            spindexer.startAccurateColorScan();
            colorScanInProgress = spindexer.isAccurateColorScanInProgress();
            if (colorScanInProgress) {
                barIntake.stop();
                intakeFlap.off();
                intakeServo.outtake();
                gamepad2.rumble(200);
            }
        }


        // Spindex control
        if (!colorScanInProgress && gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (!colorScanInProgress && gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }


        if (!colorScanInProgress && gamepad1.xWasPressed()) {
            spindexer.clearTracking();
            barIntake.spinIntake();
        }


        // Outtake routine trigger (manual left trigger)
        if (!colorScanInProgress && gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            turret.on();
            startOuttakeRoutine();
        }

//         Turret tracking: use velocity compensation when shooting while moving
//         If robot Y position is > 30, compute adjusted aiming point by subtracting robot velocity * flightTime from target
        if (follower.getPose().getY() > 30) {
            Vector vel = follower.getVelocity();
            if (vel == null) {
                turret.trackTarget(follower.getPose(), targetPose, offset_turret);
            } else {
                double flightTime = 0.6; // .2 second constant as requested
                double adjustX = vel.getXComponent() * flightTime;
                double adjustY = vel.getYComponent() * flightTime;
                Pose adjustedTarget = new Pose(targetPose.getX() - adjustX, targetPose.getY() - adjustY, targetPose.getHeading());
                turret.trackTarget(follower.getPose(), adjustedTarget, offset_turret);
                telemetry.addData("CompAdjustX", adjustX);
                telemetry.addData("CompAdjustY", adjustY);
                telemetry.addData("AdjustedTarget", "(" + adjustedTarget.getX() + ", " + adjustedTarget.getY() + ")");
            }
        } else {
            turret.trackTarget(follower.getPose(), targetPose, offset_turret);
        }




        double distance = Math.sqrt((targetPose.getX() - follower.getPose().getX())
                * (targetPose.getX() - follower.getPose().getX())
                + (targetPose.getY() - follower.getPose().getY())
                * (targetPose.getY() - follower.getPose().getY()));

        currentRPM = 12.98196 * distance + 2192.57653;
        currentHood = (1.07947*Math.pow(10,-7))*Math.pow(distance, 4) - 0.0000376157*Math.pow(distance, 3) + 0.00473038*Math.pow(distance, 2) - 0.256541*distance + 5.77716;


        // Velocity compensation:
        // - if moving toward goal (radialVelocityIps negative) => decrease RPM
        // - if moving away (radialVelocityIps positive) => increase RPM
//        double velComp = RPM_PER_IPS * radialVelocityIps;
//        velComp = Math.max(-MAX_RPM_VEL_COMP, Math.min(MAX_RPM_VEL_COMP, velComp));
//        currentRPM += velComp;

        if (currentPose.getY() < 25){
            currentRPM = 17.1 * distance + 1700;
            currentHood = 0.5;
        }

        double rampUpFactor = (distance > 100) ? 0.5 * distance : 0.3 * distance;
        currentRPM += shotCount * (250 + rampUpFactor);
        currentHood = turret.clamp(currentHood, 0, 1.0);
        //currentHood += shotCount * 0.02;





        // Update RPM
        turret.setShooterRPM(currentRPM);
        turret.on(); // Update velocity
        //update hood
        turret.setHoodPosition(currentHood);


        telemetry.addData("Calculated Distance (in)", distance);
        telemetry.addData("Radial Vel (ips)", radialVelocityIps);
        //telemetry.addData("RPM Vel Comp", velComp);

        if (!colorScanInProgress && gamepad1.leftStickButtonWasPressed()) {
            startSingleOuttake('P');
        }
        if (!colorScanInProgress && gamepad1.rightStickButtonWasPressed()) {
            startSingleOuttake('G');
        }
        // Handle outtake routine sequence
        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }
        if (singleOuttakeInProgress) {
            handleSingleOuttake();
        }

        // spit out
        if (!colorScanInProgress && spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress) {
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

        if (outtakeInProgress) {
            barIntake.stop();
        }

        spindexer.update();


        // Spindexer diagnostic telemetry (angle, velocity, adaptive tolerance, output, etc.)

        // Telemetry
        telemetry.addData("Lock Mode Active", isLocked);
        telemetry.addData("Spindexer Index", spindexer.getIntakeIndex());
        telemetry.addData("Robot Pose: ", "(" + follower.getPose().getX() + ", " + follower.getPose().getY() + ", " + follower.getPose().getHeading() + ")");
        telemetry.addData("Adaptive Tolerance", String.format(java.util.Locale.US, "%.2f", spindexer.getLastAdaptiveTol()));
        telemetry.addData("Turret RPM Error", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM() - turret.getSetShooterRPM()));
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        telemetry.addData("Color Scan In Progress", spindexer.isAccurateColorScanInProgress());
        telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        intakeFlap.off();
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;


        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();

        isLocked = true;

        // Step 2: Set kicker servo to kick
        lastAdvanceTime = outtakeTimer.milliseconds();
    }


    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call
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
                isLocked = false;
            }
        }
    }

    private void startSingleOuttake(char color){
        int index = -1;
        char[] filled = spindexer.getFilled();
        for (int i = 0; i < 3; i++){
            if (filled[i] == color){
                index = i;
                break;
            }
        }
        if (index == -1) return;
        singleOuttakeInProgress = true;
        singleAtPosition = false;

        spindexer.setShootIndex(index);
    }

    private void handleSingleOuttake(){
        if (!singleAtPosition) {
            if (spindexer.isAtTarget(5.0)){
                singleAtPosition = true;
                outtakeTimer.reset();
                turret.transferOn();
                shotCount++;
            }
        } else {
            if (outtakeTimer.milliseconds() > OUTTAKE_DELAY_MS){
                turret.transferOff();
                spindexer.setColorAtPos('_', spindexer.getShootIndex());
                singleOuttakeInProgress = false;
                shotCount = 0;
                if (spindexer.isEmpty()) {
                    barIntake.spinIntake();
                    spindexer.setIntakeIndex(0);
                }
            }
        }
    }
}
