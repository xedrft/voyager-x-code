package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.bylazar.lights.Headlight;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {
    private Follower follower;
    private LockMode lockMode;
    private boolean isLocked = false;
    private static final Pose startingPose = PoseStorage.currentPose;

    private BarIntake barIntake;

    private Servo ledHeadlight;
    private Servo ledHeadlight2;

    private Spindexer spindexer;
    private int offset_turret = 0;
    private KickerServo kickerServo;
    private Turret turret;
    private ColorSensor colorSensor;
    private ElapsedTime loopTimer;
    private ElapsedTime outtakeTimer;
    private LynxModule expansionHub;
    private static final double OFFSET = Math.toRadians(180.0);
    private Pose targetPose = new Pose(0, 144, 0); // Fixed target
    private GoBildaPinpointDriver pinpoint;

    // Outtake routine state
    private boolean outtakeInProgress = false;

    boolean rpmCap = true;
    private boolean singleOuttakeInProgress = false;
    private boolean singleAtPosition = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;
    private static double OUTTAKE_DELAY_MS = 150;

    private int spinInterval = 0;
    private boolean goingToPosition = false;
    private static Pose GO_TO_TARGET = new Pose(18.53, 58.42, 2.67);


    private double currentRPM = 2500.0;

    // --- velocity-based RPM compensation ---
    private Pose lastPose = null;
    private boolean lastFull = false;
    private double lastPoseTimeSec = 0.0;

    /**
     * Inches/sec. Positive = robot moving away from target (distance increasing),
     * negative = robot moving toward target (distance decreasing).
     */
    private double radialVelocityIps = 0.0;

    /** Tune: RPM change per (inch/sec) of radial velocity. */
    private static final double RPM_PER_IPS = 4.0;

    /** Tune: ignore tiny velocity noise. */
    private static final double RADIAL_VEL_DEADBAND_IPS = 1.0;

    /** Tune: clamp total velocity compensation so it can’t run away. */
    private static final double MAX_RPM_VEL_COMP = 250.0;

    private static int CloseCap = 2600;

    // --- shoot-while-moving lead compensation ---
    /** Toggle for shoot-while-moving lead adjustment. */
    private boolean shootWhileMoving = true;

    /** Tune: flat RPM boost when robot speed exceeds MOVING_SPEED_THRESHOLD. */
    private static final double MOVING_RPM_BOOST = 100.0;
    /** Tune: minimum speed (ips) to be considered "moving". */
    private static final double MOVING_SPEED_THRESHOLD = 3.0;

    @Override
    public void init() {
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        lockMode = new LockMode(follower);
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);
        loopTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //turret.goToPosition(180);
        ledHeadlight = hardwareMap.get(Servo.class, "ledLight");
        ledHeadlight.setPosition(0.0);
        ledHeadlight2 = hardwareMap.get(Servo.class, "ledLight2");
        ledHeadlight2.setPosition(0.0);
        pinpoint.recalibrateIMU();

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
        turret.transferOn();
    }

    @Override
    public void loop() {
        expansionHub.clearBulkCache();

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Update follower first
        follower.update();

        // --- lock mode drive control ---
        // When locked, LockMode runs a tiny oscillation path to keep translational/heading PIDs engaged.
        // Otherwise, ensure we are in normal teleop drive.
        if (isLocked) {
            lockMode.lockPosition();
        } else {
            lockMode.unlockPosition();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false,
                    OFFSET
            );
        }

        // --- go-to-position on A button ---
        if (gamepad1.aWasPressed()) {
            Pose cur = follower.getPose();
            PathChain goToPath = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(cur.getX(), cur.getY(), cur.getHeading()),
                            GO_TO_TARGET))
                    .setLinearHeadingInterpolation(cur.getHeading(), GO_TO_TARGET.getHeading())
                    .build();
            follower.followPath(goToPath, 0.5, false);
            goingToPosition = true;
        }
        if (goingToPosition) {
            boolean stickMoved = Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1;
            if (!follower.isBusy() || stickMoved) {
                goingToPosition = false;
                follower.setMaxPower(1.0);
                follower.startTeleopDrive();
            }
        }

        if(gamepad1.bWasPressed()){
            GO_TO_TARGET = follower.getPose();
        }

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

        // Field Reset
        if (gamepad1.shareWasPressed()){
            follower.setPose(new Pose(136.5, 7.75, Math.toRadians(0)));
            turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);
            pinpoint.recalibrateIMU();
            // Ensure LockMode doesn't keep stale state across reset
            isLocked = false;
            lockMode.unlockPosition();
        }

        // Spindex control
        if (gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }


        if (gamepad1.xWasPressed()){
            spindexer.clearTracking();
            barIntake.spinIntake();
        }

        // Outtake routine trigger
        if (gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            turret.on();
            startOuttakeRoutine();
        }

        if(gamepad1.dpadDownWasPressed()){
            rpmCap = !rpmCap;
            gamepad1.rumble(200);
        }

        if (!rpmCap){ //if there is NO rpm cap.
            OUTTAKE_DELAY_MS = 600;
            offset_turret = -7;
        }
        else { //if there IS an RPM cap
            OUTTAKE_DELAY_MS = 300;
            offset_turret = 0;

        }


        double distance = Math.sqrt((targetPose.getX() - currentPose.getX())
                * (targetPose.getX() - currentPose.getX())
                + (targetPose.getY() - currentPose.getY())
                * (targetPose.getY() - currentPose.getY()));

        // Turret always aims at the real goal
        turret.trackTarget(currentPose, targetPose, offset_turret);

        // --- shoot-while-moving: adjust RPM based on lead-compensated distance ---
        double aimDistance;
        if (shootWhileMoving) {
            Pose aimPose = getLeadAdjustedTarget(currentPose, targetPose, distance);
            aimDistance = Math.sqrt((aimPose.getX() - currentPose.getX())
                    * (aimPose.getX() - currentPose.getX())
                    + (aimPose.getY() - currentPose.getY())
                    * (aimPose.getY() - currentPose.getY()));
        } else {
            aimDistance = distance;
        }

        currentRPM = 0.0151257 * aimDistance * aimDistance
                + 10.03881 * aimDistance
                + 1382.4428;

        // Flat RPM boost when robot is moving to compensate for energy loss
        double robotSpeed = Math.hypot(
                follower.getVelocity().getXComponent(),
                follower.getVelocity().getYComponent());
        if (robotSpeed > MOVING_SPEED_THRESHOLD) {
            currentRPM += MOVING_RPM_BOOST;
        }

        currentRPM = (currentRPM > CloseCap && rpmCap) ? CloseCap : currentRPM;


        if(gamepad1.dpadLeftWasPressed()){
            if(CloseCap == 2800){
                CloseCap = 2600;
            }else{
                CloseCap = 2800;
            }
            gamepad1.rumble(200);
        }

        // Update RPM
        turret.setShooterRPM(currentRPM);
        turret.on(); // Update velocity


        telemetry.addData("Goal Distance (in)", String.format(java.util.Locale.US, "%.1f", distance));
        telemetry.addData("Aim Distance (in)", String.format(java.util.Locale.US, "%.1f", aimDistance));
        telemetry.addData("Distance Delta (in)", String.format(java.util.Locale.US, "%.2f", aimDistance - distance));
        telemetry.addData("Est. Air Time (s)", String.format(java.util.Locale.US, "%.3f", estimateAirTime(distance)));
        telemetry.addData("Robot Speed (ips)", String.format(java.util.Locale.US, "%.1f", robotSpeed));
        telemetry.addData("RPM Boost Active", robotSpeed > MOVING_SPEED_THRESHOLD);
        telemetry.addData("Current target RPM:", String.format(java.util.Locale.US, "%.0f", currentRPM));

        if (gamepad1.leftStickButtonWasPressed()){
            startSingleOuttake('P');
        }
        if (gamepad1.rightStickButtonWasPressed()){
            startSingleOuttake('G');
        }
        // Handle outtake routine sequence
        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }
        if (singleOuttakeInProgress){
            handleSingleOuttake();
        }


        if (spindexer.isFull()){
            ledHeadlight.setPosition(1.0);
            ledHeadlight2.setPosition(1.0);
            if (!lastFull) gamepad2.rumble(2000);
            lastFull = true;
        }
        else{
            ledHeadlight.setPosition(0.0);
            ledHeadlight2.setPosition(0.0);
            lastFull = false;
        }

        if (spindexer.isFull() && !outtakeInProgress && !singleOuttakeInProgress){
            spindexer.setShootIndex(1);
            spinInterval++;
            if (spinInterval > 30 && spinInterval < 50)
                barIntake.spinOuttake();
            else {
                barIntake.stop();
            }
        }

        if (outtakeInProgress){
            barIntake.stop();
        }

        spindexer.update();




        // Spindexer diagnostic telemetry (angle, velocity, adaptive tolerance, output, etc.)

        // Telemetry
        telemetry.addData("Lock Mode Active", isLocked);
        telemetry.addData("Spindexer Index", spindexer.getIntakeIndex());
        telemetry.addData("Robot Pose: ", "(" + follower.getPose().getX() + ", " + follower.getPose().getY() + ", " + follower.getPose().getHeading() + ")" );
        telemetry.addData("Adaptive Tolerance", String.format(java.util.Locale.US, "%.2f", spindexer.getLastAdaptiveTol()));
        telemetry.addData("Turret RPM Error", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM() - turret.getSetShooterRPM()));
        telemetry.addData("Turret Current Error", String.format(java.util.Locale.US, "%.2f", turret.getCurrentError()));
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;


        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();

        // Step 2: Set kicker servo to kick
        kickerServo.kick();
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call
        if (outtakeAdvanceCount < 2) {
            if (currentTime - lastAdvanceTime >= (outtakeAdvanceCount == 0 ? OUTTAKE_DELAY_MS / 3 : OUTTAKE_DELAY_MS)) {
                spindexer.advanceShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                // All 3 advanceIntake calls completed, set kicker back to normal
                kickerServo.normal();
                spindexer.clearTracking();
                barIntake.spinIntake();
                spinInterval = 0;
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

        turret.transferOn();

        spindexer.setShootIndex(index);
    }

    private void handleSingleOuttake(){
        if (!singleAtPosition) {
            if (spindexer.isAtTarget(5.0)){
                singleAtPosition = true;
                outtakeTimer.reset();
                kickerServo.kick();
            }
        } else {
            if (outtakeTimer.milliseconds() > OUTTAKE_DELAY_MS){
                kickerServo.normal();
                spindexer.setColorAtPos('_', spindexer.getShootIndex());
                singleOuttakeInProgress = false;
                if (!spindexer.isFull()) {
                    barIntake.spinIntake();
                }
            }
        }
    }

    /**
     * Estimates the ball air time (in seconds) based on distance to the goal.
     * TODO: Replace with actual regression equation from collected data.
     *
     * @param distanceInches distance from robot to goal in inches
     * @return estimated air time in seconds
     */
    private double estimateAirTime(double distanceInches) {
        // TODO: Replace with regression equation, e.g.:
        // return a * distanceInches * distanceInches + b * distanceInches + c;
        return -0.0000100511*distanceInches*distanceInches+0.00572639*distanceInches+0.694903; // placeholder constant (seconds)
    }

    /**
     * Computes a lead-adjusted target pose to aim at so the ball arrives at the
     * actual goal despite the robot moving while the ball is in the air.
     *
     * Formula: aimTarget = actualGoal - (robotVelocity × airTime)
     *
     * @param robotPose   current robot pose
     * @param goalPose    actual goal position
     * @param distInches  distance from robot to goal
     * @return adjusted aim pose
     */
    private Pose getLeadAdjustedTarget(Pose robotPose, Pose goalPose, double distInches) {
        double airTime = estimateAirTime(distInches);

        // Get robot velocity vector from the follower (inches/sec)
        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();

        // Adjusted target = actual goal - (velocity × air time)
        double adjustedX = goalPose.getX() - (vx * airTime);
        double adjustedY = goalPose.getY() - (vy * airTime);

        return new Pose(adjustedX, adjustedY, goalPose.getHeading());
    }
}

