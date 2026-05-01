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

@Autonomous(name = "Blue leave", group = "Autonomous")
@Configurable
public class BlueCloseLeave extends OpMode {

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
        targetAngle = 289;
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();





        spindexer.update();

        autonomousUpdate();
        PoseStorage.currentPose = currentPose;
    }

    private void autonomousUpdate() {

        switch (pathState) {
            case 0: // Shoot presets immediately
                follower.followPath(paths.Leave);
                setState(1);
                break;

            case 1:
                break;
        }
    }






    public static class Paths {
        public PathChain PickupCorner;
        public PathChain ShootCorner;
        public PathChain PickupSpike;
        public PathChain ShootSpike;
        public PathChain PickupStray;
        public PathChain ShootStray;
        public PathChain PickupStray2;
        public PathChain ShootStray2;
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
                                    new Pose(55.280, 36.970),
                                    new Pose(13.000, 35.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootSpike = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(22.500, 28.000),
                            new Pose(58.000, 20.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140)).build();

            PickupStray = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(58.000, 20.000),
                            new Pose(49.000, 11.000),
                            new Pose(9.000, 9.000)
                    )
            ).setTangentHeadingInterpolation().build();

            ShootStray = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(9.000, 9.000),
                            new Pose(40.000, 9.000)
                    )
            ).setTangentHeadingInterpolation().setReversed().build();

            PickupStray2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(40.000, 9.000),
                            new Pose(9.000, 9.000)
                    )
            ).setTangentHeadingInterpolation().build();

            ShootStray2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(9.000, 9.000),
                            new Pose(40.000, 9.000)
                    )
            ).setTangentHeadingInterpolation().setReversed().build();

            Leave = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(39.000, 9.000),
                            new Pose(25.000, 9.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
        }
    }
}
