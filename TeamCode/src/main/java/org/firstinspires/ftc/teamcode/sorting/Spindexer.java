package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.intake.IntakeFlap;

public class Spindexer {
    private enum ColorScanState {
        IDLE,
        MOVING,
        SAMPLING,
        WAITING_BETWEEN_SLOTS
    }

    private final DcMotorEx spindexerMotor;
    private final AnalogInput analogEncoder;
    private final DigitalChannel distanceSensor;
    private final IntakeFlap intakeFlap;

    // Telemetry / diagnostics
    private double lastVelocity = 0.0;
    private double lastAdaptiveTol = 0.0;
    private double lastCurrentAngle = 0.0;
    private double lastError = 0.0;
    private double lastOutput = 0.0;
    private double lastDt = 0.0;

    // --- PIDF Coefficients ---
    // Start with these. If it oscillates, lower Kp. If it stops short, raise kStatic.
    public static double Kp = 0.009;
    public static double Ki = 0.000;
    public static double Kd = 0.0006;
    public static double kStatic = 0.0325; // Minimum power to overcome friction
    public ColorSensor colorSensor;

    // PID state
    private double integralSum = 0.0;
    private double lastMeasuredAngle = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime runtimeTimer = new ElapsedTime();
    private final ElapsedTime colorScanTimer = new ElapsedTime();

    private double referenceAngle = 0.0;

    // Settings
    private static final double ANALOG_MAX_VOLTAGE = 3.3;
    public static double FLAP_ON_DETECTION_DELAY_MS = 250.0;
    public static double COLOR_SCAN_POSITION_TOLERANCE_DEG = 5.0;
    public static double COLOR_SCAN_SETTLE_MS = 150.0;
    public static double COLOR_SCAN_NEXT_SLOT_DELAY_MS = 100.0;

    // Calibration
    private double angleOffsetDegrees = 30.0;

    // Intake flap gating
    private final ElapsedTime flapOnTimer = new ElapsedTime();
    private boolean lastFlapOn = false;

    // Tracking
    public char[] filled = {'_', '_', '_'};
    private int intakeIndex = 0;
    private int shootIndex = 0;
    private ColorScanState colorScanState = ColorScanState.IDLE;
    private int colorScanIndex = -1;
    private double preScanReferenceAngle = 0.0;
    private int scanGreenCount = 0;
    private int scanPurpleCount = 0;
    private int scanUnknownCount = 0;

    // Spin mode state: allow spinning a specified number of degrees by driving motor open-loop
    private boolean spinModeActive = false;
    private double spinTargetDegrees = 0.0;       // total degrees to spin (positive)
    private double spinAccumulatedDegrees = 0.0;  // accumulated absolute rotation
    private double spinPower = 0.6;              // motor power to use while spinning
    private int spinDirection = 1;               // 1 = positive direction, -1 = negative

    // Positions (Degrees)
    // Intake: 0, 120, 240
    public static final double[] INTAKE_ANGLES = {240.0, 120.0, 0.0};
    // Shoot: 180 (0.5), 300 (0.833), 60 (0.167)
    public static final double[] SHOOT_ANGLES = {240.0, 120.0, 0.0};
    public static final double[] COLOR_SCAN_ANGLES = {60.0, 300.0, 180.0};
    public static final double OUTTAKE_ANGLE = 60.0;

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName, String distanceSensorName, ColorSensor colorSensor, IntakeFlap intakeFlap) {
        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.analogEncoder = hardwareMap.get(AnalogInput.class, analogName);
        this.distanceSensor = hardwareMap.get(DigitalChannel.class, distanceSensorName);
        this.distanceSensor.setMode(DigitalChannel.Mode.INPUT);

        spindexerMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.colorSensor = colorSensor;
        this.intakeFlap = intakeFlap;
    }

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName, String distanceSensorName, ColorSensor colorSensor) {
        this(hardwareMap, motorName, analogName, distanceSensorName, colorSensor, null);
    }

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName, String distanceSensorName) {
        this(hardwareMap, motorName, analogName, distanceSensorName, null, null);
    }

    // --- Input Processing ---

    public double getAngleFromAnalog() {
        double v = analogEncoder.getVoltage();
        // Clamp to prevent weird spikes
        if (v < 0) v = 0;
        if (v > ANALOG_MAX_VOLTAGE) v = ANALOG_MAX_VOLTAGE;
        return (v / ANALOG_MAX_VOLTAGE) * 360.0;
    }

    public double getCalibratedAngle() {
        double raw = getAngleFromAnalog();
        return normalizeAngleDegrees(raw + angleOffsetDegrees);
    }

    public void calibrateSetCurrentAsZero() {
        double raw = getAngleFromAnalog();
        angleOffsetDegrees = normalizeAngleDegrees(-raw);
    }

    public void goToOuttakePosition() {
        startMoveToAngle(OUTTAKE_ANGLE);
    }


    // --- Control Loop ---

    public void startMoveToAngle(double targetDegrees) {
        // Cancel any spin mode when going to a specific angle
        spinModeActive = false;
        referenceAngle = normalizeAngleDegrees(targetDegrees);
        integralSum = 0.0;
        timer.reset();
        runtimeTimer.reset();

        // Seed the last measured angle so derivative doesn't spike on first frame
        lastMeasuredAngle = getCalibratedAngle();
    }

    /**
     * Start an open-loop spin for a given number of degrees (absolute, positive). The controller
     * will drive the motor at the provided power until the accumulated rotation reaches target.
     * This is useful when you want the spindexer to make multiple full revolutions.
     */
    public void startSpinDegrees(double degrees, double power) {
        if (degrees <= 0) return;
        cancelAccurateColorScan();
        spinModeActive = true;
        spinTargetDegrees = degrees;
        spinAccumulatedDegrees = 0.0;
        spinPower = Math.max(0.0, Math.min(1.0, Math.abs(power)));
        spinDirection = 1; // default: positive direction

        // Seed lastMeasuredAngle so first delta is small/accurate
        lastMeasuredAngle = getCalibratedAngle();
        timer.reset();
    }

    public void startSpin720(double power) {
        // Reasonable default power; adjust if needed
        startSpinDegrees(720.0, power);
    }

    public boolean isSpinInProgress() {
        return spinModeActive;
    }

    private boolean pidInitialized = false;

    public boolean update() {
        double dt = timer.seconds();
        timer.reset();

        // Protect against tiny / bad dt
        if (dt <= 1e-4) dt = 1e-4;
        lastDt = dt;

        final double BASE_TOL = 15.0;
        final double MIN_TOL = 3.0;
        final double VELOCITY_FACTOR = 0.04;

        // Optional: helps overcome static friction from rest
        final double MIN_MOVE_POWER = 0.12; // tune this, maybe 0.08 to 0.18

        double currentAngle = getCalibratedAngle();
        lastCurrentAngle = currentAngle;

        // Initialize derivative state on first loop
        if (!pidInitialized) {
            lastMeasuredAngle = currentAngle;
            pidInitialized = true;
        }

        // Compute angular change/velocity once
        double angleDelta = smallestAngleDifference(currentAngle, lastMeasuredAngle);
        double velocity = angleDelta / dt;
        lastMeasuredAngle = currentAngle;
        lastVelocity = velocity;

        // If spin mode is active, accumulate absolute rotation and drive motor open-loop
        if (spinModeActive) {
            spinAccumulatedDegrees += Math.abs(angleDelta);
            // Drive motor directly at spinPower in chosen direction
            spindexerMotor.setPower(spinDirection * spinPower);

            // Check completion
            if (spinAccumulatedDegrees >= spinTargetDegrees) {
                // Stop spinning and restore PID reference to current heading
                spinModeActive = false;
                spindexerMotor.setPower(0.0);
                referenceAngle = normalizeAngleDegrees(currentAngle);
                integralSum = 0.0;
                // Reset timers/state to avoid derivative spikes
                timer.reset();
                pidInitialized = true;
            }

            // Still return to allow telemetry etc.
            updateColorScan(currentAngle);
            return true;
        }

        // Update color scan using current angle
        updateColorScan(currentAngle);

        // -------------------------------
        // Ball detection logic
        // -------------------------------
        if (distanceSensor.getState() && isDetectionEnabled()) {
            double adaptiveTol = Math.max(MIN_TOL, BASE_TOL - VELOCITY_FACTOR * Math.abs(velocity));
            lastAdaptiveTol = adaptiveTol;

            for (int i = 0; i < 3; i++) {
                if (Math.abs(smallestAngleDifference(currentAngle, INTAKE_ANGLES[i])) < adaptiveTol) {
                    if (filled[i] == '_') {
                        filled[i] = 'X';

                        if (!isFull()) {
                            advanceIntake();
                        }
                    }
                    break;
                }
            }
        }

        // -------------------------------
        // PID control
        // -------------------------------
        double error = smallestAngleDifference(referenceAngle, currentAngle);
        lastError = error;

        // Integral zoning
        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        double pTerm = Kp * error;
        double iTerm = Ki * integralSum;
        double dTerm = -Kd * velocity; // derivative on measurement
        double fTerm = (Math.abs(error) > 0.5) ? Math.signum(error) * kStatic : 0.0;

        double out = pTerm + iTerm + dTerm + fTerm;

        // Minimum power to break stiction when error is meaningful
        if (Math.abs(error) > 2.0 && Math.abs(out) < MIN_MOVE_POWER) {
            out = Math.signum(error) * MIN_MOVE_POWER;
        }

        // Clamp
        out = Math.max(-1.0, Math.min(1.0, out));

        lastOutput = out;
        spindexerMotor.setPower(out);

        return true;
    }

    // --- Telemetry helpers ---
    public double getLastVelocity() { return lastVelocity; }
    public double getLastAdaptiveTol() { return lastAdaptiveTol; }
    public double getLastCurrentAngle() { return lastCurrentAngle; }
    public double getLastError() { return lastError; }
    public double getLastOutput() { return lastOutput; }
    public double getLastDt() { return lastDt; }


    public double getReferenceAngle() { return referenceAngle; }

    // --- Tracking & Positions ---

    public void startAccurateColorScan() {
        if (colorSensor == null || colorScanState != ColorScanState.IDLE) {
            return;
        }

        preScanReferenceAngle = referenceAngle;
        colorScanIndex = -1;
        if (!moveToNextScannableSlot()) {
            colorScanState = ColorScanState.IDLE;
        }
    }

    public boolean isAccurateColorScanInProgress() {
        return colorScanState != ColorScanState.IDLE;
    }

    public void cancelAccurateColorScan() {
        if (colorScanState == ColorScanState.IDLE) {
            return;
        }

        colorScanState = ColorScanState.IDLE;
        colorScanIndex = -1;
        resetColorScanCounts();
        startMoveToAngle(preScanReferenceAngle);
    }

    public void setIntakeIndex(int index) {
        cancelAccurateColorScan();
        intakeIndex = index % 3;
        if (intakeIndex < 0) intakeIndex += 3;
        startMoveToAngle(INTAKE_ANGLES[intakeIndex]);
    }

    public void advanceIntake() {
        intakeIndex = (intakeIndex + 1) % 3;
        setIntakeIndex(intakeIndex);
    }

    public void retreatIntake() {
        intakeIndex = (intakeIndex + 2) % 3; // equivalent to -1
        setIntakeIndex(intakeIndex);
    }

    public void setShootIndex(int index) {
        cancelAccurateColorScan();
        index %= 3;
        if (index < 0) index += 3;
        shootIndex = index;
        startMoveToAngle(SHOOT_ANGLES[index]);
    }

    public void advanceShoot() {
        shootIndex = (shootIndex + 1) % 3;
        setShootIndex(shootIndex);
    }

    public void retreatShoot() {
        shootIndex = (shootIndex + 2) % 3; // equivalent to -1
        setShootIndex(shootIndex);
    }
    public void setColorAtPos(char color, int index) {
        if (index >= 0 && index < 3) filled[index] = color;
    }

    public void setColorAtPos(char color) {
        setColorAtPos(color, intakeIndex);
    }

    public boolean isFull() {
        for (char c : filled) {
            if (c == '_') return false;
        }
        return true;
    }

    public boolean isEmpty() {
        for (char c : filled) {
            if (c == '_') return true;
        }
        return false;
    }

    public int getBalls(){
        int ret = 0;
        for (char c : filled) {
            if (c != '_') ret++;
        }
        return ret;
    }

    public void clearTracking() {
        cancelAccurateColorScan();
        filled[0] = '_';
        filled[1] = '_';
        filled[2] = '_';
    }

    public int getIntakeIndex() {
        return intakeIndex;
    }

    public int getShootIndex() {
        return shootIndex;
    }

    public char[] getFilled() {
        return filled;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(smallestAngleDifference(referenceAngle, getCalibratedAngle())) < tolerance;
    }

    // --- Helpers ---
    private boolean isDetectionEnabled() {
        if (intakeFlap == null) {
            return true;
        }

        boolean flapOn = intakeFlap.isOn();
        if (!flapOn) {
            lastFlapOn = false;
            return false;
        }

        if (!lastFlapOn) {
            lastFlapOn = true;
            flapOnTimer.reset();
            return false;
        }

        return flapOnTimer.milliseconds() >= FLAP_ON_DETECTION_DELAY_MS;
    }

    private void updateColorScan(double currentAngle) {
        if (colorScanState == ColorScanState.IDLE || colorSensor == null) {
            return;
        }

        double targetAngle = COLOR_SCAN_ANGLES[colorScanIndex];
        boolean atScanAngle = Math.abs(smallestAngleDifference(currentAngle, targetAngle)) <= COLOR_SCAN_POSITION_TOLERANCE_DEG;

        switch (colorScanState) {
            case MOVING:
                if (atScanAngle) {
                    colorScanState = ColorScanState.SAMPLING;
                    colorScanTimer.reset();
                    resetColorScanCounts();
                }
                break;

            case SAMPLING:
                if (!atScanAngle) {
                    colorScanState = ColorScanState.MOVING;
                    break;
                }

                recordColorSample(colorSensor.detection());
                if (colorScanTimer.milliseconds() >= COLOR_SCAN_SETTLE_MS) {
                    filled[colorScanIndex] = chooseScannedColor();
                    colorScanState = ColorScanState.WAITING_BETWEEN_SLOTS;
                    colorScanTimer.reset();
                }
                break;

            case WAITING_BETWEEN_SLOTS:
                if (colorScanTimer.milliseconds() >= COLOR_SCAN_NEXT_SLOT_DELAY_MS) {
                    if (!moveToNextScannableSlot()) {
                        finishAccurateColorScan();
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    private boolean moveToNextScannableSlot() {
        for (int i = colorScanIndex + 1; i < filled.length; i++) {
            if (filled[i] != '_') {
                colorScanIndex = i;
                colorScanState = ColorScanState.MOVING;
                resetColorScanCounts();
                startMoveToAngle(COLOR_SCAN_ANGLES[colorScanIndex]);
                return true;
            }
        }

        colorScanState = ColorScanState.IDLE;
        return false;
    }

    private void finishAccurateColorScan() {
        colorScanState = ColorScanState.IDLE;
        colorScanIndex = -1;
        resetColorScanCounts();
        startMoveToAngle(preScanReferenceAngle);
    }

    private void resetColorScanCounts() {
        scanGreenCount = 0;
        scanPurpleCount = 0;
        scanUnknownCount = 0;
    }

    private void recordColorSample(char sample) {
        switch (sample) {
            case 'G':
                scanGreenCount++;
                break;
            case 'P':
                scanPurpleCount++;
                break;
            default:
                scanUnknownCount++;
                break;
        }
    }

    private char chooseScannedColor() {
        if (scanGreenCount >= scanPurpleCount && scanGreenCount >= scanUnknownCount) {
            return 'G';
        }
        if (scanPurpleCount >= scanGreenCount && scanPurpleCount >= scanUnknownCount) {
            return 'P';
        }
        return 'X';
    }

    private double normalizeAngleDegrees(double a) {
        double res = a % 360.0;
        if (res < 0) res += 360.0;
        return res;
    }

    private double smallestAngleDifference(double target, double current) {
        return Math.IEEEremainder(target - current, 360.0);
    }
}
