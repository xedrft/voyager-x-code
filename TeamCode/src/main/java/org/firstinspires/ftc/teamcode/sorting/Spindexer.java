package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Spindexer {
    private final DcMotorEx spindexerMotor;
    private final AnalogInput analogEncoder;
    private final DigitalChannel distanceSensor;

    // Telemetry / diagnostics
    private double lastVelocity = 0.0;
    private double lastAdaptiveTol = 0.0;
    private double lastCurrentAngle = 0.0;
    private double lastError = 0.0;
    private double lastOutput = 0.0;
    private double lastDt = 0.0;

    // --- PIDF Coefficients ---
    // Start with these. If it oscillates, lower Kp. If it stops short, raise kStatic.
    public static double Kp = 0.0131;
    public static double Ki = 0.001;
    public static double Kd = 0.0008;
    public static double kStatic = 0.048; // Minimum power to overcome friction
    public ColorSensor colorSensor;

    // PID state
    private double integralSum = 0.0;
    private double lastMeasuredAngle = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime runtimeTimer = new ElapsedTime();
    private final ElapsedTime detectionDelayTimer = new ElapsedTime();

    private double referenceAngle = 0.0;

    // Settings
    private static final double ANALOG_MAX_VOLTAGE = 3.3;
    private static final double DETECTION_BASE_TOL = 20.0;
    private static final double DETECTION_MIN_TOL = 3.0;
    private static final double DETECTION_VELOCITY_FACTOR = 0.03;
    public static double DETECTION_DELAY_MS = 200.0;

    // Calibration
    private double angleOffsetDegrees = 30.0;

    // Detection delay / latching
    private boolean detectionPending = false;
    private boolean lastDistanceSensorState = false;

    // Tracking
    public char[] filled = {'_', '_', '_'};
    private int intakeIndex = 0;
    private int shootIndex = 0;

    // Positions (Degrees)
    // Intake: 0, 120, 240
    public static final double[] INTAKE_ANGLES = {240, 120.0, 0.0};
    // Shoot: 180 (0.5), 300 (0.833), 60 (0.167)
    public static final double[] SHOOT_ANGLES = {240.0, 120.0, 0.0};

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName, String distanceSensorName, ColorSensor colorSensor) {
        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.analogEncoder = hardwareMap.get(AnalogInput.class, analogName);
        this.distanceSensor = hardwareMap.get(DigitalChannel.class, distanceSensorName);
        this.distanceSensor.setMode(DigitalChannel.Mode.INPUT);

        spindexerMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.colorSensor = colorSensor;
    }

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName, String distanceSensorName) {
        this(hardwareMap, motorName, analogName, distanceSensorName, null);
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


    // --- Control Loop ---

    public void startMoveToAngle(double targetDegrees) {
        referenceAngle = normalizeAngleDegrees(targetDegrees);
        integralSum = 0.0;
        timer.reset();
        runtimeTimer.reset();

        // Seed the last measured angle so derivative doesn't spike on first frame
        lastMeasuredAngle = getCalibratedAngle();
    }

    public boolean update() {
        // Ball detection logic
        double dt = timer.seconds();
        timer.reset();
        lastDt = dt;

        double currentAngle = getCalibratedAngle();
        lastCurrentAngle = currentAngle;
        double velocity = smallestAngleDifference(currentAngle, lastMeasuredAngle) / Math.max(dt, 1e-6);
        lastVelocity = velocity;
        boolean distanceDetected = distanceSensor.getState();

        if (distanceDetected && !lastDistanceSensorState && !detectionPending) {
            detectionPending = true;
            detectionDelayTimer.reset();
        }

        if (detectionPending && detectionDelayTimer.milliseconds() >= DETECTION_DELAY_MS) {
            if (handleDelayedDetection(currentAngle, velocity)) {
                detectionPending = false;
            }
        }

        if (!distanceDetected && !detectionPending) {
            lastAdaptiveTol = 0.0;
        }

        lastDistanceSensorState = distanceDetected;

        double error = smallestAngleDifference(referenceAngle, currentAngle);
        lastError = error;

        if (dt <= 0) dt = 1e-6; // safety

        // 1. Integral Zoning: Only integrate if error is small (prevents windup)
        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        // 2. Derivative on Measurement: Calculates velocity directly
        // (Avoids "kick" when changing target)
        double velocity2 = smallestAngleDifference(currentAngle, lastMeasuredAngle) / dt;
        lastMeasuredAngle = currentAngle;

        // 3. Calculate Terms
        double pTerm = Kp * error;
        double iTerm = Ki * integralSum;
        double dTerm = -Kd * velocity2; // Negative because it opposes motion

        // 4. Feedforward (kStatic): Helps overcome friction near target
        double fTerm = Math.signum(error) * kStatic;

        double out = pTerm + iTerm + dTerm + fTerm;

        // Clamp
        if (out > 1.0) out = 1.0;
        if (out < -1.0) out = -1.0;

        lastOutput = out;
        spindexerMotor.setPower(out);

        return true;
    }

    private boolean handleDelayedDetection(double currentAngle, double velocity) {
        // adaptive tolerance: reduce base tolerance by factor * |velocity|, but clamp
        double adaptiveTol = Math.max(DETECTION_MIN_TOL, DETECTION_BASE_TOL - DETECTION_VELOCITY_FACTOR * Math.abs(velocity));
        lastAdaptiveTol = adaptiveTol;

        for (int i = 0; i < 3; i++) {
            if (Math.abs(smallestAngleDifference(currentAngle, INTAKE_ANGLES[i])) < adaptiveTol) {
                // Ball detected at slot i after the delay
                if (filled[i] == 'X' && colorSensor != null) {
                    filled[i] = colorSensor.detection();
                }

                if (filled[i] == '_') {
                    if (colorSensor != null) filled[i] = colorSensor.detection();
                    else filled[i] = 'X';

                    if (!isFull()) {
                        advanceIntake();
                    }
                }
                return true;
            }
        }

        return false;
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

    public void setIntakeIndex(int index) {
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

    public int getBalls(){
        int ret = 0;
        for (char c : filled) {
            if (c != '_') ret++;
        }
        return ret;
    }

    public void clearTracking() {
        filled[0] = '_';
        filled[1] = '_';
        filled[2] = '_';
        resetDetectionState();
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
    private double normalizeAngleDegrees(double a) {
        double res = a % 360.0;
        if (res < 0) res += 360.0;
        return res;
    }

    private void resetDetectionState() {
        detectionPending = false;
        lastDistanceSensorState = distanceSensor.getState();
        lastAdaptiveTol = 0.0;
    }

    private double smallestAngleDifference(double target, double current) {
        return Math.IEEEremainder(target - current, 360.0);
    }
}

