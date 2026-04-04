package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.intake.IntakeServo;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;

@TeleOp(name = "Kicker Servo Test", group = "Test")
public class KickerServoTest extends OpMode {
    private static final double NORMAL_POSITION = 0.52;
    private static final double KICK_POSITION = 0.40;
    private static final double STEP = 0.01;

    private IntakeServo kickerServo;
    private double currentPosition = NORMAL_POSITION;

    @Override
    public void init() {
        kickerServo = new IntakeServo(hardwareMap, "intakeServo");
        currentPosition = NORMAL_POSITION;
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            kickerServo.intake();
            currentPosition = KICK_POSITION;
        } else if (gamepad1.bWasPressed()) {
            kickerServo.outtake();
            currentPosition = NORMAL_POSITION;
        }

        if (gamepad1.dpadUpWasPressed()) {
            currentPosition = Math.min(1.0, currentPosition + STEP);
            kickerServo.setPosition(currentPosition);
        } else if (gamepad1.dpadDownWasPressed()) {
            currentPosition = Math.max(0.0, currentPosition - STEP);
            kickerServo.setPosition(currentPosition);
        }

        telemetry.addData("Current Position", String.format(Locale.US, "%.2f", currentPosition));
        telemetry.addData("Controls", "A: Intake, B: Outtake, Dpad Up/Down: Adjust");
        telemetry.update();
    }
}
