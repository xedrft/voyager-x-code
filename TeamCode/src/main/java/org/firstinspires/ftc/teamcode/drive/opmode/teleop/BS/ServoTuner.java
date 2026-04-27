package org.firstinspires.ftc.teamcode.drive.opmode.teleop.BS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Servo Tuner", group = "Tuning")
public class ServoTuner extends OpMode {

    private Turret turret;

    private double Position = 0.39;
    private static final double INCREMENT = 0.01;
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "intakeFlapServo");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            Position = Math.min(1.0, Position + INCREMENT);
            servo.setPosition(Position);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            Position = Math.max(0.0, Position - INCREMENT);
            servo.setPosition(Position);
        }

        telemetry.addData("Hood Position", String.format(java.util.Locale.US, "%.3f", Position));
        telemetry.addData("Controls", "DPAD UP = increase, DPAD DOWN = decrease");
        telemetry.update();
    }
}
