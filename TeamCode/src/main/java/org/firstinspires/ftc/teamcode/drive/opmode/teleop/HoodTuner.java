package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Hood Tuner", group = "Tuning")
public class HoodTuner extends OpMode {

    private Turret turret;

    private double hoodPosition = 0.39;
    private static final double INCREMENT = 0.01;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", false, false);
        turret.setHoodPosition(hoodPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            hoodPosition = Math.min(1.0, hoodPosition + INCREMENT);
            turret.setHoodPosition(hoodPosition);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            hoodPosition = Math.max(0.39, hoodPosition - INCREMENT);
            turret.setHoodPosition(hoodPosition);
        }

        telemetry.addData("Hood Position", String.format(java.util.Locale.US, "%.3f", hoodPosition));
        telemetry.addData("Controls", "DPAD UP = increase, DPAD DOWN = decrease");
        telemetry.update();
    }
}
