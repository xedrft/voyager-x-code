package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Turret Installation TeleOp", group = "TeleOp")
public class TurretInstallationTeleOp extends OpMode {
    private Turret turret;
    private double currentDegrees = 180;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);
        turret.goToPosition(currentDegrees);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            currentDegrees = Math.min(currentDegrees + 10, 360);
        }

        if (gamepad1.dpadDownWasPressed()) {
            currentDegrees = Math.max(currentDegrees - 10, 0);
        }

        turret.goToPosition(currentDegrees);

        telemetry.addData("Turret Degrees", currentDegrees);
        telemetry.addData("Turret Voltage", turret.getTurretVoltage());
        telemetry.update();
    }

}
