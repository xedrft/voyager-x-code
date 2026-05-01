package org.firstinspires.ftc.teamcode.drive.opmode.teleop.BS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Turret Installation TeleOp", group = "TeleOp")
public class TurretInstallationTeleOp extends OpMode {
    private Turret turret;
    private Servo turretServo;
    private double currentPos = 0;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", "hoodServo", false, false);
        turretServo = hardwareMap.get(Servo.class, "turret");
        turretServo.setPosition(currentPos);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            currentPos = Math.min(currentPos + 0.01, 1);
        }

        if (gamepad1.dpadDownWasPressed()) {
            currentPos = Math.max(currentPos - 0.01, 0);
        }

        turretServo.setPosition(currentPos);

        telemetry.addData("Turret Degrees", currentPos);
        telemetry.addData("Turret Voltage", turret.getTurretVoltage());
        telemetry.update();
    }

}
