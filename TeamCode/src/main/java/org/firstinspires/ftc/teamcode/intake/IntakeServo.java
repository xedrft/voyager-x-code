package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServo {
    private Servo servo;

    // Position constants
    private static final double INTAKE_POSITION = 0.5;
    private static final double OUTTAKE_POSITION = 0.43;

    public IntakeServo(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(INTAKE_POSITION);
    }

    public void intake() {
        servo.setPosition(INTAKE_POSITION);
    }

    public void outtake() {
        servo.setPosition(OUTTAKE_POSITION);
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }
}
