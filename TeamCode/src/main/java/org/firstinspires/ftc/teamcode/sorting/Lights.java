package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    Servo lightLeft;
    Servo lightRight;
    Servo lightBack;
    public Lights(HardwareMap hardwareMap){
        lightLeft = hardwareMap.get(Servo.class, "lightLeft");
        lightRight = hardwareMap.get(Servo.class, "lightRight");
        lightBack = hardwareMap.get(Servo.class, "lightBack");
    }

    public void turnAllOn(){
        lightLeft.setPosition(1);
        lightRight.setPosition(1);
        lightBack.setPosition(1);
    }

    public void turnAllOff(){
        lightLeft.setPosition(0);
        lightRight.setPosition(0);
        lightBack.setPosition(0);
    }
}
