package org.firstinspires.ftc.teamcode.Core.Subsystems.servos;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Basket extends SubsystemBase {
    private final Servo basket;
    public Basket(HardwareMap hardwareMap) {
        basket = hardwareMap.get(Servo.class,"basketServo");
    }

    public void setPosition(double targetPosition) {
        basket.setPosition(targetPosition);
    }

    public double getPosition() {
        return basket.getPosition();
    }
}