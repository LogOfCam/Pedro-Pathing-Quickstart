package org.firstinspires.ftc.teamcode.Core.Subsystems.servos;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {
    private final Servo wrist;
    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class,"wristServo");
    }

    public void setPosition(double targetPosition) {
        wrist.setPosition(targetPosition);
    }

    public double getPosition() {
        return wrist.getPosition();
    }
}