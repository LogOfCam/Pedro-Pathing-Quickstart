package org.firstinspires.ftc.teamcode.Core.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide extends SubsystemBase {
    private DcMotorEx slideMotor;
    private PIDController controller;
    double p = 0.006, i = 0, d = 0.0001;
    double f = 0.04;
    double ticksInDegrees = 358.466 / 180;
    int threshold = 50;
    private double targetPosition = 0;
    private boolean isHolding = false;

    public Slide(HardwareMap hardwareMap){
        slideMotor = hardwareMap.get(DcMotorEx.class,"slideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p,i,d);
    }

    public void setTargetPosition(int target) {
        targetPosition = target;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        int currentPosition = slideMotor.getCurrentPosition();
        double pid = controller.calculate(currentPosition, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;
        double power = pid + ff;//double power = Math.min(1.0, Math.min(1.0, pid + ff));
        setSlidePower(power);
    }

    public void setSlidePower(double power) {
        slideMotor.setPower(power);
    }
}
