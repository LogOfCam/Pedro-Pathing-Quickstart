package org.firstinspires.ftc.teamcode.Core.Subsystems.motors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.util.Constants;

public class Slide extends SubsystemBase {
    private final DcMotorEx slideMotor;
    private final PIDController controller;
    double p = 0.01, i = 0, d = 0.0; // Was 0.006 and .0001
    double f = 0.005;
    double ticksInDegrees = 358.466 / 180;
    private double targetPosition;
    public Slide(HardwareMap hardwareMap){
        slideMotor = hardwareMap.get(DcMotorEx.class,"slideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p,i,d);
    }

    public void setTargetPosition(double targetPosition) {

        this.targetPosition = Math.max(Constants.slideMinPosition, (Math.min(Constants.slideMaxPosition, targetPosition)));;
    }

    @Override
    public void periodic() {
        updateSlide();
    }

    public void updateSlide() {
        // This method will be called once per scheduler run
        int currentPosition = slideMotor.getCurrentPosition();
        double pid = controller.calculate(currentPosition, targetPosition);
        //double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;
        double power = pid + f; //double power = Math.min(1.0, Math.min(1.0, pid + ff));
        setPower(power);
    }

    public double getActualTargetPosition() {
        return slideMotor.getTargetPosition();
    }
    public double getTargetPosition() {return targetPosition; }
    public double getCurrentPosition() {
        return slideMotor.getCurrentPosition();
    }
    public double getPower() { return slideMotor.getPower(); }

    public void setPower(double power) {
        slideMotor.setPower(power);
    }
}
