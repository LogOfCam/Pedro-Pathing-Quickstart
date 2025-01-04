package org.firstinspires.ftc.teamcode.Core.Subsystems.motors;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Joint extends SubsystemBase {
    private final DcMotorEx jointMotor;
    private final PIDController controller;
    double p = 0.004, i = 0, d = 0.0002;
    double f = 0.03;
    double ticksInDegrees = 285 / 180;
    private double targetPosition;

    public Joint(HardwareMap hardwareMap){
        jointMotor = hardwareMap.get(DcMotorEx.class,"jointMotor");
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDController(p,i,d);
    }

    public void setTargetPosition(double targetPosition) {

        this.targetPosition = targetPosition;
    }

    @Override
    public void periodic() {
        updateJoint();
    }

    public void updateJoint() {
        // This method will be called once per scheduler run
        int currentPosition = jointMotor.getCurrentPosition();
        double pid = controller.calculate(currentPosition, targetPosition);
        //double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;
        double power = pid + f;
        setPower(power);

    }

    public double getTargetPosition() {
        return targetPosition;
    }
    public double getActualTargetPosition() {return jointMotor.getTargetPosition(); }
    public double getCurrentPosition() {
        return jointMotor.getCurrentPosition();
    }
    public double getPower() {
        return jointMotor.getPower();
    }

    public void setPower(double power) {
        jointMotor.setPower(power);
    }
}
