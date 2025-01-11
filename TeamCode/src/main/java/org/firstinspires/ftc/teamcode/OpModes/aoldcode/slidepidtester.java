package org.firstinspires.ftc.teamcode.OpModes.aoldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class slidepidtester extends OpMode {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0; // test github 
    public static double f = 0.005;
    public static int target = 0;
    private final double ticksInDegree = 358.466 / 180; //1425
    private DcMotorEx slideMotor;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int slidePos = slideMotor.getCurrentPosition();
        double pid = controller.calculate(slidePos,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + f;
        slideMotor.setPower(power);

        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
