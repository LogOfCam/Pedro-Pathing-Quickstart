package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Core.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class Robot extends Follower {

    public Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public Slide slide;
    public Claw claw;

    private static Robot instance = null;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {
        initializePedro(hardwareMap); // initialize Pedro

        telemetry = telemetry_;

        this.claw = new Claw(hardwareMap);
        this.slide = new Slide(hardwareMap);

        CommandScheduler.getInstance().reset();
    }

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        return instance;
    }
}
