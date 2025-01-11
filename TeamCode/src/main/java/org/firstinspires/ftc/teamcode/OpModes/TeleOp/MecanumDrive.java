package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Commands.automation.Random;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.drive.DriveCommand;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.drive.SetDriveMultiplier;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.ManualJointControl;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.ManualSlideControl;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.motors.SetJoint;
import org.firstinspires.ftc.teamcode.Core.Commands.subsystems.servos.SetClaw;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.util.Constants;

@TeleOp
@Disabled
public class MecanumDrive extends CommandOpMode {
    private Robot robot;
    private GamepadEx driver, operator;

    @Override
    public void initialize() {
        robot = Robot.getInstance();

        robot.initialize(hardwareMap, telemetry);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        DriveCommand driveCommand = new DriveCommand(
                robot.drive,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> driver.getRightX()

        );

        register(robot.drive);
        robot.drive.setDefaultCommand(driveCommand);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new SetDriveMultiplier(robot.drive, Constants.driveFastMultiplier),
                                new SetDriveMultiplier(robot.drive, Constants.driveSlowMultiplier),
                                () -> {
                                    robot.drive.toggle();
                                    return robot.drive.getState();
                                }
                        ));



        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SetClaw(robot.claw, Constants.clawOpenPosition))
                .whenReleased(new SetClaw(robot.claw, Constants.clawClosedPosition));
        register(robot.claw);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new Random(robot));

        /* ------- GAMEPAD 2 ------- */

        ManualJointControl jointControl = new ManualJointControl(
                robot.joint,
                () -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        );

        register(robot.joint);
        robot.joint.setDefaultCommand(jointControl);

        ManualSlideControl slideControl = new ManualSlideControl(
                robot.slide,
                () -> operator.getLeftY()
        );

        register(robot.slide);
        robot.slide.setDefaultCommand(slideControl);

    }
}
