package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.InternalPosition;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@TeleOp(name = "DriveOnly", group = "Testing")
@Config
public class DriveOnly extends CommandOpMode{
    GamepadEx driver1, driver2;

    public static States.Alliance alliance = States.Alliance.Red;
    public static double driveMult = 1;

    @Override
    public void initialize(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);

        telemetry.addData("Test Instructions", "Place the robot at the center of the field");
        telemetry.update();

        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90))), telemetry);
        //InternalPosition positionCalc = new InternalPosition(drive::getPose, ()->0);
        // Drive
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver1.getLeftX() * driveMult,
                () -> -driver1.getLeftY() * driveMult,
                () -> -driver1.getRightX() * driveMult,
                true);

//        Command turnCommand = new ActionCommand(
//                drive.actionBuilder(drive.getPose())
//                        .turnTo(positionCalc.autoGetAngle(alliance))
//                        .build(),
//
//                Stream.of(drive).collect(Collectors.toSet())
//        );
//
//        new GamepadButton(driver1, GamepadKeys.Button.X)
//                .whenPressed(() -> DriveCommand.driving = false)
//                .whileHeld(turnCommand)
//                .whenReleased(() -> {
//                    DriveCommand.driving = true;
//                });

        new GamepadButton(driver1, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> StateTransfer.alliance = States.Alliance.Red));

        new GamepadButton(driver1, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> StateTransfer.alliance = States.Alliance.Blue));

        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> drive.drive.localizer.setPose(new Pose2d(drive.getPose().position, Math.toRadians(90)))));
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> drive.drive.localizer.setPose(new Pose2d(drive.getPose().position, Math.toRadians(180)))));
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> drive.drive.localizer.setPose(new Pose2d(drive.getPose().position, Math.toRadians(270)))));
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> drive.drive.localizer.setPose(new Pose2d(drive.getPose().position, Math.toRadians(0)))));


        schedule(driveCommand);
    }
}
