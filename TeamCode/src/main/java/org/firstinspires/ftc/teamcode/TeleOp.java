package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.States;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode{
    GamepadEx driver1, driver2;

    DriveSubsystem drive;

    IntakeSubsystem intake;
    OuttakeSubsystem flywheel;

    public static double driveMult = 1;

    @Override
    public void initialize(){



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        flywheel = new OuttakeSubsystem(hardwareMap, telemetry, /*TODO: once kicker is built change to true*/ false);


        flywheel.setDefaultCommand(new RunCommand(flywheel::holdSpeed, flywheel));
        intake.setDefaultCommand(new RunCommand(() ->intake.setPower(driver2.getLeftY()), intake));

        // Drive
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver1.getLeftX() * driveMult,
                () -> driver1.getLeftY() * driveMult,
                () -> -driver1.getRightX() * driveMult,
                true);


        // Flywheel Control
        new GamepadButton(driver2, GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(() -> flywheel.setPower(0.0), flywheel),
                        new InstantCommand(() -> flywheel.setPower(OuttakeSubsystem.flywheelVelocity), flywheel));

        // Kicker Control

        /* TODO: Un-comment this section to allow kicker controls
        new GamepadButton(driver2, GamepadKeys.Button.B)
                .toggleWhenPressed(flywheel::kick, flywheel::home);
        */



        // Drive Command
        schedule(driveCommand);
    }
}
