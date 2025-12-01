package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.MacroCommands;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.InternalPosition;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.stream.Collectors;
import java.util.stream.Stream;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode{
    GamepadEx driver1, driver2;

    DriveSubsystem drive;

    IntakeSubsystem intake;
    OuttakeSubsystem outtake;
    TurretSubsystem turret;
    private InternalPosition positionCalc;

    public static double driveMult = 1;

    @Override
    public void initialize(){



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, StateTransfer.pose), telemetry);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        outtake = new OuttakeSubsystem(hardwareMap, telemetry, true);

        //turret = new TurretSubsystem(hardwareMap, telemetry);

        positionCalc = new InternalPosition(drive::getPose, () ->0);

        outtake.setDefaultCommand(new RunCommand(outtake::holdSpeed, outtake));
        intake.setDefaultCommand(new RunCommand(() ->intake.setPower(driver2.getLeftY()*12), intake));

        // Drive
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver1.getLeftX() * driveMult,
                () -> -driver1.getLeftY() * driveMult,
                () -> -driver1.getRightX() * driveMult,
                true);

//        new GamepadButton(driver1, GamepadKeys.Button.X)
//                .whenPressed(
//                        new ActionCommand(
//                                drive.actionBuilder(drive.getPose())
//                                        .turnTo(positionCalc.autoGetAngle(StateTransfer.alliance))
//                                        .build(),
//
//                                Stream.of(drive).collect(Collectors.toSet())
//                        )
//                );

        // To be used in macro
        SequentialCommandGroup shiftBalls = new SequentialCommandGroup(
                new WaitCommand(600),
                new InstantCommand(() -> intake.setPower(12), intake),
                new WaitCommand(600),
                new InstantCommand(() -> intake.setPower(0), intake)
        );

        // Position reset
        new GamepadButton(driver1, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> drive.drive.localizer.setPose(new Pose2d(0,0,0))));

        // Flywheel Control
        new GamepadButton(driver2, GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(() -> outtake.setPower(OuttakeSubsystem.flywheelVelocity)),
                        new InstantCommand(() -> outtake.setPower(0.0)));


        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(outtake::resetMotor));

        // Kicker Control
        new GamepadButton(driver2, GamepadKeys.Button.B)
                .whenPressed(
                        MacroCommands.kickSequence(outtake));


        //Macro to outtake 3 balls
        new GamepadButton(driver2, GamepadKeys.Button.X)
                .whenPressed(MacroCommands.launchSequence(outtake, intake));




        // Drive Command
        schedule(driveCommand);
    }
}
