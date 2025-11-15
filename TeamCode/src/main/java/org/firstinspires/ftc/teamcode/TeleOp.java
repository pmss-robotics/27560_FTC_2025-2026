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
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode{
    GamepadEx driver1, driver2;

    DriveSubsystem drive;

    IntakeSubsystem intake;
    OuttakeSubsystem outtake;

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

        outtake = new OuttakeSubsystem(hardwareMap, telemetry, /*TODO: once kicker is built change to true*/ true);


        outtake.setDefaultCommand(new RunCommand(outtake::holdSpeed, outtake));
        intake.setDefaultCommand(new RunCommand(() ->intake.setPower(driver2.getLeftY()*12), intake));

        // Drive
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver1.getLeftX() * driveMult,
                () -> driver1.getLeftY() * driveMult,
                () -> -driver1.getRightX() * driveMult,
                true);

        // To be used in macro
        SequentialCommandGroup shiftBalls = new SequentialCommandGroup(
                new WaitCommand(600),
                new InstantCommand(() -> intake.setPower(12), intake),
                new WaitCommand(600),
                new InstantCommand(() -> intake.setPower(0), intake)
        );


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
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.kick()),
                                new WaitCommand(500),
                                new InstantCommand(() -> outtake.home())
                        ));


        //Macro to outtake 3 balls
        new GamepadButton(driver2, GamepadKeys.Button.X)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.setPower(OuttakeSubsystem.flywheelVelocity), outtake),
                                new WaitCommand(1200),

                                new InstantCommand(() -> intake.setPower(12), intake),
                                new WaitCommand(400),
                                new InstantCommand(() -> intake.setPower(0), intake),
                                new WaitCommand(800),

                                new InstantCommand(() -> intake.setPower(12), intake),
                                new WaitCommand(400),
                                new InstantCommand(() -> intake.setPower(0), intake),
                                new WaitCommand(800),

                                new InstantCommand(() -> outtake.kick()),
                                new WaitCommand(500),
                                new InstantCommand(() -> outtake.home()),
                                new InstantCommand(() -> outtake.setPower(0), outtake)

                        ));




        // Drive Command
        schedule(driveCommand);
    }
}
