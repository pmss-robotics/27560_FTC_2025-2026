package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.MacroCommands;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.InternalPosition;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Collections;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp RoadRunner")
public class TeleOp extends CommandOpMode{
    GamepadEx driver1, driver2;

    DriveSubsystem drive;

    IntakeSubsystem intake;
    OuttakeSubsystem outtake;
    TurretSubsystem turret;


    TurretVisionSubsystem turretVision;
    LoopTimer timer;
    boolean autoRotate;
    private InternalPosition positionCalc;

    public static double driveMult = 1;

    @Override
    public void initialize(){



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        if (Settings.get("loop_detect_mode", false)) {
            timer = new LoopTimer(telemetry, "Main");
        }

        try {
            turretVision = new TurretVisionSubsystem(hardwareMap, telemetry, false);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        turretVision.enableDetection(true);

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, StateTransfer.pose), telemetry);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        outtake = new OuttakeSubsystem(hardwareMap, telemetry, true);

        turret = new TurretSubsystem(hardwareMap, telemetry);

        //positionCalc = new InternalPosition(drive::getPose, () ->0);

        outtake.setDefaultCommand(new RunCommand(outtake::holdSpeed, outtake));
        intake.setDefaultCommand(new RunCommand(() ->intake.setPower(driver2.getLeftY()*12), intake));

        turret.setDefaultCommand(new RunCommand(
                () -> turret.turnTo(turretVision.update(), () -> !driver2.gamepad.right_stick_button), turret, turretVision)
        );


        // Drive
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> driver1.getLeftX() * driveMult,
                () -> driver1.getLeftY() * driveMult,
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


        // Position reset
        new GamepadButton(driver1, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> drive.drive.localizer.setPose(new Pose2d(0,0, Math.toRadians(90)))));


        // Drive Speed Toggle
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(
                        new InstantCommand(() -> driveMult = 0.5),
                        new InstantCommand(() -> driveMult = 1)
                );

        // Return turret
        new GamepadButton(driver1, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> turret.turn(0)));


        // Close shot
        new GamepadButton(driver2, GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> outtake.setVelocityRpm(2920)),
                                new InstantCommand(() -> outtake.setColor(0.3))),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> outtake.setVelocityRpm(0)),
                                new InstantCommand(() -> outtake.setColor(0.0))

                        )

                );

        // Enable/Disable manual mode
        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whileHeld(
                        new InstantCommand(() -> turret.manualTurn(-driver2.getRightX()), turret)
                );

        // Far shot
        new GamepadButton(driver2, GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> outtake.setVelocityRpm(3600)),
                                new InstantCommand(() -> outtake.setColor(0.5))),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> outtake.setVelocityRpm(0)),
                                new InstantCommand(() -> outtake.setColor(0.0))

                        )
                );


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
