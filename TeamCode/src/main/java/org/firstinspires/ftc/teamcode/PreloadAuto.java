package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.stream.Collectors;
import java.util.stream.Stream;

@Autonomous(name = "PreloadAuto", group = "Auto")
public class PreloadAuto extends CommandOpMode {

    DriveSubsystem drive;
    OuttakeSubsystem flywheel;
    private Prompter prompter = new Prompter(this);

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
                .onComplete(this::onPromptsComplete);
    }

    private void onPromptsComplete() {
        StateTransfer.alliance = prompter.get("alliance");

        Pose2d startPose;
        Pose2d shootingSpot;
        Pose2d parkingSpot;

        if (StateTransfer.alliance.equals(States.Alliance.Red)) {
            startPose = new Pose2d(-40, 54, Math.toRadians(180));
            shootingSpot = new Pose2d(-29, 29, Math.toRadians(135));
            parkingSpot = new Pose2d(-12,29, Math.toRadians(90));

        } else { // Blue
            startPose = new Pose2d(-40, -54, Math.toRadians(180));
            shootingSpot = new Pose2d(-29,-29, Math.toRadians(215));
            parkingSpot = new Pose2d(-12,-29, Math.toRadians(270));
        }

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);
        flywheel = new OuttakeSubsystem(hardwareMap, telemetry, true);

        Action toShootingSpot = drive.actionBuilder(drive.getPose())
                .strafeTo(shootingSpot.position)
                .turnTo(shootingSpot.heading)
                .build();

        Action toPark = drive.actionBuilder(drive.getPose())
                .turnTo(parkingSpot.heading)
                .strafeTo(parkingSpot.position)
                .build();

        // Make this to a sequential command
        Command trajectory = new ActionCommand(toShootingSpot, Stream.of(drive).collect(Collectors.toSet()));

        flywheel.setDefaultCommand(new RunCommand(flywheel::holdSpeed));

        SequentialCommandGroup routine = new SequentialCommandGroup(
                new ActionCommand(toShootingSpot, Stream.of(drive).collect(Collectors.toSet())),
                new InstantCommand(() -> flywheel.setPower(12), flywheel),
                new WaitCommand(1000)


        );
        schedule(trajectory);
    }
}
