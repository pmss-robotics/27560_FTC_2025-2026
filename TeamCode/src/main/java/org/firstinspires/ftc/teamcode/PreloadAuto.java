package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.commands.MacroCommands;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Autonomous(name = "PreloadAuto", group = "Auto")
public class PreloadAuto extends CommandOpMode {

    DriveSubsystem drive;
    OuttakeSubsystem outtake;
    IntakeSubsystem intake;
    private Prompter prompter;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        prompter = new Prompter(this);

        outtake = new OuttakeSubsystem(hardwareMap, telemetry, true);
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
                .onComplete(this::createPaths);
        prompter.run();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot and prompter
        initialize();

        while (!isStarted()) {
            prompter.run();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // This is the main loop where the command scheduler runs.
            run();

            // Run any other continuous robot logic here.
        }
        reset();
        StateTransfer.pose = drive.getPose();
    }
    private void createPaths() {
        StateTransfer.alliance = prompter.get("alliance");

        Pose2d startPose;
        Pose2d shootingSpot;
        Pose2d parkingSpot;

        if (StateTransfer.alliance.equals(States.Alliance.Red)) {
            startPose = new Pose2d(-40, 54, Math.toRadians(180));
            shootingSpot = new Pose2d(-25, 25, Math.toRadians(135));
            parkingSpot = new Pose2d(-10,29, Math.toRadians(90));

        } else { // Blue
            startPose = new Pose2d(40, 54, Math.toRadians(0));
            shootingSpot = new Pose2d(25,25, Math.toRadians(45));
            parkingSpot = new Pose2d(10,25, Math.toRadians(90));
        }

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        Action toShootingSpot = drive.actionBuilder(drive.getPose())
                .strafeTo(shootingSpot.position)
                //.turnTo(shootingSpot.heading)
                .build();


        Supplier<Action> toPark = () -> drive.actionBuilder(drive.getPose())
                .splineTo(parkingSpot.position, parkingSpot.heading)
                .build();

        //outtake.setDefaultCommand(new RunCommand(outtake::holdSpeed, outtake));
        intake.setDefaultCommand(new RunCommand(intake::holdSpeed, intake));

        SequentialCommandGroup routine = new SequentialCommandGroup(

                // Drive from starting pose to shooting spot
                new ActionCommand(toShootingSpot, Stream.of(drive).collect(Collectors.toSet())),

                // Copy Pasted Jerry Launch macro :)
                MacroCommands.launchSequence(outtake, intake),

                // Go park
                new ActionCommand(toPark.get(), Stream.of(drive).collect(Collectors.toSet()))
        );

        schedule(routine);
    }
}
