package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.InternalPosition.flipYIf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;
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



        boolean flip = StateTransfer.alliance.equals(States.Alliance.Blue);

        Pose2d startPose = flipYIf(new Pose2d(-40, 54, Math.toRadians(180)), flip);
        Pose2d shootingSpot = flipYIf(new Pose2d(-25, 25, Math.toRadians(135)), flip);
        Pose2d parkingSpot = flipYIf(new Pose2d(-10,29, Math.toRadians(90)), flip);


        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        Action toShootingSpot = drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(shootingSpot.position, shootingSpot.heading)
                .build();


        Supplier<Action> toPark = () -> drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(parkingSpot.position, parkingSpot.heading)
                .build();

        intake.setDefaultCommand(new RunCommand(intake::holdSpeed, intake));

        SequentialCommandGroup routine = new SequentialCommandGroup(
                new InstantCommand(() -> outtake.setVelocityRpm(OuttakeSubsystem.closeShot)),

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
