package org.firstinspires.ftc.teamcode.rrAuto;

import static org.firstinspires.ftc.teamcode.util.InternalPosition.flipYIf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.MacroCommands;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Autonomous(name = "TwelveBallAutoBlueRR", group = "RR")
public class TwelveBallAutoBlueRR extends CommandOpMode {

    DriveSubsystem drive;
    OuttakeSubsystem outtake;
    IntakeSubsystem intake;
    TurretSubsystem turret;
    LoopTimer timer;
    //TurretVisionSubsystem turretVision;

    private Prompter prompter;



    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // Initialize subsystems here
        if (Settings.get("loop_detect_mode", false)) {
            timer = new LoopTimer(telemetry, "Main");
        }

        prompter = new Prompter(this);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
                .prompt("startDelay", new ValuePrompt("Starting Delay (ms)", 0, 20000, 0, 250))
                .prompt("startPosition", new OptionPrompt<>("Starting Position", StartingPosition.goalSide, StartingPosition.farSide))
                .onComplete(this::createPaths);


        prompter.run();
    }

    @Override
    public void initialize_loop() {
        prompter.run();
    }

    @Override
    public void end() {
        StateTransfer.pose = drive.getPose();
    }

    private void createPaths() {
        // Find starting position and generate paths
        StateTransfer.alliance = prompter.get("alliance");
        StartingPosition startPosition = prompter.get("startPosition");

        boolean flip = StateTransfer.alliance == States.Alliance.Blue;

        // Init poses
        Pose2d startPose = flipYIf(new Pose2d(-40, 54, Math.toRadians(180)), flip);
        Pose2d shootPose = flipYIf(new Pose2d(-24, 24, Math.toRadians(135)), flip);
        Pose2d shootPosetspmo = flipYIf(new Pose2d(-21, 21, Math.toRadians(132)), flip);
        Pose2d farPose = flipYIf(new Pose2d(56,9,Math.toRadians(180)), flip);
        Pose2d gatePose = flipYIf(new Pose2d(-4, 56, Math.toRadians(180)), flip);
        Rotation2d row1Tangent = flipYIf(Rotation2d.exp(Math.toRadians(100)), flip);
        Pose2d row1 = flipYIf(new Pose2d(-14,45, Math.toRadians(90)), flip);
        Rotation2d row2Tangent = flipYIf(Rotation2d.exp(Math.toRadians(0)), flip);
        Pose2d row2 = flipYIf(new Pose2d(14, 45, Math.toRadians(90)), flip);
        Rotation2d row3Tangent = flipYIf(Rotation2d.exp(Math.toRadians(0)), flip);
        Pose2d row3 = flipYIf(new Pose2d(40, 32, Math.toRadians(90)), flip);
        Pose2d farPark = flipYIf(new Pose2d(40, 9, Math.toRadians(180)), flip);

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);
        /*try {
            //turretVision = new TurretVisionSubsystem(hardwareMap, telemetry, false);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }*/

        //turretVision.enableDetection(true);

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        outtake = new OuttakeSubsystem(hardwareMap, telemetry, true);

        turret = new TurretSubsystem(hardwareMap, telemetry);

        intake.setDefaultCommand(new RunCommand(intake::holdSpeed, intake));

        // To shoot preloads
        Action path1 = drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .build();

        // To intake row1

        Supplier<Action> path2 = () -> drive.actionBuilder(drive.getPose())
                .setTangent(row1Tangent)
                .splineToSplineHeading(row1, row1.heading)
                .splineToLinearHeading(new Pose2d(row1.position.x, 72, row1.heading.log()), row1.heading) // Intake
                .build();

        // To gate and shoot
        Supplier<Action> path3 = () -> drive.actionBuilder(drive.getPose())
                //.splineToSplineHeading(gatePose, gatePose.heading)
                //.setTangent(gatePose.heading)
                //.strafeTo(new Vector2d(0, 56))
                .strafeToLinearHeading(shootPosetspmo.position, shootPosetspmo.heading)

                .build();

        // To intake row2
        Supplier<Action> path4 = () -> drive.actionBuilder(drive.getPose())
                .setTangent(row2Tangent)
                .splineToSplineHeading(row2, row2.heading)
                .splineToLinearHeading(new Pose2d(row2.position.x, 61, row2.heading.log()), row2.heading) // Intake
                .build();

        // To shoot
        Supplier<Action> path5 = () -> drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .build();

        // To intake row3
        Supplier<Action> path6 = () -> drive.actionBuilder(drive.getPose())
                .setTangent(row3Tangent)
                .splineToSplineHeading(row3, row3.heading)
                .splineToLinearHeading(new Pose2d(row3.position.x, 50, row3.heading.log()), row3.heading) // Intake
                .build();

        // To shoot
        Supplier<Action> path7 = () -> drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(farPose.position, farPose.heading)
                .build();

        // To park
        Supplier<Action> path8 = () -> drive.actionBuilder(drive.getPose())
                .splineToLinearHeading(farPark, farPark.heading)
                .build();


        Command trajectory = new SequentialCommandGroup(
                new WaitCommand(0),

                // Go shoot
                new InstantCommand(() -> outtake.setVelocityRpm(2900)),
                new ActionCommand(path1, Stream.of(drive).collect(Collectors.toSet())),

                // Launch
                MacroCommands.launchSequence(outtake, intake),
                new InstantCommand(() -> outtake.setVelocityRpm(0)),

                // Drive and intake
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(12)),
                        new ActionCommand(path2.get(), Stream.of(drive).collect(Collectors.toSet()))
                ),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.setPower(-12)),
                                new WaitCommand(100),
                                new InstantCommand(() -> intake.setPower(0)),
                                new InstantCommand(() -> outtake.setVelocityRpm(OuttakeSubsystem.closeShot))
                        ),
                        // Open the gate and return to shoot
                        new ActionCommand(path3.get(), Stream.of(drive).collect(Collectors.toSet()))
                        ),

                MacroCommands.launchSequence(outtake, intake),
                new InstantCommand(() -> outtake.setVelocityRpm(0)),

                // Drive and intake row2
                new ParallelCommandGroup(
                    new InstantCommand(() -> intake.setPower(12)),
                    new ActionCommand(path4.get(), Stream.of(drive).collect(Collectors.toSet()))
                ),

                // Drive to shoot while unintaking and turning flywheel on
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.setPower(-12)),
                                new WaitCommand(100),
                                new InstantCommand(() -> intake.setPower(0)),
                                new InstantCommand(() -> outtake.setVelocityRpm(OuttakeSubsystem.closeShot))
                        ),
                        // Return to shoot
                        new ActionCommand(path5.get(), Stream.of(drive).collect(Collectors.toSet()))
                ),


                MacroCommands.launchSequence(outtake, intake),
                new InstantCommand(() -> outtake.setVelocityRpm(0)),


                // Drive and intake row3
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(12)),
                        new ActionCommand(path6.get(), Stream.of(drive).collect(Collectors.toSet()))
                ),

                // Drive to shoot far
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.setPower(-12)),
                                new WaitCommand(100),
                                new InstantCommand(() -> intake.setPower(0)),
                                new InstantCommand(() -> outtake.setVelocityRpm(OuttakeSubsystem.farShot))
                        ),
                        // Return to shoot
                        new ActionCommand(path7.get(), Stream.of(drive).collect(Collectors.toSet()))
                ),


                MacroCommands.launchSequence(outtake, intake),
                new InstantCommand(() -> outtake.setVelocityRpm(0)),

                new ActionCommand(path8.get(), Stream.of(drive).collect(Collectors.toSet()))
        );



        schedule(trajectory);
    }

    enum StartingPosition {
        goalSide,
        farSide
    }
}
