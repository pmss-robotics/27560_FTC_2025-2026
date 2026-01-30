package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.InternalPosition.mirrorIf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.MacroCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.InternalPosition;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;


@Autonomous(name = "12 Ball Auto", group = "Auto")
public class TwelveBallAuto extends CommandOpMode {

    PedroDriveSubsystem drive;
    OuttakeSubsystem outtake;
    TurretSubsystem turret;
    IntakeSubsystem intake;
    LoopTimer timer;
    private Prompter prompter;

    @Override
    public void initialize() {
        super.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // Initialize subsystems here
        if (Settings.get("loop_detect_mode", false)) {
            timer = new LoopTimer(telemetry, "Main");
        }
        outtake = new OuttakeSubsystem(hardwareMap, telemetry, true);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap, telemetry);

        outtake.setDefaultCommand(new RunCommand(outtake::holdSpeed, outtake));
        intake.setDefaultCommand(new RunCommand(intake::holdSpeed, intake));

        prompter = new Prompter(this);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
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
        StateTransfer.posePedro = drive.follower.getPose();
    }

    private void createPaths() {
        // Find starting position and generate paths
        StateTransfer.alliance = prompter.get("alliance");
        StartingPosition startPosition = prompter.get("startPosition");

        boolean flip = StateTransfer.alliance == States.Alliance.Red;

        // Init poses
        Pose startPose, shootPose, parkPose;
        switch (startPosition) {
            case farSide:
                startPose = mirrorIf(57, 9, 90, flip);
                shootPose = mirrorIf(57, 9, 90, flip);
                parkPose = mirrorIf(57, 36, 36, flip);
                break;
            case goalSide:
                startPose = mirrorIf(23, 120, 180, flip);
                shootPose = mirrorIf(48, 96, 180, flip);
                parkPose = mirrorIf(48, 120, 90, flip);
                break;
            default: throw new RuntimeException("no poses");
        }


        drive = new PedroDriveSubsystem(Constants.createFollower(hardwareMap), startPose, telemetry);

        PathChain path1 = drive.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        PathChain path2 = drive.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();


        Command sequence = new SequentialCommandGroup(

                new InstantCommand(() -> outtake.setVelocityRpm(2900), outtake),
                new InstantCommand(() -> turret.turn(InternalPosition.getTurretAngle(shootPose))),

                // Drive to the shooting position

                new ConditionalCommand(
                        new FollowPathCommand(drive.follower, path1), //Drive
                        new WaitCommand(1000),
                        () -> startPosition == StartingPosition.goalSide
                ),

                MacroCommands.launchSequence(outtake, intake), // Launch

                new FollowPathCommand(drive.follower, path2, false), // Drive to park, disable flywheel

                new InstantCommand(() -> outtake.setVelocityRpm(0), outtake)
        );

        schedule(sequence);
    }

    enum StartingPosition {
        goalSide,
        farSide
    }
}
