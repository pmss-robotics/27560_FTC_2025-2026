package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;


@Autonomous(name = "EmptyAuto", group = "Auto")
@Disabled
public class EmptyAuto extends CommandOpMode {

    PedroDriveSubsystem drive;
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
        StateTransfer.posePedro = drive.follower.getPose();
    }

    private void createPaths() {
        // Find starting position and generate paths
        StateTransfer.alliance = prompter.get("alliance");
        StartingPosition startPosition = prompter.get("startPosition");

        // Init poses
        Pose startPose = new Pose(-40, 54, Math.toRadians(180));
        Pose shootPose = new Pose(-20, 20, Math.toRadians(135));
        Pose gatePose = new Pose(0, 52, Math.toRadians(180));
        Pose row1 = new Pose(-12,26, Math.toRadians(90));
        Pose row2 = new Pose(12, 26, Math.toRadians(90));
        Pose row3 = new Pose(36, 20, Math.toRadians(90));


        if (StateTransfer.alliance == States.Alliance.Blue) { //Map the poses when blue
            //startPose = flipY(startPose);
        }

        drive = new PedroDriveSubsystem(Constants.createFollower(hardwareMap), startPose, telemetry);

        PathChain path1 = drive.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .build();

        Command trajectory = new SequentialCommandGroup(
                new WaitCommand(prompter.get("startDelay")),
                new FollowPathCommand(drive.follower, path1)
        );

        schedule(trajectory);
    }

    enum StartingPosition {
        goalSide,
        farSide
    }
}
