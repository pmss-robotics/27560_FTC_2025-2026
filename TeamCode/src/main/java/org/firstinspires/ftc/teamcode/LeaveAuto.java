package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.InternalPosition.mirrorIf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
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

@Autonomous(name = "LeaveAuto", group = "Auto")
public class LeaveAuto extends CommandOpMode {

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
                //.prompt("startDelay", new ValuePrompt("Starting Delay (ms)", 0, 20000, 0, 250))
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
        // All poses are for blue side
        boolean flip = StateTransfer.alliance == States.Alliance.Red;

        Pose startPose;
        BezierCurve toPark;

        switch (startPosition) {
            case farSide:
                startPose = mirrorIf(57.0, 9.0, 90, flip);
                toPark = new BezierLine(
                        startPose,
                        mirrorIf(57.0, 36.0, 180, flip)
                );
                break;
            case goalSide:
                startPose = mirrorIf(21, 118, 180, flip);
                toPark = new BezierCurve(
                        startPose,
                        mirrorIf(54.0, 104.0, 180, flip),
                        mirrorIf(40.0, 84.0, 180, flip)
                );
                break;
            default:
                throw new IllegalStateException("Why no set start pose?");
        }


        if (StateTransfer.alliance == States.Alliance.Blue) { //Map the poses when blue
            //startPose = flipY(startPose);
        }

        drive = new PedroDriveSubsystem(Constants.createFollower(hardwareMap), startPose, telemetry);

        PathChain path1 = drive.follower.pathBuilder()
                .addPath(toPark)
                .build();

        Command trajectory = new SequentialCommandGroup(
                //new WaitCommand(prompter.get("startDelay").longValue()),
                new FollowPathCommand(drive.follower, path1)
        );

        schedule(trajectory);
    }

    enum StartingPosition {
        goalSide,
        farSide
    }
}
