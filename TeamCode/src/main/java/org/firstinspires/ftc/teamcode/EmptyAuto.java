package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.InternalPosition.flipY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Autonomous(name = "EmptyAuto", group = "Auto")
@Disabled
public class EmptyAuto extends CommandOpMode {

    DriveSubsystem drive;
    private Prompter prompter;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // Initialize subsystems here

        prompter = new Prompter(this);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
                .prompt("startDelay", new ValuePrompt("Starting Delay (ms)", 0, 20000, 0, 250))
                .prompt("startPosition", new OptionPrompt<>("Starting Position", StartingPosition.goalSide, StartingPosition.farSide))
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
            // prompter.run() should be placed inside this loop.c
            run();

        }
        reset();
        StateTransfer.pose = drive.getPose();
    }

    private void createPaths() {
        // Find starting position and generate paths
        StateTransfer.alliance = prompter.get("alliance");
        StartingPosition startPosition = prompter.get("startPosition");

        // Init poses
        Pose2d startPose;
        Pose2d parkingSpot;

        switch (startPosition){
            case goalSide:
            case farSide:
                startPose = new Pose2d(0,0,0);
                parkingSpot = new Pose2d(0,0,0);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + startPosition); // Android Studio wanted me to
        }

        if (StateTransfer.alliance == States.Alliance.Blue) { //Map the poses when blue
            startPose = flipY(startPose);
            parkingSpot = flipY(parkingSpot);
        }

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .lineToX(parkingSpot.position.x)
                .build();

        Command trajectory = new SequentialCommandGroup(
                new WaitCommand(prompter.get("startDelay")),
                new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()))
        );

        schedule(trajectory);
    }

    enum StartingPosition {
        goalSide,
        farSide
    }
}
