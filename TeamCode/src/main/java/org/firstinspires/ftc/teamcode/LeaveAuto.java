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
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.stream.Collectors;
import java.util.stream.Stream;

@Autonomous(name = "LeaveAuto", group = "Auto")
public class LeaveAuto extends CommandOpMode {

    DriveSubsystem drive;
    private Prompter prompter;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        prompter = new Prompter(this);

        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", States.Alliance.Red, States.Alliance.Blue))
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
        StateTransfer.alliance = prompter.get("alliance");
        StartingPosition startingPosition = prompter.get("startPosition");
        Pose2d startPose;
        Pose2d parkingSpot;

        switch (startingPosition){
            case goalSide:
                startPose = new Pose2d(-40, 54, 0);
                parkingSpot = new Pose2d(-28, 54, 0);
                break;
            case farSide:
                startPose = new Pose2d(64,10, Math.toRadians(180));
                parkingSpot = new Pose2d(54, 10, Math.toRadians(180));
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + startingPosition); // Android Studio wanted me to
        }

        // Manually map the poses because I can't understand posemaps
        if (StateTransfer.alliance == States.Alliance.Blue) {
            startPose = flipY(startPose);
            parkingSpot = flipY(parkingSpot);
        }

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .splineTo(parkingSpot.position, parkingSpot.heading)
                .build();

        Command trajectory = new SequentialCommandGroup(
                new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()))
        );

        schedule(trajectory);
    }

    enum StartingPosition {
        goalSide,
        farSide
    }
}
