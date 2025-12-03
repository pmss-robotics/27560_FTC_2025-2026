package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.InternalPosition.flipY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
        Pose2d startPose = new Pose2d(-40, 54, Math.toRadians(180));
        Pose2d shootPose = new Pose2d(-20, 20, Math.toRadians(135));
        Pose2d gatePose = new Pose2d(0, 52, Math.toRadians(180));
        Pose2d row1 = new Pose2d(-12,26, Math.toRadians(90));
        Pose2d row2 = new Pose2d(12, 26, Math.toRadians(90));
        Pose2d row3 = new Pose2d(36, 20, Math.toRadians(90));


        if (StateTransfer.alliance == States.Alliance.Blue) { //Map the poses when blue
            //startPose = flipY(startPose);
        }

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(row1, row1.heading)
                .splineToLinearHeading(new Pose2d(row1.position.x, 46, row1.heading.log()), row1.heading) // Intake

                .splineToSplineHeading(gatePose, gatePose.heading)
                .setTangent(gatePose.heading)
                .strafeTo(new Vector2d(0, 56)) // Push Gate

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(-25))
                .splineToSplineHeading(row2, row2.heading)
                .splineToLinearHeading(new Pose2d(row2.position.x, 46, row2.heading.log()), row2.heading) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(-5))
                .splineToSplineHeading(row1, row1.heading)
                .splineToLinearHeading(new Pose2d(row3.position.x, 46, row3.heading.log()), row3.heading) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(gatePose.position, gatePose.heading)

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
