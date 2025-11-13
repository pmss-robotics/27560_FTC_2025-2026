package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.stream.Collectors;
import java.util.stream.Stream;

@Autonomous(name = "LeaveAuto", group = "Auto")
public class LeaveAuto extends CommandOpMode {

    DriveSubsystem drive;
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

        if (StateTransfer.alliance.equals(States.Alliance.Red)) {
            startPose = new Pose2d(-40, 54, 0);
        } else { // Blue
            startPose = new Pose2d(-40, -54, 0);
        }

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, startPose), telemetry);

        Action trajectoryAction = drive.actionBuilder(drive.getPose())
                .lineToX(-28)
                .build();

        Command trajectory = new ActionCommand(trajectoryAction, Stream.of(drive).collect(Collectors.toSet()));

        schedule(trajectory);
    }
}
