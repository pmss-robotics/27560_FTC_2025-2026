package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

@Config
public class MacroCommands {
    public static long initialFlywheelSpinUp = 5000, kickDelay = 500, intakeFeedTime = 565, spinUp = 250;
    public static Command launchSequence(OuttakeSubsystem outtake, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
                //new InstantCommand(() -> outtake.setPower(OuttakeSubsystem.flywheelVelocity), outtake),
                //new WaitCommand(initialFlywheelSpinUp),

                new InstantCommand(() -> intake.setPower(12), intake),
                new WaitCommand(intakeFeedTime),
                new InstantCommand(() -> intake.setPower(0), intake),
                new WaitCommand(spinUp),

                new InstantCommand(() -> intake.setPower(12), intake),
                new WaitCommand(intakeFeedTime),
                new InstantCommand(() -> intake.setPower(0), intake),
                new WaitCommand(spinUp),

                MacroCommands.kickSequence(outtake)

                //new InstantCommand(() -> outtake.setPower(0), outtake)
        );
    }
    public static Command kickSequence(OuttakeSubsystem outtake) {
        return new SequentialCommandGroup(
                new InstantCommand(outtake::kick),
                new WaitCommand(kickDelay),
                new InstantCommand(outtake::home)
        );
    }
}
