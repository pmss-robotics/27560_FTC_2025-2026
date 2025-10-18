package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.util.States.OuttakeExtension;

public class PIDMoveCommand extends CommandBase {
    private OuttakeSlidesSubsystem slides;

    private OuttakeExtension state;

    public PIDMoveCommand(OuttakeSlidesSubsystem slides, OuttakeExtension position){
        this.slides = slides;
        this.state = position;
    }
    @Override
    public void initialize() {
        slides.setState(state);
    }

    @Override
    public boolean isFinished() {
        return slides.pidController.atSetPoint();
    }

}
