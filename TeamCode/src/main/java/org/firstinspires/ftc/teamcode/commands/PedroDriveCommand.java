package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PedroTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

import java.util.function.DoubleSupplier;

public class PedroDriveCommand extends CommandBase {
    private final PedroDriveSubsystem drive;
    private final DoubleSupplier lx, ly, rx;
    public static boolean enabled;
    private final boolean isFieldCentric;
    private double target;
    private PIDFController headingPIDF;

    public PedroDriveCommand(PedroDriveSubsystem drive, DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, boolean isFieldCentric) {
        this.drive = drive;
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.isFieldCentric = isFieldCentric;
        enabled = true;
    }

    @Override
    public void initialize() {
        target = drive.follower.getPose().getHeading();
        drive.follower.startTeleopDrive();
        headingPIDF = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
    }
    @Override
    public void execute() {
        headingPIDF.setCoefficients(Constants.followerConstants.getCoefficientsHeadingPIDF()); //FIXME: remove me when done tuning
        Pose currentPose = drive.follower.getPose();
        if(rx.getAsDouble()!= 0) {
            target = drive.follower.getPose().getHeading();
            drive.follower.setTeleOpDrive(ly.getAsDouble(), lx.getAsDouble(), rx.getAsDouble(), !isFieldCentric);
        } else {
            // possibly deadband?
            double headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), target) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), target);
            headingPIDF.updateError(headingError);
            double heading = MathFunctions.clamp(headingPIDF.run() + Constants.followerConstants.getCoefficientsHeadingPIDF().F * MathFunctions.getTurnDirection(currentPose.getHeading(), target), -1, 1);
            drive.follower.setTeleOpDrive(ly.getAsDouble(), lx.getAsDouble(), heading, !isFieldCentric);
        }

    }
}
