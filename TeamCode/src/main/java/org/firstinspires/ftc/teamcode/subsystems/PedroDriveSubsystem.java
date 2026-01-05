package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PedroDriveSubsystem extends SubsystemBase {
    public Follower follower;
    private Telemetry telemetry;
    public PedroDriveSubsystem(Follower follower, Pose startPose, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        follower.setStartingPose(startPose);
    }

    @Override
    public void periodic() {
        follower.update();

        drawDebug(follower);

        Pose pose = getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y",pose.getY());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
    }

    public Pose getPose() {
        return follower.getPose();
    }
}
