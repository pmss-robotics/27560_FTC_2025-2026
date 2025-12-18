package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
public class DriveSubsystem extends SubsystemBase {

    public final MecanumDrive drive;
    private final Telemetry telemetry;

    public DriveSubsystem(MecanumDrive drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
    }

    /*
    field centric means that 'forward' or any other direction is always relative to the
    driver / field i.e. it stays constant regardless of the robot's current heading
     */
    public void fieldCentric(double lx, double ly, double rx) {
        // TODO verify if this works the inverse() might not be necessary
        setDrivePowers(new PoseVelocity2d(
                drive.localizer.getPose().heading.inverse().times(new Vector2d(lx, ly)),
                rx
        ));
        // heading as a complex number
        // heading.real * ly - heading.imag * lx,
        // heading.imag * ly + heading.real * lx
        updatePoseEstimate();
    }

    /*
    robot centric means that direction is always relative to the robot
     */
    public void robotCentric(double lx, double ly, double rx) {
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        ly,
                        lx
                ),
                rx
        ));
        updatePoseEstimate();
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        drive.setDrivePowers(powers);
    }

    public PoseVelocity2d updatePoseEstimate() {
        return drive.updatePoseEstimate();
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return drive.actionBuilder(beginPose);
    }

    public Pose2d getPose() {
        return drive.localizer.getPose();
    }
}
