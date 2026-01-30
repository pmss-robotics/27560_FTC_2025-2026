package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.InternalPosition;

import java.util.function.DoubleSupplier;

public class TurretLimelightSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private Limelight3A limelight;
    private TurretSubsystem turret;
    private Follower follower;


    /**
     * Limelight on Turret Subsystem
     * @param hardwareMap the OpMode's hardwareMap
     * @param telemetry the OpMode's telemetry or MultipleTelemetry
     * @param follower a reference to the follower to get pose, velocity and update pose
     * @param turret a reference to the robot's turret and turret angle
     */
    public TurretLimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, TurretSubsystem turret) {
        this.telemetry = telemetry;
        this.follower = follower;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(40);

        limelight.pipelineSwitch(0);

        limelight.start();
    }

    public void periodic() {
        double angle = MathFunctions.normalizeAngle(Math.toDegrees(follower.getHeading()) + turret.getAngle());
        limelight.updateRobotOrientation(angle);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D turretCentre = result.getBotpose_MT2();

            // Create the standard FTC Pose2D
            Pose2D ftcPose = new Pose2D(DistanceUnit.METER, turretCentre.getPosition().x, turretCentre.getPosition().y, AngleUnit.DEGREES, turretCentre.getOrientation().getYaw());

            Pose pedroPose = PoseConverter.pose2DToPose(ftcPose, InvertedFTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);


            Vector offset = new Vector(InternalPosition.turretOffset, Math.toRadians(angle));

            pedroPose = pedroPose.plus(new Pose(offset.getXComponent(), offset.getYComponent()));

            telemetry.addData("Follower Pose", follower.getPose().toString());
            telemetry.addData("Limelight Pose", pedroPose.toString());

            // TODO: Logic here for when stationary, update pose

            /*
            if (follower.getVelocity().getMagnitude() < 0.5){
                follower.setPose(pedroPose);
            }
             */
        }
    }
}
