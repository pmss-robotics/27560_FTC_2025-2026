package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Config
public class InternalPosition {
    private Supplier<Pose> robot;
    private DoubleSupplier turret;

    public static double turretOffset = 56.93625/25.4, cameraOffset = 180.97618/25.4;
    public static Pose blueGoal = new Pose(12,134), redGoal = mirrorIf(blueGoal, true);
    public InternalPosition(Supplier<Pose> robotPosition, DoubleSupplier turretRotation) {
        robot = robotPosition;
        turret = turretRotation;
    }

    public double getDistance() {
        return robot.get().distanceFrom(goal());
    }

    public double getTurretAngle() {
        Pose r = robot.get();
        Pose g = goal();
        return Math.atan2(g.getY() - r.getY(), g.getX() - r.getX()) - r.getHeading();
    }

    public static double getTurretAngle(Pose robot) {
        Pose g = goal();
        return Math.toDegrees(Math.atan2(g.getY() - robot.getY(), g.getX() - robot.getX()) - robot.getHeading());
    }

    public static Pose goal() {
        switch (StateTransfer.alliance) {
            case Red: return redGoal;
            case Blue: return blueGoal;
        }
        return new Pose(Double.NaN,Double.NaN,Double.NaN);
    }

    public static Pose mirrorIf(Pose oldPose, boolean flip) {
        if (flip) {
            return oldPose.mirror().withX(141.5 - oldPose.getX());
        } else {
            return oldPose;
        }
    }

    public static Pose mirrorIf(double x, double y, double degrees, boolean flip) {
        return mirrorIf(new Pose(x, y, Math.toRadians(degrees)), flip);
    }

    public static double mirrorAngleIf(double theta, boolean flip) {
        if (flip) {
            return MathFunctions.normalizeAngle(Math.PI - theta);
        } else {
            return theta;
        }
    }


    // RR Methods
    //

    public static Pose2d flipY(Pose2d oldPose) {
        return new Pose2d(new Vector2d(-oldPose.position.x, oldPose.position.y), new Rotation2d(-oldPose.heading.real, oldPose.heading.imag));
    }

    public static Rotation2d flipY(Rotation2d oldRotation) {
        return new Rotation2d(-oldRotation.real, oldRotation.imag);
    }

    public static Pose2d flipYIf(Pose2d oldPose, boolean flip) {
        if (flip) {
            return flipY(oldPose);
        } else {
            return oldPose;
        }
    }

    public static Rotation2d flipYIf(Rotation2d oldRotation, boolean flip) {
        if (flip) {
            return flipY(oldRotation);
        } else {
            return oldRotation;
        }
    }
}
