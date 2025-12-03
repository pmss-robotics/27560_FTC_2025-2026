package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class InternalPosition {
    private Supplier<Pose2d> robot;
    private DoubleSupplier turret;

    public static double turretOffset = 56.93625/25.4, cameraOffset = 180.97618/25.4;
    public static Vector2d redGoal = new Vector2d(-60,60), blueGoal = new Vector2d(60, 60);

    public InternalPosition(Supplier<Pose2d> robotPosition, DoubleSupplier turretRotation) {
        robot = robotPosition;
        turret = turretRotation;
    }


    /**
     * Camera's 2d Position in field coordinates from DriveLocalizer and
     **/
    public Pose2d turretCameraPosition() {
        Pose2d robotPose = robot.get();
        double robotHeading = robotPose.heading.toDouble();
        double turretHeading = turret.getAsDouble();

        return new Pose2d(
                robotPose.position
                        .plus(fromPolar(turretOffset, robotHeading))
                        .plus(fromPolar(cameraOffset, robotHeading + turretHeading)),
                robotHeading + turret.getAsDouble()
        );
    }

    public static Vector2d fromPolar(double distance, double angle) {
        return new Vector2d(distance * Math.cos(angle), distance * Math.sin(angle));
    }

    public static double getAngle(Pose2d robot, Vector2d object) {
        return Math.atan2(object.y - robot.position.y, object.x - robot.position.x);
    }


    public static Pose2d flipY(Pose2d oldPose) {
        return new Pose2d(new Vector2d(-oldPose.position.x, oldPose.position.y), new Rotation2d(-oldPose.heading.real, oldPose.heading.imag));
    }

    public static Rotation2d flipY(double angle) {
        Rotation2d rot = Rotation2d.fromDouble(angle);
        return new Rotation2d(-rot.real, rot.imag);
    }
}
