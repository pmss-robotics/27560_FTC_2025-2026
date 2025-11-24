package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class InternalPosition {
    private Supplier<Pose2d> robot;
    private DoubleSupplier turret;

    public static double turretOffset = 56.93625/25.4, cameraOffset = 180.97618/25.4;

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

    public Vector2d fromPolar(double distance, double angle) {
        return new Vector2d(distance * Math.sin(angle), distance * Math.cos(angle));
    }
}
