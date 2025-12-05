package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.InternalPosition.flipY;
import static com.example.meepmeeptesting.InternalPosition.flipYIf;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Image FIELD_DECODE_BLUE = null;
        try { FIELD_DECODE_BLUE = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/field-2025-juice-dark.png")); }
        catch(IOException e) {}

        MeepMeep meepMeep = new MeepMeep(600);

        // Red Auto

        RoadRunnerBotEntity myRedBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.25, 16.378)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        RoadRunnerBotEntity myBlueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.25, 16.378)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        boolean flip = false;

        Pose2d startPose = flipYIf(new Pose2d(-40, 54, Math.toRadians(180)), flip);
        Pose2d shootPose = flipYIf(new Pose2d(-10, 10, Math.toRadians(135)), flip);
        Pose2d farPose = flipYIf(new Pose2d(56,9,Math.toRadians(180)), flip);
        Pose2d gatePose = flipYIf(new Pose2d(0, 52, Math.toRadians(180)), flip);
        Rotation2d row1Tangent = flipYIf(Rotation2d.exp(Math.toRadians(100)), flip);
        Pose2d row1 = flipYIf(new Pose2d(-12,32, Math.toRadians(90)), flip);
        Rotation2d row2Tangent = flipYIf(Rotation2d.exp(Math.toRadians(0)), flip);
        Pose2d row2 = flipYIf(new Pose2d(12, 32, Math.toRadians(90)), flip);
        Rotation2d row3Tangent = flipYIf(Rotation2d.exp(Math.toRadians(0)), flip);
        Pose2d row3 = flipYIf(new Pose2d(36, 32, Math.toRadians(90)), flip);
        Pose2d farPark = flipYIf(new Pose2d(40, 9, Math.toRadians(180)), flip);


        Pose2d startPoseB = flipY(startPose);
        Pose2d shootPoseB = flipY(shootPose);
        Pose2d gatePoseB = flipY(gatePose);
        Pose2d row1B = flipY(row1);
        Pose2d row2B = flipY(row2);
        Pose2d row3B = flipY(row3);

        myRedBot.runAction(myRedBot.getDrive().actionBuilder(startPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(row1Tangent)
                .splineToSplineHeading(row1, row1.heading)
                .splineToLinearHeading(new Pose2d(row1.position.x, 48, row1.heading.log()), row1.heading) // Intake

                .splineToSplineHeading(gatePose, gatePose.heading)
                .setTangent(gatePose.heading)
                .strafeTo(new Vector2d(0, 56)) // Push Gate

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(row2Tangent)
                .splineToSplineHeading(row2, row2.heading)
                .splineToLinearHeading(new Pose2d(row2.position.x, 48, row2.heading.log()), row2.heading) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(row3Tangent)
                .splineToSplineHeading(row3, row3.heading)
                .splineToLinearHeading(new Pose2d(row3.position.x, 48, row3.heading.log()), row3.heading) // Intake

                .strafeToLinearHeading(farPose.position, farPose.heading)
                .waitSeconds(1) // Launch balls

                //.strafeToLinearHeading(gatePose.position, gatePose.heading)
                        .splineToLinearHeading(farPark, farPark.heading)

                .build());

        myBlueBot.runAction(myBlueBot.getDrive().actionBuilder(startPoseB)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(row1Tangent)
                .splineToSplineHeading(row1, row1.heading)
                .splineToLinearHeading(new Pose2d(row1.position.x, 48, row1.heading.log()), row1.heading) // Intake

                .splineToSplineHeading(gatePose, gatePose.heading)
                .setTangent(gatePose.heading)
                .strafeTo(new Vector2d(0, 56)) // Push Gate

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(row2Tangent)
                .splineToSplineHeading(row2, row2.heading)
                .splineToLinearHeading(new Pose2d(row2.position.x, 48, row2.heading.log()), row2.heading) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(row3Tangent)
                .splineToSplineHeading(row3, row3.heading)
                .splineToLinearHeading(new Pose2d(row3.position.x, 48, row3.heading.log()), row3.heading) // Intake

                .strafeToLinearHeading(farPose.position, farPose.heading)
                .waitSeconds(1) // Launch balls

                //.strafeToLinearHeading(gatePose.position, gatePose.heading)
                .splineToLinearHeading(farPark, farPark.heading)
                .build());

        // Driving Testbot
        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.25, 16.378)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .build();

        testBot.runAction(myRedBot.getDrive().actionBuilder(startPose)
                //.splineToLinearHeading(shootPose, -shootPose.heading.log())
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myRedBot)
                //.addEntity(testBot)
                //.addEntity(myBlueBot)
                .start();
    }

    /*Action LeaveAuto(RoadRunnerBotEntity bot, boolean red) {
        Pose2d startPose, parkPose;

        startPose = new Pose2d(0,0,0);
        parkPose = new Pose2d(0,0,0);

        TrajectoryActionBuilder
        startPose = red ? startPose : flipY(startPose);
        return bot.getDrive().actionBuilder(startPose)
                .build();
    }*/


    private static double toObject(Pose2d robot, Vector2d object) {
        return Math.atan2(object.y - robot.position.y, object.x - robot.position.x);
    }
}