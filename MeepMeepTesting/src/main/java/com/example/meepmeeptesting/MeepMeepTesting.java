package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

        Pose2d startPose = new Pose2d(-40, 54, Math.toRadians(180));
        Pose2d shootPose = new Pose2d(-20, 20, Math.toRadians(135));
        Pose2d gatePose = new Pose2d(0, 52, Math.toRadians(180));
        Pose2d row1 = new Pose2d(-12,32, Math.toRadians(90));
        Pose2d row2 = new Pose2d(12, 32, Math.toRadians(90));
        Pose2d row3 = new Pose2d(36, 32, Math.toRadians(90));

        Pose2d startPoseB = new Pose2d(-40, -54, Math.toRadians(180));
        Pose2d shootPoseB = new Pose2d(-20, -20, Math.toRadians(225));
        Pose2d gatePoseB = new Pose2d(0, -52, Math.toRadians(180));
        Pose2d row1B = new Pose2d(-12,-26, Math.toRadians(270));
        Pose2d row2B = new Pose2d(12, -26, Math.toRadians(270));
        Pose2d row3B = new Pose2d(36, -26, Math.toRadians(270));

        myRedBot.runAction(myRedBot.getDrive().actionBuilder(startPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(row1, row1.heading)
                .splineToLinearHeading(new Pose2d(row1.position.x, 46, row1.heading.log()), row1.heading) // Intake

                .splineToSplineHeading(gatePose, gatePose.heading)
                .setTangent(gatePose.heading)
                .strafeTo(new Vector2d(0, 56)) // Push Gate

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(-5))
                .splineToSplineHeading(row2, row2.heading)
                .splineToLinearHeading(new Pose2d(row2.position.x, 46, row2.heading.log()), row2.heading) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(-15))
                .splineToSplineHeading(row3, row3.heading)
                .splineToLinearHeading(new Pose2d(row3.position.x, 46, row3.heading.log()), row3.heading) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(gatePose.position, gatePose.heading)

                .build());

        myBlueBot.runAction(myBlueBot.getDrive().actionBuilder(startPoseB)
                .strafeToLinearHeading(shootPoseB.position, shootPoseB.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(row1B, row1B.heading)
                .splineToLinearHeading(new Pose2d(row1B.position.x, -46, row1B.heading.log()), row1B.heading) // Intake

                .splineToSplineHeading(gatePoseB, gatePoseB.heading)
                .setTangent(gatePoseB.heading)
                .strafeTo(new Vector2d(0, -56)) // Push Gate

                .strafeToLinearHeading(shootPoseB.position, shootPoseB.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(25))
                .splineToSplineHeading(row2B, row2B.heading)
                .splineToLinearHeading(new Pose2d(row2B.position.x, -46, row2B.heading.log()), row2B.heading) // Intake

                .strafeToLinearHeading(shootPoseB.position, shootPoseB.heading)
                .waitSeconds(1) // Launch balls

                .setTangent(Math.toRadians(5))
                .splineToSplineHeading(row1B, row1B.heading)
                .splineToLinearHeading(new Pose2d(row3B.position.x, -46, row3B.heading.log()), row3B.heading) // Intake

                .strafeToLinearHeading(shootPoseB.position, shootPoseB.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(gatePoseB.position, gatePoseB.heading)
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

        testBot.runAction(myBlueBot.getDrive().actionBuilder(startPoseB)
                //.splineToLinearHeading(shootPose, -shootPose.heading.log())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myRedBot)
                //.addEntity(testBot)
                .addEntity(myBlueBot)
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