package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.InternalPosition.flipY;
import static com.example.meepmeeptesting.InternalPosition.getAngle;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.function.Function;

import javax.imageio.ImageIO;

import kotlin.jvm.functions.Function1;

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

        Pose2d startPose = new Pose2d(-40, 54, Math.toRadians(180));
        Pose2d shootPose = new Pose2d(-20, 20, Math.toRadians(135));
        Pose2d gatePose = new Pose2d(0, 52, Math.toRadians(180));
        Pose2d row1 = new Pose2d(-12,26, Math.toRadians(90));
        Pose2d row2 = new Pose2d(12, 26, Math.toRadians(90));
        Pose2d row3 = new Pose2d(36, 26, Math.toRadians(90));

        myRedBot.runAction(myRedBot.getDrive().actionBuilder(startPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(row1.position, row1.heading)
                .setTangent(row1.heading)
                .lineToY(48) // Intake

                .strafeToLinearHeading(gatePose.position, gatePose.heading)
                .setTangent(gatePose.heading)
                .strafeTo(new Vector2d(0, 56)) // Push Gate
                .waitSeconds(2) // Wait for balls to exit the gate

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(row2.position, row2.heading)
                .setTangent(row2.heading)
                .lineToY(48) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(row3.position, row3.heading)
                .setTangent(row3.heading)
                .lineToY(48) // Intake

                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .waitSeconds(1) // Launch balls

                .strafeToLinearHeading(gatePose.position, gatePose.heading)

                .build());

        // Driving Testbot
        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.25, 16.378)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .build();

        testBot.runAction(myRedBot.getDrive().actionBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(shootPose.position, shootPose.heading)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myRedBot)
                //.addEntity(testBot)
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