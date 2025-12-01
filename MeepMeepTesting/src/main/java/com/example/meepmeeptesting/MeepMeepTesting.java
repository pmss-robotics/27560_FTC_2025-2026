package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.InternalPosition.flipY;

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

        Pose2d redPose = new Pose2d(-40, 54, Math.toRadians(180));

        myRedBot.runAction(myRedBot.getDrive().actionBuilder(redPose)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-29, 29), Math.toRadians(0))
                .turnTo(toObject(myRedBot.getPose(), new Vector2d(-60,60)))
                .waitSeconds(3) // Launch 3 Balls here
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-10,29))
                .build());

        // Blue Auto

        RoadRunnerBotEntity myBlueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.25, 16.378)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();
        Pose2d bluePose = new Pose2d(40, 54, 0);

        myBlueBot.runAction(myBlueBot.getDrive().actionBuilder(bluePose)
                .strafeTo(new Vector2d(29, 29))
                .turnTo(Math.toRadians(45))
                .waitSeconds(3)// Between here launch 3 balls
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(10,29))
                .build());

        RoadRunnerBotEntity strangeBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.25, 16.378)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        Pose2d strangePose = new Pose2d(0,0,0);
        strangeBot.runAction(strangeBot.getDrive().actionBuilder(strangePose)
                        .turnTo(Math.PI/2)
                        .turnTo(Math.PI)
                        .turnTo(Math.PI*1.5)
                        .turnTo(0)
                        .build());

        
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myRedBot)
                //.addEntity(myBlueBot)
                .addEntity(strangeBot)
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