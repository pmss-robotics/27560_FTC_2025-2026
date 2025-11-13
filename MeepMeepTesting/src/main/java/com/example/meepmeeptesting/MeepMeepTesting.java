package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
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
                .strafeTo(new Vector2d(-29, 29))
                .turnTo(Math.toRadians(135))
                // Between here launch 3 balls
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
        Pose2d bluePose = new Pose2d(-40, -54, Math.PI);

        myBlueBot.runAction(myBlueBot.getDrive().actionBuilder(bluePose)
                .strafeTo(new Vector2d(-29, -29))
                .turnTo(Math.toRadians(215))
                // Between here launch 3 balls
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(-10,-29))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myRedBot)
                .addEntity(myBlueBot)
                .start();
    }

}