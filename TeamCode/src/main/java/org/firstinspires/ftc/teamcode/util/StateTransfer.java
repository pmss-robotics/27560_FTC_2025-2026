package org.firstinspires.ftc.teamcode.util;

// import com.pedropathing.localization.Pose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;

@Config
public class StateTransfer {
    // public static Pose pose = new Pose();

    public static States.Obelisk motif = States.Obelisk.unread;

    public static States.Alliance alliance;
    public static Pose2d pose = new Pose2d(0,0,Math.PI/2);
    public static Pose posePedro = new Pose(0,0,0);
}
