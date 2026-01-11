package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class LoopTimer extends SubsystemBase {
    private Telemetry telemetry;
    private final ElapsedTime timer;
    private double lastFrameTime;
    private int frames;
    private String name;
    public boolean showHz = false;

    public LoopTimer(Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        frames = 1;
        lastFrameTime = 0;
    }

    @Override
    public void periodic() {
        frames++;

        telemetry.addData(name + " Loop Time (ms)", timer.time()-lastFrameTime);
        telemetry.addData(name + " Average Loop Time (ms)", timer.time()/frames);
        if (showHz) {
            telemetry.addData(name + " Average Loop Frequency (hz)", frames/timer.time(TimeUnit.SECONDS));

        }

        telemetry.update();

        lastFrameTime = timer.time();
    }

}