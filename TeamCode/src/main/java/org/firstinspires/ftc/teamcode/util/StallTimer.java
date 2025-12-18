package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class StallTimer {
    private final ElapsedTime timer;
    private boolean stalled;
    private double timeout;

    public StallTimer(double timeout, ElapsedTime.Resolution resolution) {
        this.timeout = timeout;
        timer = new ElapsedTime(resolution);
        stalled = false;
    }

    public void motorOn() {
        stalled = false;
    }

    public void stalling() {
        if (!stalled) {
            stalled = true;
            timer.reset();
        }
    }

    public boolean shutOff() {
        return (stalled && timer.time() > timeout);
    }
}
