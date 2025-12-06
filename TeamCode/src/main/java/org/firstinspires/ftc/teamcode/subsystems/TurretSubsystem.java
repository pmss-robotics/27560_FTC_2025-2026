package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class TurretSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;

    // TODO: Rename these
    ServoImplEx servo1;
    ServoImplEx servo2;
    private static double angle = 0;
    public static double minAngle = -90, maxAngle = 90, turnDelay = 400, allowableError = 1, manualAutoReset = 5000, turnMult = 0.05;
    private ElapsedTime turnTimer;
    private ElapsedTime manualTimer;


    public States.TurretMode turretMode = States.TurretMode.autoAprilTag;

    public TurretSubsystem (HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;


        // TODO: Rename these
        servo1 = hardwareMap.get(ServoImplEx.class, "turret1");
        servo2 = hardwareMap.get(ServoImplEx.class, "turret2");

        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        angle = 0;
        servo1.setPosition(scale(angle));
        servo2.setPosition(scale(angle));


        turretMode = States.TurretMode.autoAprilTag;

        turnTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        manualTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void periodic() {
        /*
        Put necessary control flows that need to be updated per command scheduler run
        Similar functionality can be accomplished through default commands.
        I think we should limit periodic() to telemetry and other things alike
         */
    }

    /**
     * Use turn for absolute angles
     * @param angle the angle in degrees to rotate the turret to
     */
    public void turn(double angle) {
        turnTimer.reset();
        if (Double.isNaN(angle)) {
            return;
        }
        angle = Range.clip(angle, minAngle, maxAngle);
        servo1.setPosition(scale(angle));
        servo2.setPosition(scale(angle));
        TurretSubsystem.angle = angle;
    }

    /**
     * Use for relative angle positioning (with timer)
     * @param offset the angle difference for the
     */
    public void turnTo(double offset) {
        /*
        if (manualTimer.time() > manualAutoReset) {
            turretMode = States.TurretMode.autoAprilTag;
        }*/

        //if (turretMode == States.TurretMode.autoAprilTag) {
            if (turnTimer.time() > turnDelay && Math.abs(offset) > allowableError) {
                turn(offset);
                turnTimer.reset();
            }
        //}
    }

    public void manualTurn(double offset) {
        double delta;
        if (manualTimer.time() > manualAutoReset) {
            delta = 1;
        } else {
            delta = manualTimer.time();
        }
        if (offset != 0) {
            manualTimer.reset();
            turn(delta * offset * turnMult + getAngle());
        }
    }

    public double getAngle() {
        return angle;
    }

    private double scale(double angle) {
        return Range.scale(angle, -432, 432, 0, 1);
    }
}
