package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class TurretSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;

    // TODO: Rename these
    ServoImplEx servo1;
    ServoImplEx servo2;
    private static double angle = 0;

    public TurretSubsystem (HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;


        // TODO: Rename these
        servo1 = hardwareMap.get(ServoImplEx.class, "turret1");
        servo2 = hardwareMap.get(ServoImplEx.class, "turret2");

        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void periodic() {
        /*
        Put necessary control flows that need to be updated per command scheduler run
        Similar functionality can be accomplished through default commands.
        I think we should limit periodic() to telemetry and other things alike
         */
    }

    public void turn(double angle) {
        servo1.setPosition(scale(angle));
        servo2.setPosition(scale(angle));
        TurretSubsystem.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    private double scale(double angle) {
        // TODO: Set the range based on the gear ratio of the servos (how many turns the actual turret moves not the servos)
        return Range.scale(angle, -90, 90, 0, 1);
    }
}
