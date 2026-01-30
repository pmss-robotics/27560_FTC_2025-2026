package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodSubsystem extends SubsystemBase {
    ServoImplEx leftHood, rightHood;

    Telemetry telemetry;

    // TODO: Actually set these values based on cad/spec
    public static double minAngle = 30, maxAngle = 48, gearRatio = 353.0/36.0;

    public HoodSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");

        leftHood.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightHood.setPwmRange(new PwmControl.PwmRange(500, 2500));

        setPosition(scale(minAngle));
    }

    private void setPosition(double[] angles) {
        leftHood.setPosition(angles[0]);
        rightHood.setPosition(angles[1]);
    }
    private double[] scale(double angle) {


        angle = clamp(angle, minAngle, maxAngle);

        angle = Range.scale(angle, minAngle, maxAngle, 0, maxAngle-minAngle);

        // Hood angle - an angle between 30-48 degrees
        // Actual angle / Gear Ratio = Servo angle

        double servoside = angle / gearRatio;

        Range.scale(servoside, 0, 300, 0, 1);

        /*
        double left = Range.scale(angle, minAngle, maxAngle, leftMin, leftMax);
        double right = Range.scale(angle, minAngle, maxAngle, rightMin, rightMax);
        */

        double left = Range.scale(servoside, 0, 300, 0, 1);
        double right = Range.scale(servoside, 0, 300, 1, 0);

        return new double[]{0, 0};
    }

}
