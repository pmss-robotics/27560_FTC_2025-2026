package org.firstinspires.ftc.teamcode.subsystems;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

public class OuttakeSubsystem extends SubsystemBase {
    private DcMotorEx flywheel;

    private Telemetry telemetry;

    private States.Flywheel currentState;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        currentState = States.Flywheel.stopped;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        // TODO: Set power based on state

        flywheel.setPower(clamp(power,-1.0,1.0));
        if (power ==0) {
            currentState = States.Flywheel.spinning;
        } else {
            currentState = States.Flywheel.stopped;
        }
    }
}
