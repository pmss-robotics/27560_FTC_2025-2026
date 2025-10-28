package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    }

    public void toggleState() {
        // TODO: Set power based on state
    }
}
