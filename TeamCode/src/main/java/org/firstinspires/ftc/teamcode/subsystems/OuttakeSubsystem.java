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

    public States.Flywheel currentState;

    private VoltageSensor voltageSensor;

    private double speed;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        currentState = States.Flywheel.stopped;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        speed = 0;
    }

    public void toggleState() {
        switch (currentState) {
            case stopped:
                speed = 11;
                currentState = States.Flywheel.spinning;
            case spinning:
                speed = 0;
                currentState = States.Flywheel.stopped;
        }
        setPower(speed);
    }

    public void holdSpeed() {
        flywheel.setPower(clamp(speed/voltageSensor.getVoltage(),0,1));
    }

    public void setPower(double power) {
        speed = power;
        power /= voltageSensor.getVoltage();

        flywheel.setPower(clamp(power,-1.0,1.0));

        if (power == 0) {
            currentState = States.Flywheel.stopped;
        } else {
            currentState = States.Flywheel.spinning;
        }
    }
}
