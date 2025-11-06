package org.firstinspires.ftc.teamcode.subsystems;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

public class IntakeSubsystem extends SubsystemBase {

    private DcMotorEx intakeMotor;

    private Telemetry telemetry;

    public States.Intake currentState;

    private VoltageSensor voltageSensor;

    private double speed;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        currentState = States.Intake.stopped;

        speed = 0;
    }

    /*
    public void setCurrentState(States.Intake state) {
        switch (state) {
            case stopped:
                speed = 0;
            case feeding:
                speed = 12;
            case reverse:
                speed = -12;
        }
    }

    public void holdSpeed() {
        intakeMotor.setPower(speed);
    }
    */

    public void setPower(double power) {
        // power/=2;
        intakeMotor.setPower(clamp(power/voltageSensor.getVoltage(),-1,1));
    }
}
