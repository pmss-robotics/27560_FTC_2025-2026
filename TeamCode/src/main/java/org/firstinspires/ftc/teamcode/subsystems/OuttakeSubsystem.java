package org.firstinspires.ftc.teamcode.subsystems;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    private DcMotorEx flywheel;
    private ServoImplEx leftKicker, rightKicker;
    private Telemetry telemetry;
    private VoltageSensor voltageSensor;

    public States.Flywheel flywheelState;
    public States.Kicker kickerState;

    public static double flywheelVelocity = 12;

    public static double lHome = 210, lKick = 100, rHome = 45, rKick = 155;
    private double speed;
    private double kTarget;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean useKicker) {
        this.telemetry = telemetry;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        if (useKicker) {
            leftKicker = hardwareMap.get(ServoImplEx.class, "leftKicker");
            rightKicker = hardwareMap.get(ServoImplEx.class, "rightKicker");
        }

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelState = States.Flywheel.stopped;
        kickerState = States.Kicker.home;

        if (useKicker) {
            leftKicker.setPwmRange(new PwmControl.PwmRange(500, 2500));
            rightKicker.setPwmRange(new PwmControl.PwmRange(500, 2500));

            leftKicker.setPosition(scale(lHome));
            rightKicker.setPosition(scale(rHome));
        }

    }


    public void toggleFlywheelState() {
        switch (flywheelState) {
            case stopped:
                speed = flywheelVelocity;
                flywheelState = States.Flywheel.spinning;
                break;
            case spinning:
                speed = 0;
                flywheelState = States.Flywheel.stopped;
                break;
        }
        setPower(speed);
    }

    public void holdSpeed() {
        flywheel.setPower(clamp(speed/voltageSensor.getVoltage(),0,1));
        telemetry.addData("flywheel power", speed);
    }

    public void setPower(double power) {
        speed = power;
        power /= voltageSensor.getVoltage();

        flywheel.setPower(clamp(power,-1.0,1.0));

        if (power == 0) {
            flywheelState = States.Flywheel.stopped;
        } else {
            flywheelState = States.Flywheel.spinning;
        }
    }

    public void kick() {
        leftKicker.setPosition(scale(lKick));
        rightKicker.setPosition(scale(rKick));
    }

    public void home() {
        leftKicker.setPosition(scale(lHome));
        rightKicker.setPosition(scale(rHome));
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }
}
