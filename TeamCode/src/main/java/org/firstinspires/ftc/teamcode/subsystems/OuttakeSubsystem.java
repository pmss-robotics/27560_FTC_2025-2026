package org.firstinspires.ftc.teamcode.subsystems;


import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.lights.RGBIndicator;
import com.qualcomm.robotcore.hardware.Light;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import com.seattlesolvers.solverslib.util.InterpLUT;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.StallTimer;
import org.firstinspires.ftc.teamcode.util.States;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    private DcMotorEx flywheel;
    private ServoImplEx leftKicker, rightKicker, light;
    private Telemetry telemetry;
    private VoltageSensor voltageSensor;

    public States.Flywheel flywheelState;
    public States.Kicker kickerState;

    private InterpLUT lut;
    public static double flywheelMaxCurrent = 7, flywheelStallTimeout = 3000;
    public static double farShot = 3600, closeShot = 2920;
    public static double P = 29.0, I=0, D=0.2, F=14;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;
    public double targetRpm = 0.0;


    private PIDFController pidf;
    public static double lHome = 210, lKick = 100, rHome = 45, rKick = 155;
    public double speed;
    private StallTimer stallTimer;

    /**
     * To access Flywheel and Servo Kicker control
     * @param hardwareMap the OpMode's hardwareMap
     * @param telemetry the OpMode's telemetry
     * @param useKicker whether to enable the servo kicker; use false when servos aren't plugged in
     **/
    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean useKicker) {
        this.telemetry = telemetry;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        light = hardwareMap.get(ServoImplEx.class, "rgb");

        if (useKicker) {
            leftKicker = hardwareMap.get(ServoImplEx.class, "leftKicker");
            rightKicker = hardwareMap.get(ServoImplEx.class, "rightKicker");
        }

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        //flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidf = new PIDFController(P, I, D, F, 0, flywheel.getVelocity());

        flywheel.setPower(pidf.calculate(flywheel.getVelocity()));
        // createLut();

        stallTimer = new StallTimer(flywheelStallTimeout, ElapsedTime.Resolution.MILLISECONDS);

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

    public void createLUT() {
        lut = new InterpLUT();
        //Table entries
        lut.add(0,0);
        lut.createLUT();
    }

    public void holdSpeed() {
        speed = flywheel.getVelocity();
        flywheel.setPower(clamp(pidf.calculate(speed)/voltageSensor.getVoltage(),-1,1));

        //telemetry.addData("flywheel voltage", speed);
        //telemetry.addData("flywheel current", flywheel.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("flywheel stall", !flywheel.isMotorEnabled());

    }

//    public void setPower(double power) {
//        //if (power > 0) speed = 1;
//        //else speed = 0;
//        speed = power;
//        power /= voltageSensor.getVoltage();
//
//        //flywheel.setPower(speed);
//        flywheel.setPower(clamp(power,-1.0,1.0));
//
//        if (power == 0) {
//            flywheelState = States.Flywheel.stopped;
//        } else {
//            flywheelState = States.Flywheel.spinning;
//        }
//    }
    public void setPidfCoefficients(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
        pidf.setPIDF(P,I,D,F);
    }

    public void updatePIDF() {
        pidf.setPIDF(P,I,D,F);
    }

    public void setVelocityRpm(double rpm) {
        targetRpm = rpm;
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        pidf.setSetPoint(ticksPerSec);
        speed = flywheel.getVelocity();
        flywheel.setPower(clamp(pidf.calculate(speed)/voltageSensor.getVoltage(),-1,1));
    }
    public void setVelocityByDistance(double distance) {
        setVelocityRpm(lut.get(distance));
    }
    public void kick() {
        kickerState = States.Kicker.kick;
        leftKicker.setPosition(scale(lKick));
        rightKicker.setPosition(scale(rKick));
    }

    public void home() {
        kickerState = States.Kicker.home;
        leftKicker.setPosition(scale(lHome));
        rightKicker.setPosition(scale(rHome));
    }

    public void resetMotor() {
        stallTimer.motorOn();
        flywheel.setMotorEnable();
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

    public void setColor(double position) {
        light.setPosition(position);
    }
}
