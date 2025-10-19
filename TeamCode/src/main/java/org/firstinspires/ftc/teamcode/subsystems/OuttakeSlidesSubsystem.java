package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

// tuning guide: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
@Config
public class OuttakeSlidesSubsystem extends SubsystemBase {

    private DcMotorEx leftExtension;
    private DcMotorEx rightExtension;
    private Telemetry telemetry;
    public static double P = 0.01, I = 0, D = 0; // p: 0.021, i: 0.003
    public static double kSpring = 0;
    public static int pHome = 0, pSpecimen = 0, pBucket = 4200, pStart = 0;
    public static int target = 0;
//    public static double resetPower = 0;
    public PIDController pidController;
    private VoltageSensor voltageSensor;
    private States.OuttakeExtension currentState;

    public OuttakeSlidesSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        currentState = States.OuttakeExtension.home;

        leftExtension = hardwareMap.get(DcMotorEx.class, "slideLeft");
        rightExtension = hardwareMap.get(DcMotorEx.class, "slideRight");
        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        target = 0;
        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    @Override
    public void periodic() {
        if (Objects.nonNull(leftExtension.getCurrentPosition())) {
            telemetry.addData("Extension Target: ", target);
            telemetry.addData("Extension Pos: ", leftExtension.getCurrentPosition());
            telemetry.addData("Extension state: ", currentState);
            telemetry.update();
        }
    }

    public void holdPosition() {
        leftExtension.setPower(calculate());
        rightExtension.setPower(calculate());
    }

    public void moveTo(int target) {
        OuttakeSlidesSubsystem.target = target;
        holdPosition();
    }

    public void manual(double power) {
        leftExtension.setPower(calculate() + power);
        rightExtension.setPower(calculate() + power);
        target = leftExtension.getCurrentPosition();
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = leftExtension.getCurrentPosition();

        double power = pidController.calculate(current, target)+kSpring;
        power /= voltageSensor.getVoltage();

        telemetry.addData("Extension Power:", power);
        return power;
    }

    public States.OuttakeExtension getCurrentOutExState() {
        return currentState;
    }

    public void toggleState() {
        switch (currentState) {
            case home:
                moveTo(pBucket);
                currentState = States.OuttakeExtension.bucket;
                break;
            case specimen:
            case bucket:
                moveTo(pHome);
                currentState = States.OuttakeExtension.home;
                break;
        }
    }


    public void resetEncoder () {
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = 0;
    }

    public void setState(States.OuttakeExtension state) {
        currentState = state;
        switch (currentState) {
            case bucket:
                moveTo(pBucket);
                break;
            case specimen: //maybe change later so it works
                moveTo(pSpecimen);
                break;
            case home:
                moveTo(pHome);
                break;
            case start:
                moveTo(pStart);
                break;
        }
    }
}
