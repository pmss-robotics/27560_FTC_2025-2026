package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretVisionSubsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
import org.firstinspires.ftc.teamcode.util.States;

@TeleOp(name = "TurretTest", group = "Testing")
public class TurretTest extends CommandOpMode {
    private GamepadEx driver1;
    private TurretSubsystem turret;
    private TurretVisionSubsystem turretVision;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);

        turret = new TurretSubsystem(hardwareMap, telemetry);
        try {
            turretVision = new TurretVisionSubsystem(hardwareMap, telemetry, false);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        turretVision.enableDetection(true);

        turret.setDefaultCommand(new RunCommand(() -> turret.turnTo(turretVision.update()+turret.getAngle(), () -> true), turret, turretVision));

        new GamepadButton(driver1, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> turret.turn(0)));

        new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> StateTransfer.alliance = States.Alliance.Red));

        new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> StateTransfer.alliance = States.Alliance.Blue));
    }
}
