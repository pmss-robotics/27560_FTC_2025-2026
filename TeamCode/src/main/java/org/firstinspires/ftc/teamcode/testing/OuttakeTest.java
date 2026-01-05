package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OuttakeTest", group = "Testing")
public class OuttakeTest extends CommandOpMode{
    GamepadEx driver1, driver2;

    OuttakeSubsystem flywheel;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry, false);


        // Close shot
        new GamepadButton(driver2, GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(() -> outtake.setVelocityRpm(2900)),
                        new InstantCommand(() -> outtake.setVelocityRpm(0))
                );

        // Far shot
        new GamepadButton(driver2, GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new InstantCommand(() -> outtake.setVelocityRpm(3650)),
                        new InstantCommand(() -> outtake.setVelocityRpm(0))
                );

    }
}
