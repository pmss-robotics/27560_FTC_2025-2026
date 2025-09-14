package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;


@TeleOp(name = "VisionTesting")
public class VisionTesting extends CommandOpMode {

    VisionSubsystem vision;

    GamepadEx tools;


    @Override
    public void initialize() {

        // Telemetry on 192.168.49.1:8080/dash
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // Add april tag stuff
        tools = new GamepadEx(gamepad2);

        new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    vision.enableDetection(true);

                }));
        new GamepadButton(tools, GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    vision.enableDetection(false);
                }));
    }




}
