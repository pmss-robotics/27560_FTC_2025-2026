package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ObeliskVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretVisionSubsystem;


@TeleOp(name = "VisionTesting", group = "Testing")
public class VisionTesting extends CommandOpMode {

    ObeliskVisionSubsystem obeliskVision;
    TurretVisionSubsystem turretVision;

    GamepadEx tools;


    @Override
    public void initialize() {

        // Telemetry on 192.168.49.1:8080/dash
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);


        try {
            obeliskVision = new ObeliskVisionSubsystem(hardwareMap, telemetry, true);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Add april tag stuff
        tools = new GamepadEx(gamepad2);

        new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    obeliskVision.enableDetection(true);
                    telemetry.addData("Detecting?","true");
                    telemetry.update();
                }));
        new GamepadButton(tools, GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    obeliskVision.enableDetection(false);
                    telemetry.addData("Detecting?", "false");
                    telemetry.update();
                }));


    }




}
