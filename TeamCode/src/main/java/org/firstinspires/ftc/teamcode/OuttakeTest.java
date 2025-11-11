package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OuttakeTest extends CommandOpMode{
    GamepadEx driver1, driver2;

    OuttakeSubsystem flywheel;

    public static double driveMult = 1;

    @Override
    public void initialize(){



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        OuttakeSubsystem flywheel = new OuttakeSubsystem(hardwareMap, telemetry, false);


        new GamepadButton(driver2, GamepadKeys.Button.A)
                .whenReleased(new InstantCommand(() -> flywheel.setPower(11.0)));

        new GamepadButton(driver2, GamepadKeys.Button.B)
                .whenReleased(new InstantCommand(() -> flywheel.setPower(0.0)));

        new GamepadButton(driver2, GamepadKeys.Button.X)
                .toggleWhenPressed(
                        new InstantCommand(() -> flywheel.setPower(0.0)),
                        new InstantCommand(() -> flywheel.setPower(12.0)));

    }
}
