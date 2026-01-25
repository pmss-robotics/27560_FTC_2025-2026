package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

import java.util.List;

@Config
@TeleOp(name = "Flywheel Tuner", group = "Testing")
public class FlywheelTuner extends CommandOpMode {
    GamepadEx driver1, driver2;

    OuttakeSubsystem flywheel;
    LoopTimer timer;
    public static double off = 0, close = 1000, far = 3000;
    public static List<String> modes;
    public static List<Double> increments;
    public static double increment;
    public static int modeIndex, incrementIndex;

    Telemetry.Item mode, pidf, target, speed;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        if (Settings.get("loop_detect_mode", false)) {
            timer = new LoopTimer(telemetry, "Main");
        }

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap, telemetry, false);
        outtake.setDefaultCommand(new RunCommand(outtake::holdSpeed, outtake));

        modeIndex = 0;
        modes.add("F");
        modes.add("P");
        modes.add("D");
        modes.add("I");

        incrementIndex = 0;
        increments.add(10.0);
        increments.add(1.0);
        increments.add(0.1);
        increments.add(0.01);
        increments.add(0.001);

        mode = telemetry.addData("Mode", "%s %f", modes.get(modeIndex), increments.get(incrementIndex));
        pidf = telemetry.addData("PIDF","P %f I %f D %f F %f", OuttakeSubsystem.P, OuttakeSubsystem.I, OuttakeSubsystem.D, OuttakeSubsystem.F);
        target = telemetry.addData("target", outtake.targetRpm);
        speed = telemetry.addData("measured", outtake.speed);

        // Increment the pidf (DPAD l/r) Change size of increment (DPAD up/down)
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> {
                    switch (modes.get(modeIndex)) {
                        case "P": OuttakeSubsystem.P -= increment; break;
                        case "I": OuttakeSubsystem.I -= increment; break;
                        case "D": OuttakeSubsystem.D -= increment; break;
                        case "F": OuttakeSubsystem.F -= increment; break;
                    }
                    outtake.updatePIDF();
                }
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                () -> {
                    switch (modes.get(modeIndex)) {
                        case "P": OuttakeSubsystem.P += increment; break;
                        case "I": OuttakeSubsystem.I += increment; break;
                        case "D": OuttakeSubsystem.D += increment; break;
                        case "F": OuttakeSubsystem.F += increment; break;
                    }
                    outtake.updatePIDF();
                }
        );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> {
                    if(incrementIndex < increments.size() - 2) {
                        incrementIndex += 1;
                        increment = increments.get(incrementIndex);
                    }
                }
        );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> {
                    if(incrementIndex > 0) {
                        incrementIndex -= 1;
                        increment = increments.get(incrementIndex);
                    }
                }
        );

        // Next PIDF l/r bumpers
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> {
                    if(modeIndex < modes.size() - 2) {
                        modeIndex += 1;
                    }
                }
        );

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    if(modeIndex > 0) {
                        modeIndex -= 1;
                    }
                }
        );

        // Driver 2 Button A far speed
        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(far), outtake)
        );

        // Driver 2 Button B close speed
        driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(close), outtake)
        );

        // Driver 2 Button X off
        driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(off), outtake)
        );


        schedule(new RunCommand(() -> {
                    mode.setValue("%s %f", modes.get(modeIndex), increments.get(incrementIndex));
                    pidf.setValue("P %f I %f D %f F %f", OuttakeSubsystem.P, OuttakeSubsystem.I, OuttakeSubsystem.D, OuttakeSubsystem.F);
                    target.setValue(outtake.targetRpm);
                    speed.setValue(outtake.speed);
                })
        );
    }
}
