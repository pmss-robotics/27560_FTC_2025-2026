package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.settings.SettingsOpMode;

@TeleOp(name = "Debug Settings", group = "Testing")
public class DebugSettings extends SettingsOpMode {
    @Override
    public void defineSettings() {
        add("loop_detect_mode", "Loop Detection Mode", new BooleanPrompt("Enable loop time detection?", false));
    }


}
