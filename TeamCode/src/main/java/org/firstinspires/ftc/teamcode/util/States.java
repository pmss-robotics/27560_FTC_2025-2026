package org.firstinspires.ftc.teamcode.util;

public class States {
    public enum Obelisk { // Will be used inside
        unread,
        GPP,
        PGP,
        PPG
    }

    public enum ObeliskVisionMode {
        detectLeft,
        detectRight,
        detectCenter
    }

    public enum OuttakeExtension { // Copied from T1 2024-2025 - Remember to change these values later - Once fixed delete this comment
        start,
        home,
        bucket,
        specimen,
    }

    public enum Flywheel {
        stopped,
        spinning
    }

    public enum Kicker {
        home,
        kick
    }

    public enum Intake {
        stopped,
        feeding,
        reverse
    }

    public enum Alliance {
        Red,
        Blue
    }

    public enum TurretMode {
        autoAprilTag,
        manual
    }
}
