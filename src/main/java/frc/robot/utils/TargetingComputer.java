package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum TargetingComputer {
    RED_ALPHA (7, 180, "left"),
    RED_BRAVO (7, 180, "right"),
    RED_CHARLIE (8, 240, "left"),
    RED_DELTA (8, 240, "right"),
    RED_ECHO (9, 300, "left"),
    RED_FOXTROT (9, 300, "right"),
    RED_GOLF (10, 0, "left"),
    RED_HOTEL (10, 0, "right"),
    RED_INDIA (11, 60, "left"),
    RED_JULIET (11, 60, "right"),
    RED_KILO (6, 120, "left"),
    RED_LIMA (6, 120, "right"),
    RED_SOURCE_LEFT (1, 306, null),
    RED_SOURCE_RIGHT (2, 54, null),
    RED_PROCESSOR (3, 90, null),
    RED_NET (5, 180, null),

    BLUE_ALPHA (18, 0, "left"),
    BLUE_BRAVO (18, 0, "right"),
    BLUE_CHARLIE (17, 60, "left"),
    BLUE_DELTA (17, 60, "right"),
    BLUE_ECHO (22, 120, "left"),
    BLUE_FOXTROT (22, 120, "right"),
    BLUE_GOLF (21, 180, "left"),
    BLUE_HOTEL (21, 180, "right"),
    BLUE_INDIA (20, 240, "left"),
    BLUE_JULIET (20, 240, "right"),
    BLUE_KILO (19, 300, "left"),
    BLUE_LIMA (19, 300, "right"),
    BLUE_SOURCE_LEFT (13, 126, null),
    BLUE_SOURCE_RIGHT (12, 234, null),
    BLUE_PROCESSOR (16, 270, null),
    BLUE_NET (14, 0, null);

    private final double apriltag;
    private final double targetingAngle; // in deg
    private final String side;

    TargetingComputer(double apriltag, double targetingAngle, String side) {
        this.apriltag = apriltag;
        this.targetingAngle = targetingAngle;
        this.side = side;
    }

    public double getApriltag() {return apriltag;}
    public double getTargetingAngle() {return targetingAngle;}
    public String getSide() {return side;}
}
