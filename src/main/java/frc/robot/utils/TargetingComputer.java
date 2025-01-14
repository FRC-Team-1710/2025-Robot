package frc.robot.utils;

public class TargetingComputer {

  private static int alphaTag,
      charlieTag,
      echoTag,
      golfTag,
      indiaTag,
      kiloTag,
      leftSourceTag,
      rightSourceTag,
      processorTag,
      netTag;
  private static double alphaAngle,
      charlieAngle,
      echoAngle,
      golfAngle,
      indiaAngle,
      kiloAngle,
      leftSourceAngle,
      rightSourceAngle,
      processorAngle,
      netAngle;

  public static Targets currentTargetBranch = Targets.ALPHA;
  public static double currentTargetLevel = 4;

  public static void setAlliance(boolean redAlliance) {
    if (redAlliance) {
      alphaTag = 7;
      charlieTag = 8;
      echoTag = 9;
      golfTag = 10;
      indiaTag = 11;
      kiloTag = 6;
      leftSourceTag = 1;
      rightSourceTag = 2;
      processorTag = 3;
      netTag = 5;

      alphaAngle = 180;
      charlieAngle = 240;
      echoAngle = 300;
      golfAngle = 0;
      indiaAngle = 60;
      kiloAngle = 120;
      leftSourceAngle = 306;
      rightSourceAngle = 54;
      processorAngle = 90;
      netAngle = 180;
    } else {
      alphaTag = 18;
      charlieTag = 17;
      echoTag = 22;
      golfTag = 21;
      indiaTag = 20;
      kiloTag = 19;
      leftSourceTag = 13;
      rightSourceTag = 12;
      processorTag = 16;
      netTag = 14;

      alphaAngle = 0;
      charlieAngle = 60;
      echoAngle = 120;
      golfAngle = 180;
      indiaAngle = 240;
      kiloAngle = 300;
      leftSourceAngle = 126;
      rightSourceAngle = 234;
      processorAngle = 270;
      netAngle = 0;
    }
  }

  public static void setTargetBranch(Targets target) {
    currentTargetBranch = target;
  }

  public static void setTargetLevel(int level) {
    currentTargetLevel = level;
  }

  public enum Targets {
    ALPHA(alphaTag, alphaAngle, "left"),
    BRAVO(alphaTag, alphaAngle, "right"),
    CHARLIE(charlieTag, charlieAngle, "left"),
    DELTA(charlieTag, charlieAngle, "right"),
    ECHO(echoTag, echoAngle, "left"),
    FOXTROT(echoTag, echoAngle, "right"),
    GOLF(golfTag, golfAngle, "left"),
    HOTEL(golfTag, golfAngle, "right"),
    INDIA(indiaTag, indiaAngle, "left"),
    JULIET(indiaTag, indiaAngle, "right"),
    KILO(kiloTag, kiloAngle, "left"),
    LIMA(kiloTag, kiloAngle, "right"),
    SOURCE_LEFT(leftSourceTag, leftSourceAngle, null),
    SOURCE_RIGHT(rightSourceTag, rightSourceAngle, null),
    RED_PROCESSOR(processorTag, processorAngle, null),
    RED_NET(processorTag, processorAngle, null);

    private final double apriltag;
    private final double targetingAngle; // in deg
    private final String side;

    Targets(double apriltag, double targetingAngle, String side) {
      this.apriltag = apriltag;
      this.targetingAngle = targetingAngle;
      this.side = side;
    }

    public double getApriltag() {
      return apriltag;
    }

    public double getTargetingAngle() {
      return targetingAngle;
    }

    public String getSide() {
      return side;
    }
  }
}
