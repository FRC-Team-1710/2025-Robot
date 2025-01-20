package frc.robot.utils;

import java.util.Random;

public class TargetingComputer {

  public static final boolean gameMode = false;

  public static Targets currentTargetBranch = Targets.ALPHA;
  public static double currentTargetLevel = 4;
  public static Random random = new Random();
  public static int randomBranch;
  public static int branchGameScore = 0;

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

  public static void randomizeTargetBranch() {
    randomBranch = random.nextInt(12);
  }

  public static void checkBranchGame() {
    if (randomBranch == currentTargetBranch.gameID) {
      randomizeTargetBranch();
      branchGameScore++;
      while (randomBranch == currentTargetBranch.gameID) {
        randomizeTargetBranch();
      }
    }
  }

  public static void startBranchGame() {
    setTargetBranch(Targets.ALPHA);
    randomBranch = random.nextInt(11) + 1;
    branchGameScore = 0;
  }

  public static Targets getCurrentBranchGameTarget() {
    Targets gameTarget = Targets.ALPHA;

    if (randomBranch == Targets.ALPHA.gameID) gameTarget = Targets.ALPHA;
    else if (randomBranch == Targets.BRAVO.gameID) gameTarget = Targets.BRAVO;
    else if (randomBranch == Targets.CHARLIE.gameID) gameTarget = Targets.CHARLIE;
    else if (randomBranch == Targets.DELTA.gameID) gameTarget = Targets.DELTA;
    else if (randomBranch == Targets.ECHO.gameID) gameTarget = Targets.ECHO;
    else if (randomBranch == Targets.FOXTROT.gameID) gameTarget = Targets.FOXTROT;
    else if (randomBranch == Targets.GOLF.gameID) gameTarget = Targets.GOLF;
    else if (randomBranch == Targets.HOTEL.gameID) gameTarget = Targets.HOTEL;
    else if (randomBranch == Targets.INDIA.gameID) gameTarget = Targets.INDIA;
    else if (randomBranch == Targets.JULIET.gameID) gameTarget = Targets.JULIET;
    else if (randomBranch == Targets.KILO.gameID) gameTarget = Targets.KILO;
    else if (randomBranch == Targets.LIMA.gameID) gameTarget = Targets.LIMA;

    return gameTarget;
  }

  public enum Targets {
    ALPHA(alphaTag, alphaAngle, "left", 0),
    BRAVO(alphaTag, alphaAngle, "right", 1),
    CHARLIE(charlieTag, charlieAngle, "left", 2),
    DELTA(charlieTag, charlieAngle, "right", 3),
    ECHO(echoTag, echoAngle, "left", 4),
    FOXTROT(echoTag, echoAngle, "right", 5),
    GOLF(golfTag, golfAngle, "left", 6),
    HOTEL(golfTag, golfAngle, "right", 7),
    INDIA(indiaTag, indiaAngle, "left", 8),
    JULIET(indiaTag, indiaAngle, "right", 9),
    KILO(kiloTag, kiloAngle, "left", 10),
    LIMA(kiloTag, kiloAngle, "right", 11),
    SOURCE_LEFT(leftSourceTag, leftSourceAngle, null, 12),
    SOURCE_RIGHT(rightSourceTag, rightSourceAngle, null, 13),
    RED_PROCESSOR(processorTag, processorAngle, null, 14),
    RED_NET(processorTag, processorAngle, null, 15);

    private final double apriltag;
    private final double targetingAngle; // in deg
    private final String side;
    private final int gameID;

    Targets(double apriltag, double targetingAngle, String side, int gameID) {
      this.apriltag = apriltag;
      this.targetingAngle = targetingAngle;
      this.side = side;
      this.gameID = gameID;
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
