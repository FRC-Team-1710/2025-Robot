package frc.robot.utils;

import java.util.Random;

public class TargetingComputer {

  public static final boolean gameMode = false;

  public static Targets currentTargetBranch = Targets.CHARLIE;
  public static double currentTargetLevel = 4;
  public static Random random = new Random();
  public static int randomBranch;
  public static int branchGameScore = 0;

  private static boolean isRedAlliance;

  public static int alphaTag,
      charlieTag,
      echoTag,
      golfTag,
      indiaTag,
      kiloTag,
      leftSourceTag,
      rightSourceTag,
      processorTag,
      netTag;
  public static double alphaAngle,
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
    isRedAlliance = redAlliance;
  }

  public static int getTagForTarget(Targets target) {
    return switch (target) {
      case ALPHA -> isRedAlliance ? 7 : 18;
      case BRAVO -> isRedAlliance ? 7 : 18;
      case CHARLIE -> isRedAlliance ? 8 : 17;
      case DELTA -> isRedAlliance ? 8 : 17;
      case ECHO -> isRedAlliance ? 9 : 22;
      case FOXTROT -> isRedAlliance ? 9 : 22;
      case GOLF -> isRedAlliance ? 10 : 21;
      case HOTEL -> isRedAlliance ? 10 : 21;
      case INDIA -> isRedAlliance ? 11 : 20;
      case JULIET -> isRedAlliance ? 11 : 20;
      case KILO -> isRedAlliance ? 6 : 19;
      case LIMA -> isRedAlliance ? 6 : 19;
      case SOURCE_LEFT -> isRedAlliance ? 1 : 13;
      case SOURCE_RIGHT -> isRedAlliance ? 2 : 12;
      case RED_PROCESSOR -> isRedAlliance ? 3 : 16;
      case RED_NET -> isRedAlliance ? 5 : 14;
    };
  }

  public static double getAngleForTarget(Targets target) {
    return switch (target) {
      case ALPHA, BRAVO -> isRedAlliance ? 180 : 0;
      case CHARLIE, DELTA -> isRedAlliance ? 240 : 60;
      case ECHO, FOXTROT -> isRedAlliance ? 300 : 120;
      case GOLF, HOTEL -> isRedAlliance ? 0 : 180;
      case INDIA, JULIET -> isRedAlliance ? 60 : 240;
      case KILO, LIMA -> isRedAlliance ? 120 : 300;
      case SOURCE_LEFT -> isRedAlliance ? 306 : 126;
      case SOURCE_RIGHT -> isRedAlliance ? 54 : 234;
      case RED_PROCESSOR -> isRedAlliance ? 90 : 270;
      case RED_NET -> isRedAlliance ? 180 : 0;
    };
  }

  public static void setTargetBranch(Targets target) {
    currentTargetBranch = target;
  }

  public static void setTargetLevel(int level) {
    currentTargetLevel = level;
  }

  public static Targets getCurrentTargetBranch() {
    return currentTargetBranch;
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

  public static Targets getCurrentTargetForBranchGame() {
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
    ALPHA(1, 0),
    BRAVO(0, 1),
    CHARLIE(1, 2),
    DELTA(0, 3),
    ECHO(1, 4),
    FOXTROT(0, 5),
    GOLF(1, 6),
    HOTEL(0, 7),
    INDIA(1, 8),
    JULIET(0, 9),
    KILO(1, 10),
    LIMA(0, 11),
    SOURCE_LEFT(0, 12),
    SOURCE_RIGHT(0, 13),
    RED_PROCESSOR(0, 14),
    RED_NET(0, 15);

    public final int preferredCamera;
    public final int gameID;

    Targets(int preferredCamera, int gameID) {
      this.preferredCamera = preferredCamera;
      this.gameID = gameID;
    }

    public int getApriltag() {
      return getTagForTarget(this);
    }

    public double getTargetingAngle() {
      return getAngleForTarget(this);
    }

    public int getPreferredCamera() {
      return preferredCamera;
    }

    public int gameID() {
      return gameID;
    }
  }
}
