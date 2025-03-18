package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.Random;

public class TargetingComputer {
  public static final boolean homeField = true; // TODO: Change before comp
  public static final boolean gameMode = false;
  public static final Translation2d primaryAlgaeOffset =
      new Translation2d(Units.inchesToMeters(32), 0);
  public static final Translation2d secondaryAlgaeOffset =
      new Translation2d(Units.inchesToMeters(23), 0);

  public static Targets currentTargetBranch = Targets.ALPHA;
  public static Levels currentTargetLevel = Levels.L4;
  public static Random random = new Random();
  public static int branchGameScore = 0;
  public static boolean targetingAlgae = false;

  /** should set the alignment target to the secondary grabbing pose */
  public static boolean readyToGrabAlgae = false;

  /**
   * should start the algae grabbing sequence (sets alignment target to the primary grabbing pose)
   */
  public static boolean aligningWithAlgae = false;

  public static boolean targetingControllerOverride = false;
  public static int randomBranch;
  public static int randomHeight;
  public static double sourceCutoffDistance = 7.5;
  public static boolean stillOuttakingAlgae = false;
  public static boolean goForClimb = false;

  public static final double alignmentTranslationTolerance = Units.inchesToMeters(1.5);
  public static final double alignmentAngleTolerance = 5;
  public static final double alignmentRange = 1;
  public static final double maxAlignSpeed = .3;

  private static final double xOffset = 17.5;
  private static final double yOffset = 7.5;
  private static final double homeYOffset = 1;

  // AprilTag Targeting
  public static boolean targetSet;

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
      case ALPHA, BRAVO -> isRedAlliance ? 7 : 18;
      case CHARLIE, DELTA -> isRedAlliance ? 8 : 17;
      case ECHO, FOXTROT -> isRedAlliance ? 9 : 22;
      case GOLF, HOTEL -> isRedAlliance ? 10 : 21;
      case INDIA, JULIET -> isRedAlliance ? 11 : 20;
      case KILO, LIMA -> isRedAlliance ? 6 : 19;
      case SOURCE_LEFT -> isRedAlliance ? 1 : 13;
      case SOURCE_RIGHT -> isRedAlliance ? 2 : 12;
      case PROCESSOR -> isRedAlliance ? 3 : 16;
      case NET -> isRedAlliance ? 5 : 14;
    };
  }

  public static void setTargetByTag(int tagID, boolean leftSide) {
    switch (tagID) {
      case 7, 18:
        setTargetBranch(leftSide ? Targets.ALPHA : Targets.BRAVO);
        break;
      case 8, 17:
        setTargetBranch(leftSide ? Targets.CHARLIE : Targets.DELTA);
        break;
      case 9, 22:
        setTargetBranch(leftSide ? Targets.ECHO : Targets.FOXTROT);
        break;
      case 10, 21:
        setTargetBranch(leftSide ? Targets.GOLF : Targets.HOTEL);
        break;
      case 11, 20:
        setTargetBranch(leftSide ? Targets.INDIA : Targets.JULIET);
        break;
      case 6, 19:
        setTargetBranch(leftSide ? Targets.KILO : Targets.LIMA);
        break;
      case 1, 13:
        setTargetBranch(Targets.SOURCE_LEFT);
        break;
      case 2, 12:
        setTargetBranch(Targets.SOURCE_RIGHT);
        break;
      case 3, 16:
        setTargetBranch(Targets.PROCESSOR);
        break;
      case 5, 14:
        setTargetBranch(Targets.NET);
        break;
    }
  }

  public static double getAngleForTarget(Targets target) {
    return switch (target) { // XOR RAAAHHHHHH!!
      case ALPHA, BRAVO -> isRedAlliance ^ currentTargetLevel == Levels.L1 ? 180 : 0;
      case CHARLIE, DELTA -> isRedAlliance ^ currentTargetLevel == Levels.L1 ? 240 : 60;
      case ECHO, FOXTROT -> isRedAlliance ^ currentTargetLevel == Levels.L1 ? 300 : 120;
      case GOLF, HOTEL -> isRedAlliance ^ currentTargetLevel == Levels.L1 ? 0 : 180;
      case INDIA, JULIET -> isRedAlliance ^ currentTargetLevel == Levels.L1 ? 60 : 240;
      case KILO, LIMA -> isRedAlliance ^ currentTargetLevel == Levels.L1 ? 120 : 300;
      case SOURCE_LEFT -> isRedAlliance ? 126 : 306;
      case SOURCE_RIGHT -> isRedAlliance ? 234 : 54;
      case PROCESSOR -> isRedAlliance ? 90 : 270;
      case NET -> isRedAlliance ? 180 : 0;
    };
  }

  public static void toggleTargetingControllerOverride() {
    targetingControllerOverride = !targetingControllerOverride;
  }

  public static void setTargetBranch(Targets target) {
    currentTargetBranch = target;
  }

  public static void setTargetBranchByOrientation(Pose2d pose) {
    if (Math.abs(new Rotation2d(Units.degreesToRadians(0)).minus(pose.getRotation()).getDegrees())
        < 30) {
      setTargetBranch(isRedAlliance ? Targets.GOLF : Targets.ALPHA);
    } else if (Math.abs(
            new Rotation2d(Units.degreesToRadians(60)).minus(pose.getRotation()).getDegrees())
        < 30) {
      setTargetBranch(isRedAlliance ? Targets.INDIA : Targets.CHARLIE);
    } else if (Math.abs(
            new Rotation2d(Units.degreesToRadians(120)).minus(pose.getRotation()).getDegrees())
        < 30) {
      setTargetBranch(isRedAlliance ? Targets.KILO : Targets.ECHO);
    } else if (Math.abs(
            new Rotation2d(Units.degreesToRadians(180)).minus(pose.getRotation()).getDegrees())
        < 30) {
      setTargetBranch(isRedAlliance ? Targets.ALPHA : Targets.GOLF);
    } else if (Math.abs(
            new Rotation2d(Units.degreesToRadians(240)).minus(pose.getRotation()).getDegrees())
        < 30) {
      setTargetBranch(isRedAlliance ? Targets.CHARLIE : Targets.INDIA);
    } else if (Math.abs(
            new Rotation2d(Units.degreesToRadians(300)).minus(pose.getRotation()).getDegrees())
        < 30) {
      setTargetBranch(isRedAlliance ? Targets.ECHO : Targets.KILO);
    }
  }

  public static void setTargetLevel(Levels level) {
    currentTargetLevel = level;
    targetingAlgae = false;
  }

  public static Targets getCurrentTargetBranch() {
    return currentTargetBranch;
  }

  public static Pose2d getCurrentTargetBranchPose() {
    return new Pose2d(
            FieldConstants.aprilTags
                .getTagPose(TargetingComputer.currentTargetBranch.getApriltag())
                .get()
                .getTranslation()
                .toTranslation2d(),
            FieldConstants.aprilTags
                .getTagPose(TargetingComputer.currentTargetBranch.getApriltag())
                .get()
                .getRotation()
                .toRotation2d())
        .plus(
            new Transform2d(
                TargetingComputer.currentTargetBranch.getOffset().getX(),
                TargetingComputer.currentTargetBranch.getOffset().getY(),
                new Rotation2d(Math.PI)));
  }

  public static Pose2d getSelectTargetBranchPose(Targets target) {
    switch (target.gameID()) {
      case 12, 13:
        return new Pose2d(
                FieldConstants.aprilTags
                    .getTagPose(target.getApriltag())
                    .get()
                    .getTranslation()
                    .toTranslation2d(),
                FieldConstants.aprilTags
                    .getTagPose(target.getApriltag())
                    .get()
                    .getRotation()
                    .toRotation2d())
            .plus(
                new Transform2d(
                    target.getOffset().getX(), target.getOffset().getY(), new Rotation2d()));
      default:
        return new Pose2d(
                FieldConstants.aprilTags
                    .getTagPose(target.getApriltag())
                    .get()
                    .getTranslation()
                    .toTranslation2d(),
                FieldConstants.aprilTags
                    .getTagPose(target.getApriltag())
                    .get()
                    .getRotation()
                    .toRotation2d())
            .plus(
                new Transform2d(
                    target.getOffset().getX(), target.getOffset().getY(), new Rotation2d(Math.PI)));
    }
  }

  public static boolean getAligningWithAlgae() {
    return aligningWithAlgae;
  }

  public static Levels getCurrentTargetLevel() {
    if (!aligningWithAlgae) {
      return currentTargetLevel;
    } else {
      return targetingAlgae ? getCurrentTargetBranch().getAlgaeLevel() : currentTargetLevel;
    }
  }

  public static void updateSourceCutoffDistance(boolean hasAlgae) {
    sourceCutoffDistance = hasAlgae ? 4.5 : 8.5;
  }

  public static void setStillOuttakingAlgae(boolean value) {
    stillOuttakingAlgae = value;
  }

  public static boolean stillInRangeOfSources(Pose2d pose) {
    return isRedAlliance
        ? pose.getX() <= sourceCutoffDistance
        : pose.getX() >= FieldConstants.fieldLength.in(Meters) - sourceCutoffDistance;
  }

  public static double getSourceTargetingAngle(Pose2d pose) {
    if (goForClimb) return Targets.PROCESSOR.getTargetingAngle();
    // if (isRedAlliance) {
    //   return (pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2) // red
    //       ? (pose.getX()
    //               >= FieldConstants.fieldLength.in(Meters) - sourceCutoffDistance) // top half
    //           ? Targets.SOURCE_RIGHT.getTargetingAngle() // close
    //           : Targets.PROCESSOR.getTargetingAngle() // far
    //       : (pose.getX()
    //               >= FieldConstants.fieldLength.in(Meters) - sourceCutoffDistance) // bottom half
    //           ? Targets.SOURCE_LEFT.getTargetingAngle() // close
    //           : Targets.NET.getTargetingAngle(); // far
    // } else {
    //   return (pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2) // blue
    //       ? (pose.getX() <= sourceCutoffDistance) // top half
    //           ? Targets.SOURCE_LEFT.getTargetingAngle() // close
    //           : Targets.NET.getTargetingAngle() // far
    //       : (pose.getX() <= sourceCutoffDistance) // bottom half
    //           ? Targets.SOURCE_RIGHT.getTargetingAngle() // close
    //           : Targets.PROCESSOR.getTargetingAngle(); // far
    // }

    if (isRedAlliance) {
      if (pose.getX() >= FieldConstants.fieldLength.in(Meters) - sourceCutoffDistance) { // close
        return (pose.getY() > FieldConstants.fieldWidth.in(Meters) / 2)
            ? Targets.SOURCE_RIGHT.getTargetingAngle()
            : Targets.SOURCE_LEFT.getTargetingAngle();
      } else
        return (pose.getY() < FieldConstants.fieldWidth.in(Meters) - 3)
            ? Targets.NET.getTargetingAngle()
            : Targets.PROCESSOR.getTargetingAngle();
    } else {
      if (pose.getX() <= sourceCutoffDistance) { // close
        return (pose.getY() < FieldConstants.fieldWidth.in(Meters) / 2)
            ? Targets.SOURCE_RIGHT.getTargetingAngle()
            : Targets.SOURCE_LEFT.getTargetingAngle();
      } else
        return (pose.getY() > 3)
            ? Targets.NET.getTargetingAngle()
            : Targets.PROCESSOR.getTargetingAngle();
    }
  }

  public static void setTargetSet(boolean isTargetSet) {
    targetSet = isTargetSet;
  }

  public static void randomizeTargetBranch() {
    randomBranch = random.nextInt(12);
    randomHeight = random.nextInt(3) + 1;
  }

  public static void checkBranchGame() {
    if (randomBranch == currentTargetBranch.gameID) {
      // && randomHeight == currentTargetLevel.gameHeight) {
      randomizeTargetBranch();
      branchGameScore++;
      while (randomBranch == currentTargetBranch.gameID) {
        randomizeTargetBranch();
      }
    }
  }

  public static void startBranchGame() {
    setTargetBranch(Targets.ALPHA);
    setTargetLevel(Levels.L1);
    randomBranch = random.nextInt(11) + 1;
    randomHeight = random.nextInt(3) + 1;
    branchGameScore = 0;
  }

  public static void setTargetingAlgae(boolean value) {
    targetingAlgae = value;
  }

  public static void setReadyToGrabAlgae(boolean value) {
    readyToGrabAlgae = value;
  }

  public static void setAligningWithAlgae(boolean value) {
    aligningWithAlgae = value;
  }

  public static void setGoForClimb(boolean value) {
    goForClimb = value;
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

  public static Levels getCurrentTargetLevelForBranchGame() {
    Levels gameTarget = Levels.L1;

    if (randomHeight == Levels.L1.gameHeight) gameTarget = Levels.L1;
    else if (randomHeight == Levels.L2.gameHeight) gameTarget = Levels.L2;
    else if (randomHeight == Levels.L3.gameHeight) gameTarget = Levels.L3;
    else if (randomHeight == Levels.L4.gameHeight) gameTarget = Levels.L4;

    return gameTarget;
  }

  public static Targets getTargetFromGameID(int gameID) {
    switch (gameID) {
      case -1:
        return Targets.LIMA;
      case 0:
        return Targets.ALPHA;
      case 1:
        return Targets.BRAVO;
      case 2:
        return Targets.CHARLIE;
      case 3:
        return Targets.DELTA;
      case 4:
        return Targets.ECHO;
      case 5:
        return Targets.FOXTROT;
      case 6:
        return Targets.GOLF;
      case 7:
        return Targets.HOTEL;
      case 8:
        return Targets.INDIA;
      case 9:
        return Targets.JULIET;
      case 10:
        return Targets.KILO;
      case 11:
        return Targets.LIMA;
      case 12:
        return Targets.ALPHA;
      default:
        return Targets.ALPHA;
    }
  }

  public static enum Targets {
    ALPHA(
        1,
        0,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? -Units.inchesToMeters(6 + homeYOffset) : Units.inchesToMeters(-yOffset)),
        Levels.ALGAE_HIGH),
    BRAVO(
        0,
        1,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? Units.inchesToMeters(6.75 + homeYOffset) : Units.inchesToMeters(yOffset)),
        Levels.ALGAE_HIGH),
    CHARLIE(
        1,
        2,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? -Units.inchesToMeters(6.5 + homeYOffset) : Units.inchesToMeters(-yOffset)),
        Levels.ALGAE_LOW),
    DELTA(
        0,
        3,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? Units.inchesToMeters(6.5 + homeYOffset) : Units.inchesToMeters(yOffset)),
        Levels.ALGAE_LOW),
    ECHO(
        1,
        4,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? -Units.inchesToMeters(6.75 + homeYOffset) : Units.inchesToMeters(-yOffset)),
        Levels.ALGAE_HIGH),
    FOXTROT(
        0,
        5,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? Units.inchesToMeters(6.25 + homeYOffset) : Units.inchesToMeters(yOffset)),
        Levels.ALGAE_HIGH),
    GOLF(
        1,
        6,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField
                ? -Units.inchesToMeters(5.875 + homeYOffset)
                : Units.inchesToMeters(-yOffset)),
        Levels.ALGAE_LOW),
    HOTEL(
        0,
        7,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? Units.inchesToMeters(6.75 + homeYOffset) : Units.inchesToMeters(yOffset)),
        Levels.ALGAE_LOW),
    INDIA(
        1,
        8,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? -Units.inchesToMeters(6.75 + homeYOffset) : Units.inchesToMeters(-yOffset)),
        Levels.ALGAE_HIGH),
    JULIET(
        0,
        9,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? Units.inchesToMeters(6.5 + homeYOffset) : Units.inchesToMeters(yOffset)),
        Levels.ALGAE_HIGH),
    KILO(
        1,
        10,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? -Units.inchesToMeters(6.5 + homeYOffset) : Units.inchesToMeters(-yOffset)),
        Levels.ALGAE_LOW),
    LIMA(
        0,
        11,
        new Translation2d(
            Units.inchesToMeters(xOffset),
            homeField ? Units.inchesToMeters(6.5 + homeYOffset) : Units.inchesToMeters(yOffset)),
        Levels.ALGAE_LOW),
    SOURCE_LEFT(0, 12, new Translation2d(Units.inchesToMeters(15.5), 0), Levels.INTAKE),
    SOURCE_RIGHT(0, 13, new Translation2d(Units.inchesToMeters(15.5), 0), Levels.INTAKE),
    PROCESSOR(0, 14, new Translation2d(), Levels.INTAKE),
    NET(0, 15, new Translation2d(1.75, Units.inchesToMeters(0)), Levels.L4);

    public final int preferredCamera;
    public final int gameID;
    public final Translation2d offset;
    public final Levels algaeLevel;

    Targets(int preferredCamera, int gameID, Translation2d offset, Levels algaeLevel) {
      this.preferredCamera = preferredCamera;
      this.gameID = gameID;
      this.offset = offset;
      this.algaeLevel = algaeLevel;
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

    public Levels getAlgaeLevel() {
      return algaeLevel;
    }

    public int gameID() {
      return gameID;
    }

    public Translation2d getOffset() {
      return aligningWithAlgae
          ? !readyToGrabAlgae ? primaryAlgaeOffset : secondaryAlgaeOffset
          : offset;
    }
  }

  public static enum Levels {
    INTAKE(0),
    L1(1),
    L2(2),
    L3(3),
    L4(4),
    ALGAE_HIGH(5),
    ALGAE_LOW(6);

    public final int gameHeight;

    Levels(int gameHeight) {
      this.gameHeight = gameHeight;
    }
  }
}
