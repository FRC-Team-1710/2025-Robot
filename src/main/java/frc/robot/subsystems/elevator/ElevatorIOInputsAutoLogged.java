package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderConnected", leaderConnected);
    table.put("LeaderPosition", leaderPosition);
    table.put("LeaderRoterPosition", leaderRotorPosition);
    table.put("LeaderVelocity", leaderVelocity);
    table.put("LeaderRoterVelocity", leaderRotorVelocity);
    table.put("LeaderStatorCurrent", leaderStatorCurrent);
    table.put("LeaderSupplyCurrent", leaderSupplyCurrent);
    table.put("FollowerConnected", followerConnected);
    table.put("FollowerPosition", followerPosition);
    table.put("FollowerRoterPosition", followerRotorPosition);
    table.put("FollowerVelocity", followerVelocity);
    table.put("FollowerRoterVelocity", followerRotorVelocity);
    table.put("FollowerStatorCurrent", followerStatorCurrent);
    table.put("FollowerSupplyCurrent", followerSupplyCurrent);
    table.put("AppliedVoltage", appliedVoltage);
    table.put("ElevatorDistance", elevatorDistance);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderConnected = table.get("LeaderConnected", leaderConnected);
    leaderPosition = table.get("LeaderPosition", leaderPosition);
    leaderRotorPosition = table.get("LeaderRoterPosition", leaderRotorPosition);
    leaderVelocity = table.get("LeaderVelocity", leaderVelocity);
    leaderRotorVelocity = table.get("LeaderRoterVelocity", leaderRotorVelocity);
    leaderStatorCurrent = table.get("LeaderStatorCurrent", leaderStatorCurrent);
    leaderSupplyCurrent = table.get("LeaderSupplyCurrent", leaderSupplyCurrent);
    followerConnected = table.get("FollowerConnected", followerConnected);
    followerPosition = table.get("FollowerPosition", followerPosition);
    followerRotorPosition = table.get("FollowerRoterPosition", followerRotorPosition);
    followerVelocity = table.get("FollowerVelocity", followerVelocity);
    followerRotorVelocity = table.get("FollowerRoterVelocity", followerRotorVelocity);
    followerStatorCurrent = table.get("FollowerStatorCurrent", followerStatorCurrent);
    followerSupplyCurrent = table.get("FollowerSupplyCurrent", followerSupplyCurrent);
    appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
    elevatorDistance = table.get("ElevatorDistance", elevatorDistance);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.leaderConnected = this.leaderConnected;
    copy.leaderPosition = this.leaderPosition;
    copy.leaderRotorPosition = this.leaderRotorPosition;
    copy.leaderVelocity = this.leaderVelocity;
    copy.leaderRotorVelocity = this.leaderRotorVelocity;
    copy.leaderStatorCurrent = this.leaderStatorCurrent;
    copy.leaderSupplyCurrent = this.leaderSupplyCurrent;
    copy.followerConnected = this.followerConnected;
    copy.followerPosition = this.followerPosition;
    copy.followerRotorPosition = this.followerRotorPosition;
    copy.followerVelocity = this.followerVelocity;
    copy.followerRotorVelocity = this.followerRotorVelocity;
    copy.followerStatorCurrent = this.followerStatorCurrent;
    copy.followerSupplyCurrent = this.followerSupplyCurrent;
    copy.appliedVoltage = this.appliedVoltage;
    copy.elevatorDistance = this.elevatorDistance;
    return copy;
  }
}
