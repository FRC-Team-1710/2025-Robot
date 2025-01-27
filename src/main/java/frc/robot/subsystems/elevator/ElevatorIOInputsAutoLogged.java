package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderConnected", leaderConnected);
    table.put("FollowerConnected", followerConnected);
    table.put("ManualSpin", manualSpin);
    table.put("Setpoint", setpoint);
    table.put("SIMsetpoint", SIMsetpoint);
    table.put("LeaderPosition", leaderPosition);
    table.put("Distance", distance);
    table.put("LeaderVelocity", leaderVelocity);
    table.put("AppliedVoltage", leaderAppliedVoltage);
    table.put("LeaderStatorCurrent", leaderStatorCurrent);
    table.put("FollowerStatorCurrent", followerStatorCurrent);
    table.put("LeaderSupplyCurrent", leaderSupplyCurrent);
    table.put("FollowerSupplyCurrent", followerSupplyCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderConnected = table.get("LeaderConnected", leaderConnected);
    followerConnected = table.get("FollowerConnected", followerConnected);
    manualSpin = table.get("ManualSpin", manualSpin);
    setpoint = table.get("Setpoint", setpoint);
    SIMsetpoint = table.get("SIMsetpoint", SIMsetpoint);
    leaderPosition = table.get("LeaderPosition", leaderPosition);
    distance = table.get("Distance", distance);
    leaderVelocity = table.get("LeaderVelocity", leaderVelocity);
    leaderAppliedVoltage = table.get("AppliedVoltage", leaderAppliedVoltage);
    leaderStatorCurrent = table.get("LeaderStatorCurrent", leaderStatorCurrent);
    followerStatorCurrent = table.get("FollowerStatorCurrent", followerStatorCurrent);
    leaderSupplyCurrent = table.get("LeaderSupplyCurrent", leaderSupplyCurrent);
    followerSupplyCurrent = table.get("FollowerSupplyCurrent", followerSupplyCurrent);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.leaderConnected = this.leaderConnected;
    copy.followerConnected = this.followerConnected;
    copy.manualSpin = this.manualSpin;
    copy.setpoint = this.setpoint;
    copy.SIMsetpoint = this.SIMsetpoint;
    copy.leaderPosition = this.leaderPosition;
    copy.distance = this.distance;
    copy.leaderVelocity = this.leaderVelocity;
    copy.leaderAppliedVoltage = this.leaderAppliedVoltage;
    copy.leaderStatorCurrent = this.leaderStatorCurrent;
    copy.followerStatorCurrent = this.followerStatorCurrent;
    copy.leaderSupplyCurrent = this.leaderSupplyCurrent;
    copy.followerSupplyCurrent = this.followerSupplyCurrent;
    return copy;
  }
}
