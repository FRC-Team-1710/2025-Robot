/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A linear path for a holonomic drivetrain (i.e. swerve or mechanum).
 * This generates profiled setpoints for the robot along a straight line
 * based on trapezoidal velocity constraints (using {@link TrapezoidProfile}).
 * <p>
 * Initialization:
 * <pre><code>
 * final LinearPath path = new LinearPath(
 *     new TrapezoidProfile.Constraints(kMaxV, kMaxA),
 *     new TrapezoidProfile.Constraints(kMaxOmega, kMaxAlpha)
 * );
 * LinearPath.State current = new LinearPath.State(
 *     initialPose, initialFieldSpeeds
 * );
 * </code></pre>
 * <p>
 * Run on update:
 * <pre><code>
 * current = path.calculate(timeSincePreviousUpdate, current, targetPose);
 * </code></pre>
 */
public class LinearPath {
    /** Path state. */
    public static class State {
        /** The pose at this state. */
        public Pose2d pose;
        /** The field-centric speeds at this state. */
        public ChassisSpeeds speeds;

        /** Constructs a new path state with zero'd pose and speeds. */
        public State() {
            this(Pose2d.kZero, new ChassisSpeeds());
        }
        /**
         * Constructs a new path state.
         *
         * @param pose The pose at this state
         * @param speeds THe field-centric speeds at this state
         */
        public State(Pose2d pose, ChassisSpeeds speeds) {
            this.pose = pose;
            this.speeds = speeds;
        }
    }

    private final TrapezoidProfile linearProfile;
    private final TrapezoidProfile angularProfile;

    private Pose2d initialPose = Pose2d.kZero;
    private Rotation2d heading = Rotation2d.kZero;

    private TrapezoidProfile.State linearGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State linearStart = new TrapezoidProfile.State();

    private TrapezoidProfile.State angularGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State angularStart = new TrapezoidProfile.State();

    /**
     * Constructs a linear path.
     *
     * @param linear The constraints on the profile linear motion
     * @param angular The constraints on the profile angular motion
     */
    public LinearPath(TrapezoidProfile.Constraints linear, TrapezoidProfile.Constraints angular) {
        this.linearProfile = new TrapezoidProfile(linear);
        this.angularProfile = new TrapezoidProfile(angular);
    }

    /**
     * Calculates the component of the velocity in the direction of travel.
     *
     * @param speeds The field-centric chassis speeds
     * @param heading The heading of the direction of travel
     * @return Component of velocity in the direction of the heading
     */
    private static double calculateVelocityAtHeading(ChassisSpeeds speeds, Rotation2d heading) {
        // vel = <vx, vy> ⋅ <cos(heading), sin(heading)>
        // vel = vx * cos(heading) + vy * sin(heading)
        return speeds.vxMetersPerSecond * heading.getCos() + speeds.vyMetersPerSecond * heading.getSin();
    }

    /**
     * Sets the current and goal states of the linear path.
     *
     * @param current The current state
     * @param goal The desired pose when the path is complete
     */
    private void setState(State current, Pose2d goal) {
        this.initialPose = current.pose;

        {
            // pull out the translation from our initial pose to the target
            final var translation = goal.getTranslation().minus(this.initialPose.getTranslation());
            // pull out distance and heading to the target
            final var distance = translation.getNorm();
            if (distance > 1e-6) {
                this.heading = translation.getAngle();
            } else {
                this.heading = Rotation2d.kZero;
            }

            this.linearGoal = new TrapezoidProfile.State(distance, 0);
        }

        {
            // start at current velocity in the direction of travel
            final var vel = calculateVelocityAtHeading(current.speeds, this.heading);
            this.linearStart = new TrapezoidProfile.State(0, vel);
        }

        this.angularStart = new TrapezoidProfile.State(
            current.pose.getRotation().getRadians(),
            current.speeds.omegaRadiansPerSecond
        );
        // wrap the angular goal so we take the shortest path
        this.angularGoal = new TrapezoidProfile.State(
            current.pose.getRotation().getRadians() +
                goal.getRotation().minus(current.pose.getRotation()).getRadians(),
            0
        );
    }

    /**
     * Calculates the pose and speeds of the path at a time t where
     * the current state is at t = 0.
     *
     * @param t How long to advance from the current state to the desired state
     * @return The pose and speeds of the profile at time t
     */
    private State calculate(double t) {
        // calculate our new distance and velocity in the desired direction of travel
        final var linearState = this.linearProfile.calculate(t, this.linearStart, this.linearGoal);
        // calculate our new heading and rotational rate
        final var angularState = this.angularProfile.calculate(t, this.angularStart, this.angularGoal);

        // x is m_state * cos(heading), y is m_state * sin(heading)
        final var pose = new Pose2d(
            this.initialPose.getX() + linearState.position * this.heading.getCos(),
            this.initialPose.getY() + linearState.position * this.heading.getSin(),
            Rotation2d.fromRadians(angularState.position)
        );
        final var speeds = new ChassisSpeeds(
            linearState.velocity * this.heading.getCos(),
            linearState.velocity * this.heading.getSin(),
            angularState.velocity
        );
        return new State(pose, speeds);
    }

    /**
     * Calculates the pose and speeds of the path at a time t where
     * the current state is at t = 0.
     *
     * @param t How long to advance from the current state to the desired state
     * @param current The current state
     * @param goal The desired pose when the path is complete
     * @return The pose and speeds of the profile at time t
     */
    public final State calculate(double t, State current, Pose2d goal) {
        setState(current, goal);
        return calculate(t);
    }

    /**
     * Returns the total time the profile takes to reach the goal.
     *
     * @return The total time the profile takes to reach the goal
     */
    public final double totalTime() {
        return Math.max(
            this.linearProfile.totalTime(),
            this.angularProfile.totalTime()
        );
    }

    /**
     * Returns true if the profile has reached the goal.
     * <p>
     * The profile has reached the goal if the time since the profile
     * started has exceeded the profile's total time.
     *
     * @param t The time since the beginning of the profile
     * @return true if the profile has reached the goal
     */
    public final boolean isFinished(double t) {
        return t >= totalTime();
    }
}
