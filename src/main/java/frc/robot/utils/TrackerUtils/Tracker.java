package frc.robot.utils.TrackerUtils;


import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Arrays;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Tracker is a lightweight, static logging utility intended for debugging.
 *
 * <p>
 * Each log entry records:
 * <ul>
 *   <li>a user-supplied timestamp (double)</li>
 *   <li>the calling class and method</li>
 *   <li>arbitrary, pre-formatted input strings</li>
 * </ul>
 *
 * <p><b>Basic usage (explicit logging):</b>
 *
 * <pre>{@code
 * Tracker.log(
 *     timer.get(),
 *     "DriveSubsystem",
 *     "update",
 *     "speed=" + speed,
 *     "angle=" + angle
 * );
 * }</pre>
 *
 * <p><b>Auto-detected logging (stack trace):</b>
 *
 * <pre>{@code
 * Tracker.log(
 *     timer.get(),
 *     "speed=" + speed,
 *     "angle=" + angle
 * );
 * }</pre>
 *
 * <p><b>Method wrapper (returns value):</b>
 *
 * <pre>{@code
 * int pathId = Tracker.track(
 *     timer.get(),
 *     () -> computePath(a, b),
 *     "a=" + a,
 *     "b=" + b
 * );
 * }</pre>
 *
 * <p><b>Method wrapper (void):</b>
 *
 * <pre>{@code
 * Tracker.track(
 *     timer.get(),
 *     () -> motor.set(speed),
 *     "speed=" + speed
 * );
 * }</pre>
 *
 * <p>
 * Log output format (plain text):
 *
 * <pre>
 * [1234.567890] frc.robot.DriveSubsystem::update inputs=[speed=0.5, angle=90]
 * </pre>
 *
 * <p>
 * This class is intentionally minimal:
 * it does not manage concurrency, log levels, or file rotation.
 */

public final class Tracker {

    /**
     * Fundamental data storage for the logging.
     */
    private record DataPoint(
            double timestamp,
            String parentFile,
            String methodName,
            String[] inputs
    ) {
        String formatPlainText() {
            return String.format(
                    "[%.6f] %s::%s inputs=%s",
                    timestamp,
                    parentFile,
                    methodName,
                    Arrays.toString(inputs)
            );
        }
    }

    // ===== File management =====

    private static final BufferedWriter writer;

    static {
        try {
            String fileName =
                    "tracker-" +
                    LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss")) +
                    ".log";

            Path logPath = Path.of(fileName);
            writer = Files.newBufferedWriter(
                    logPath,
                    StandardOpenOption.CREATE_NEW,
                    StandardOpenOption.WRITE
            );

            writer.write("=== Tracker Log Started ===");
            writer.newLine();
            writer.flush();

        } catch (IOException e) {
            throw new ExceptionInInitializerError(e);
        }
    }

    private Tracker() {
        // Prevent instantiation
    }

    // ===== Core logging =====

    public static void log(
            double timestamp,
            String parentFile,
            String methodName,
            String... inputs
    ) {
        Objects.requireNonNull(parentFile);
        Objects.requireNonNull(methodName);
        Objects.requireNonNull(inputs);

        DataPoint dp = new DataPoint(timestamp, parentFile, methodName, inputs);
        write(dp);
    }

    /**
     * Log using stack-trace auto-detection.
     */
    public static void log(
            double timestamp,
            String... inputs
    ) {
        StackTraceElement caller = findCaller();
        log(
                timestamp,
                caller.getClassName(),
                caller.getMethodName(),
                inputs
        );
    }

    private static void write(DataPoint dp) {
        try {
            writer.write(dp.formatPlainText());
            writer.newLine();
            writer.flush();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    // ===== Method wrapper (monad-style) =====

    public static <T> T track(
            double timestamp,
            Supplier<T> action,
            String... inputs
    ) {
        StackTraceElement caller = findCaller();

        log(
                timestamp,
                caller.getClassName(),
                caller.getMethodName(),
                inputs
        );

        return action.get();
    }

    public static void track(
            double timestamp,
            Runnable action,
            String... inputs
    ) {
        StackTraceElement caller = findCaller();

        log(
                timestamp,
                caller.getClassName(),
                caller.getMethodName(),
                inputs
        );

        action.run();
    }

    // ===== Stack trace logic =====

    private static StackTraceElement findCaller() {
        StackTraceElement[] stack = Thread.currentThread().getStackTrace();

        // Stack layout (typical):
        // 0 getStackTrace
        // 1 findCaller
        // 2 log / track
        // 3 actual caller
        for (int i = 0; i < stack.length; i++) {
            if (!stack[i].getClassName().equals(Tracker.class.getName())
                    && !stack[i].getClassName().equals(Thread.class.getName())) {
                return stack[i];
            }
        }

        // Fallback (should never happen)
        return stack[stack.length - 1];
    }
}
