package frc.robot.util;

/**
 * Encapsulate boolean value, but return {@code true} if a true value was saved
 * in the recent past.
 */
public class DebouncedBoolean {

    private final long debounceMilliseconds;
    private long previousTrueTime;
    private boolean recentValue;

    public DebouncedBoolean(long milliseconds) {
        assert milliseconds >= 0 : "Debounce duration must be non-negative.";
        this.debounceMilliseconds = milliseconds;
    }

    public synchronized void set(boolean b) {
        recentValue = b;
        if (b) {
            previousTrueTime = System.currentTimeMillis();
        }
    }

    public synchronized boolean get() {
        final long recentTrueTime = previousTrueTime + debounceMilliseconds;
        return recentValue || System.currentTimeMillis() < recentTrueTime;
    }
}
