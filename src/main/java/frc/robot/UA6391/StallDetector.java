package frc.robot.UA6391;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.Constants.IntakeConstants;

public class StallDetector {
    private int minStallMillis;
    private long stallMillis;
    private long lastRunMillis;
    private StallDetectorStatus stallStatus = new StallDetectorStatus();
    private final MedianFilter currentFilter = new MedianFilter(40); // enough to keep 1 second of data when called every 25ms
    private final int slot;
    private final PowerDistributionPanel pdp; 

    public StallDetector(int slot) {
        this.slot = slot;
        this.pdp = new PowerDistributionPanel(0);
    }

    public void setMinStallMillis(int minStallMillis) {
        this.minStallMillis = minStallMillis;
    }

    public double updateStallStatus() {
        double currentCurrent = currentFilter.calculate(pdp.getCurrent(slot));
        long nowMillis = System.currentTimeMillis();
        long elapsed = nowMillis - lastRunMillis;

        if (currentCurrent >= IntakeConstants.kIntakeStallCurrent) {
            stallMillis += elapsed;
        } else {
            stallMillis -= elapsed;
        }

        lastRunMillis = System.currentTimeMillis();

        stallStatus.isStalled = stallMillis >= minStallMillis;
        stallStatus.stalledForMillis = (int) stallMillis;
        return currentCurrent;
    }

    public StallDetectorStatus getStallStatus() {
        return stallStatus;
    }
}
