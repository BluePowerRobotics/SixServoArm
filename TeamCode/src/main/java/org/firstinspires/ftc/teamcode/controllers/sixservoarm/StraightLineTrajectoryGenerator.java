package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import org.firstinspires.ftc.teamcode.utility.Point3D;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 三维直线轨迹生成器与 time-scaling 工具。
 *
 * <p>核心流程：
 * <ol>
 *   <li>调用 {@link #generateLine(Point3D, Point3D, double, double, double, int)}
 *       将 3D 直线离散化为一系列 {@link TrajectoryPoint}（每点含 6 个舵机的角度，单位度）</li>
 *   <li>调用 {@link #timeScaleByServoSpeed(List, double, double)}
 *       基于最慢舵机能力为每个点分配时间戳，得到 {@link TimedTrajectoryPoint} 列表</li>
 *   <li>（可选）通过 {@link #toCSV(List)} / {@link #fromCSV(String)} 进行持久化</li>
 * </ol>
 *
 * <p>伺服索引约定（与 SixServoArmOutputter 一致）：
 * <ul>
 *   <li>0–3：由 {@link SixServoArmCalculator#calculateToServo(Point3D, double)} 解算的四个关节角</li>
 *   <li>4：夹爪绕第三节臂的旋转角（radianAroundArm3）</li>
 *   <li>5：夹爪开合角</li>
 * </ul>
 */
public class StraightLineTrajectoryGenerator {

    // ========================================================================
    //  数据类
    // ========================================================================

    /** 轨迹采样点：存储 6 个舵机的目标角度（度）。 */
    public static class TrajectoryPoint {
        /** 长度固定为 6：索引 0–3 为关节角，4 为第三节旋转，5 为夹爪。 */
        public final double[] jointDegrees;

        public TrajectoryPoint(double[] jointDegrees) {
            if (jointDegrees == null || jointDegrees.length != 6) {
                throw new IllegalArgumentException("jointDegrees must be length 6");
            }
            this.jointDegrees = jointDegrees.clone();
        }
    }

    /** 带时间戳的轨迹点：用于 time-scaling 后的播放。 */
    public static class TimedTrajectoryPoint {
        /** 从轨迹开始算起的累计时间（秒）。 */
        public final double timeSeconds;
        /** 6 个舵机的目标角度（度）。 */
        public final double[] jointDegrees;

        public TimedTrajectoryPoint(double timeSeconds, double[] jointDegrees) {
            if (jointDegrees == null || jointDegrees.length != 6) {
                throw new IllegalArgumentException("jointDegrees must be length 6");
            }
            this.timeSeconds = timeSeconds;
            this.jointDegrees = jointDegrees.clone();
        }
    }

    // ========================================================================
    //  直线生成
    // ========================================================================

    /**
     * 在 3D 空间中生成一条直线轨迹。
     *
     * @param start              起点（mm 坐标系，与机械臂 IK 一致）
     * @param end                终点
     * @param clipHeadingRadian  夹爪相对于水平面的倾角（弧度），传给 IK
     * @param radianAroundArm3   夹爪绕第三节臂的旋转角（弧度），全部采样点共用
     * @param clipOpenDegrees    夹爪开合的目标角度（度），全部采样点共用
     * @param numSamples         采样点数量（含首尾，≥2）
     * @return 轨迹点列表，长度 = numSamples
     */
    public static List<TrajectoryPoint> generateLine(
            Point3D start,
            Point3D end,
            double clipHeadingRadian,
            double radianAroundArm3,
            double clipOpenDegrees,
            int numSamples) {

        if (numSamples < 2) {
            throw new IllegalArgumentException("numSamples must be >= 2");
        }

        SixServoArmCalculator calculator = new SixServoArmCalculator();
        List<TrajectoryPoint> points = new ArrayList<>(numSamples);

        for (int i = 0; i < numSamples; i++) {
            double t = (numSamples == 1) ? 0.0 : (double) i / (numSamples - 1);
            double x = start.getX() + t * (end.getX() - start.getX());
            double y = start.getY() + t * (end.getY() - start.getY());
            double z = start.getZ() + t * (end.getZ() - start.getZ());
            Point3D pt = new Point3D(x, y, z);

            // IK 解算：返回 servos 0–3 的弧度值
            double[] radians = calculator.calculateToServo(pt, clipHeadingRadian);

            double[] degrees = new double[6];
            for (int j = 0; j < 4; j++) {
                degrees[j] = Math.toDegrees(radians[j]);
            }
            degrees[4] = Math.toDegrees(radianAroundArm3);
            degrees[5] = clipOpenDegrees;

            points.add(new TrajectoryPoint(degrees));
        }
        return points;
    }

    /**
     * 简化版：使用默认夹爪参数（radianAroundArm3=0, 夹爪角度=SixServoArmOutputter.toDegree(5, ClipOpenPosition)）。
     *
     * @see #generateLine(Point3D, Point3D, double, double, double, int)
     */
    public static List<TrajectoryPoint> generateLine(
            Point3D start, Point3D end, double clipHeadingRadian, int numSamples) {
        return generateLine(start, end, clipHeadingRadian, 0.0,
                SixServoArmOutputter.toDegree(5, SixServoArmOutputter.ClipOpenPosition),
                numSamples);
    }

    // ========================================================================
    //  Time-Scaling
    // ========================================================================

    /**
     * 基于各舵机的最大速度对轨迹点做 time-scaling。
     *
     * <p>算法：
     * <ol>
     *   <li>从 {@link SixServoArmState#getServoSpeed()} 读取各舵机名义速度
     *       vmaxDegPerSec[j] = 60.0 / servoSpeed[j]</li>
     *   <li>对每对相邻采样点，计算 t_seg = max_j(|Δθ_j| / vmax_j)</li>
     *   <li>累计得到每个点的时间戳 times[i]</li>
     * </ol>
     *
     * <p><b>静摩擦与回差处理（两种方法）：</b>
     *
     * <p><b>方法1 — minStepDeg：</b>若一段内所有关节的 |Δθ| 均小于 minStepDeg，
     * 则将该段的时间计为 0（相当于跳过该中间点）。这避免了对过小增量发送无效指令。
     *
     * <p><b>方法2 — wakeupDeg：</b>当某个关节的运动方向发生反转时，在其有效 |Δθ| 上
     * 额外增加 wakeupDeg，以补偿齿轮回差（backlash）。不影响后续点的实际目标角度，
     * 只增大该段时间以给舵机更多时间克服静摩擦。
     *
     * @param pts         未标时的轨迹点列表（来自 {@link #generateLine}）
     * @param minStepDeg  最小步长阈值（度），≤0 表示不启用方法1
     * @param wakeupDeg   方向反转唤醒偏置（度），≤0 表示不启用方法2
     * @return 带时间戳的轨迹点列表，长度 ≤ pts.size()
     */
    public static List<TimedTrajectoryPoint> timeScaleByServoSpeed(
            List<TrajectoryPoint> pts, double minStepDeg, double wakeupDeg) {

        if (pts == null || pts.isEmpty()) {
            throw new IllegalArgumentException("pts must be non-null and non-empty");
        }

        // 获取各舵机最大速度（度/秒）
        double[] servoSpeedArr = SixServoArmState.getInstance().getServoSpeed();
        double[] vmaxDegPerSec = new double[6];
        for (int j = 0; j < 6; j++) {
            vmaxDegPerSec[j] = 60.0 / servoSpeedArr[j];
        }

        // 记录上一段的运动方向（用于 wakeupDeg 检测）
        int[] prevDirection = new int[6]; // +1 正转, -1 反转, 0 静止

        List<TimedTrajectoryPoint> timedPts = new ArrayList<>();
        timedPts.add(new TimedTrajectoryPoint(0.0, pts.get(0).jointDegrees));
        double currentTime = 0.0;

        for (int i = 1; i < pts.size(); i++) {
            double[] prevDeg = pts.get(i - 1).jointDegrees;
            double[] currDeg = pts.get(i).jointDegrees;

            // ---- 方法1：minStepDeg 检测 ----
            boolean allBelowThreshold = true;
            for (int j = 0; j < 6; j++) {
                if (Math.abs(currDeg[j] - prevDeg[j]) >= minStepDeg) {
                    allBelowThreshold = false;
                    break;
                }
            }
            if (minStepDeg > 0 && allBelowThreshold) {
                // 跳过该中间点：不增加时间，也不添加到输出（除非是最后一个点）
                if (i == pts.size() - 1) {
                    // 最后一个点必须保留（保证轨迹终点正确）
                    timedPts.add(new TimedTrajectoryPoint(currentTime, currDeg));
                }
                continue;
            }

            // ---- 方法2：wakeupDeg 检测 ----
            double maxSegTime = 0.0;
            for (int j = 0; j < 6; j++) {
                double delta = currDeg[j] - prevDeg[j];
                double absDelta = Math.abs(delta);
                double effectiveDelta = absDelta;

                int curDir = (delta > 1e-9) ? 1 : ((delta < -1e-9) ? -1 : 0);

                if (wakeupDeg > 0 && i > 1 && prevDirection[j] != 0 && curDir != 0
                        && curDir != prevDirection[j]) {
                    // 方向反转：增加唤醒偏置
                    effectiveDelta += wakeupDeg;
                }

                double segTime = effectiveDelta / vmaxDegPerSec[j];
                if (segTime > maxSegTime) {
                    maxSegTime = segTime;
                }
                prevDirection[j] = curDir;
            }

            currentTime += maxSegTime;
            timedPts.add(new TimedTrajectoryPoint(currentTime, currDeg));
        }

        return timedPts;
    }

    /**
     * 简化版 time-scaling：同时启用 minStepDeg 和 wakeupDeg。
     */
    public static List<TimedTrajectoryPoint> timeScaleByServoSpeed(List<TrajectoryPoint> pts) {
        return timeScaleByServoSpeed(pts, 0.5, 2.0);
    }

    // ========================================================================
    //  CSV 导入/导出
    // ========================================================================

    /** CSV 表头。 */
    public static final String CSV_HEADER = "time_s,theta0_deg,theta1_deg,theta2_deg,theta3_deg,theta4_deg,theta5_deg";

    /**
     * 将带时间戳的轨迹导出为 CSV 字符串。
     *
     * @param timedPts 轨迹点列表
     * @return CSV 格式字符串（含表头行）
     */
    public static String toCSV(List<TimedTrajectoryPoint> timedPts) {
        StringBuilder sb = new StringBuilder();
        sb.append(CSV_HEADER).append('\n');
        for (TimedTrajectoryPoint tp : timedPts) {
            sb.append(String.format(Locale.US, "%.4f", tp.timeSeconds));
            for (int j = 0; j < 6; j++) {
                sb.append(',').append(String.format(Locale.US, "%.4f", tp.jointDegrees[j]));
            }
            sb.append('\n');
        }
        return sb.toString();
    }

    /**
     * 将 CSV 字符串保存到文件。
     *
     * @param timedPts 轨迹点列表
     * @param filePath 目标文件路径（例如 /sdcard/FIRST/trajectory.csv）
     * @throws IOException 如果写入失败
     */
    public static void saveCSV(List<TimedTrajectoryPoint> timedPts, String filePath) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
            writer.write(toCSV(timedPts));
        }
    }

    /**
     * 从 CSV 文件加载轨迹。
     *
     * @param filePath 文件路径
     * @return 解析后的 TimedTrajectoryPoint 列表
     * @throws IOException 如果读取或解析失败
     */
    public static List<TimedTrajectoryPoint> loadCSV(String filePath) throws IOException {
        List<TimedTrajectoryPoint> result = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String header = reader.readLine(); // 跳过表头
            if (header == null || !header.startsWith("time_s")) {
                throw new IOException("Invalid CSV header: " + header);
            }
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;
                String[] parts = line.split(",");
                if (parts.length != 7) {
                    throw new IOException("Invalid CSV line (expected 7 columns): " + line);
                }
                double time = Double.parseDouble(parts[0].trim());
                double[] degs = new double[6];
                for (int j = 0; j < 6; j++) {
                    degs[j] = Double.parseDouble(parts[j + 1].trim());
                }
                result.add(new TimedTrajectoryPoint(time, degs));
            }
        }
        return result;
    }

    /**
     * 从 CSV 字符串解析轨迹。
     *
     * @param csvContent CSV 格式字符串
     * @return 解析后的轨迹
     */
    public static List<TimedTrajectoryPoint> fromCSV(String csvContent) {
        List<TimedTrajectoryPoint> result = new ArrayList<>();
        String[] lines = csvContent.split("\\n");
        for (int i = 1; i < lines.length; i++) {  // 跳过表头
            String line = lines[i].trim();
            if (line.isEmpty()) continue;
            String[] parts = line.split(",");
            if (parts.length != 7) continue;
            double time = Double.parseDouble(parts[0].trim());
            double[] degs = new double[6];
            for (int j = 0; j < 6; j++) {
                degs[j] = Double.parseDouble(parts[j + 1].trim());
            }
            result.add(new TimedTrajectoryPoint(time, degs));
        }
        return result;
    }
}
