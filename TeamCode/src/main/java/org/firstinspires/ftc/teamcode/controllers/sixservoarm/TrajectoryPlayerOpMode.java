package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.utility.LinearInterpolation.LinearInterpolation;
import org.firstinspires.ftc.teamcode.utility.Point3D;

import java.util.List;

/**
 * 轨迹播放 OpMode：非阻塞式状态机架构，运行时可持续通过 Dashboard 或手柄调参。
 *
 * <h3>⚠️ 关键限制：全开环控制，无伺服反馈</h3>
 * <p>本系统使用的舵机（servo）没有位置传感器。<b>以下所有"当前位置"均为软件推算值，
 * 基于 servoSpeed 和时间积分，不代表舵机真实物理位置：</b></p>
 * <ul>
 *   <li>{@code SixServoArmState.getServoNowDegree()} — 软件推算，非实测</li>
 *   <li>{@code SixServoArmState.ifFinishedMoving()} — 模型判断时间到，非真实到达</li>
 *   <li>轨迹播放全程开环：下发 setPosition() 后无任何确认</li>
 * </ul>
 * <p><b>缓解措施：</b>回 Home 复位校准 + 肉眼观察确认 + 超时保护。</p>
 *
 * <h3>运行方式</h3>
 * <ol>
 *   <li>在 FTC Dashboard 中调整参数（controlFreq、lowSpeedFactor 等，支持运行时实时修改）</li>
 *   <li><b>A</b>：执行"回 home"校准复位</li>
 *   <li><b>B</b>：从配置起终点生成直线轨迹并开始播放</li>
 *   <li><b>Y</b>：切换低速模式（lowSpeedFactor=0.5 ↔ 1.0，播放中也可切换）</li>
 * </ol>
 *
 * <h3>架构特点</h3>
 * <ul>
 *   <li>单主循环 + 状态机（IDLE / PLAYING），不阻塞</li>
 *   <li>播放中仍可调参（Dashboard 或 Y 键），参数下一周期立即生效</li>
 *   <li>Telemetry 每周期刷新，实时显示插值结果</li>
 * </ul>
 */
@TeleOp(name = "Trajectory Player", group = "TEST")
@Config
public class TrajectoryPlayerOpMode extends LinearOpMode {

    // ========================================================================
    //  可配置参数（通过 FTC Dashboard 实时调整）
    // ========================================================================

    //TODO check FRS
    public static double controlFreq   = 100;
    //通过“慢放时间”实现减慢速度，不会影响插值表中时刻的对应。
    //e.g. 1.0s的轨迹，lowSpeedFactor=0.5 时实际播放需要 2.0s。原本0.5s的位置在低速模式下会在1.0s时刻播放出来。
    public static double lowSpeedFactor = 0.5;

    //TODO Tune this
    public static double minStepDeg    = 0.5;
    public static double wakeupDeg     = 0.0;

    /** 超时因子：实际耗时 > 预期耗时 × timeoutFactor 则强制停止（0=禁用超时检测）。 */
    public static double timeoutFactor = 0;

    public static double homeX = 0;
    public static double homeY = 100;
    public static double homeZ = 30.0;

    public static double clipHeadingRadian = -Math.PI / 2;
    public static double RadianAroundArm3 = 0;

    public static double startX = 0;
    public static double startY = 150;
    public static double startZ = 100;
    public static double endX   = 0;
    public static double endY   = 50.0;
    public static double endZ   = 100;

    //TODO tune this
    public static int numSamples = 50;

    // ========================================================================
    //  状态机
    // ========================================================================

    private enum Mode { IDLE, MOVING_TO_START, PLAYING }
    private Mode mode = Mode.IDLE;

    // ========================================================================
    //  持久硬件引用
    // ========================================================================

    private SixServoArmController armController;
    private SixServoArmOutputter outputter;

    private final double[] lastSentPositions = new double[6];
    private final double[] lastTargetAngles  = new double[6];

    // ========================================================================
    //  轨迹播放状态（PLAYING 期间使用，跨周期保持）
    // ========================================================================

    private List<StraightLineTrajectoryGenerator.TimedTrajectoryPoint> activeTraj;
    private double[] trajTimes;          // n
    private double[][] trajJointDegs;    // [6][n]
    private LinearInterpolation[] trajInterpolators; // [6] — 每关节一个，一次性构建
    private int    trajN;
    private double trajTotalDuration;
    private long   trajStartNano;
    private int    trajCurrentSegment;   // 仅用于 telemetry 显示，插值由 interpolator 内部处理
    private long   trajLoopCount;
    private long   trajTotalSendDelayNs;
    private long   trajMaxSendDelayNs;

    // ========================================================================
    //  OpMode 生命周期
    // ========================================================================

    @Override
    public void runOpMode() {
        // ---- 初始化 ----
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        InstanceTelemetry.init(telemetry);
        armController = new SixServoArmController(hardwareMap);
        outputter = armController.getOutputter();

        telemetry.addLine("Trajectory Player Ready");
        telemetry.addLine("A: Home | B: Run Line | Y: Toggle Speed");
        telemetry.addData("lowSpeedFactor", "%.2f", lowSpeedFactor);
        telemetry.addData("controlFreq (Hz)", "%.1f", controlFreq);
        telemetry.update();

        waitForStart();

        // ================================================================
        //  唯一主循环（非阻塞，所有逻辑在此驱动）
        // ================================================================
        while (opModeIsActive()) {

            // ----- 通用操作（IDLE 和 PLAYING 状态均可响应） -----
            handleCommonInputs();

            // ----- 状态分发 -----
            switch (mode) {
                case IDLE:
                    tickIdle();
                    break;
                case MOVING_TO_START:
                    tickMovingToStart();
                    break;
                case PLAYING:
                    tickPlaying();
                    break;
            }

            // ----- 每周期统一刷新 telemetry -----
            telemetry.update();

            // ----- 控制主循环频率 -----
            sleepForCycle();
        }
    }

    // ========================================================================
    //  通用输入处理
    // ========================================================================

    private void handleCommonInputs() {
        // A — 回 Home
        if (gamepad1.a) {
            goHome();
            //sleepSafe(400);
        }

        // Y — 切换低速/全速（播放中亦可切换）
        if (gamepad1.y) {
            lowSpeedFactor = (Math.abs(lowSpeedFactor - 0.5) < 0.01) ? 1.0 : 0.5;
            //sleepSafe(200);
        }
    }

    // ========================================================================
    //  IDLE 状态
    // ========================================================================

    private void tickIdle() {
        // B — 生成轨迹并运动到起始点
        if (gamepad1.b) {
            generateAndStart();
            return;
        }

        // 空闲状态 telemetry
        telemetry.addData("State", "IDLE");
        telemetry.addData("lowSpeedFactor", "%.2f", lowSpeedFactor);
        telemetry.addData("A=Home", "B=Start  Y=Speed");
        telemetry.addData("Line", "(%.0f,%.0f,%.0f)→(%.0f,%.0f,%.0f)",
                startX, startY, startZ, endX, endY, endZ);
        telemetry.addData("RadianAroundArm3", "%.2f", RadianAroundArm3);
    }

    // ========================================================================
    //  MOVING_TO_START 状态 — 等待用户确认臂已到达起点后手动开始
    // ========================================================================

    private void tickMovingToStart() {
        // X — 确认到位，开始播放
        if (gamepad1.x) {
            trajStartNano = System.nanoTime();
            telemetry.addLine("Playing...");
            mode = Mode.PLAYING;
            sleepSafe(200);
            return;
        }

        // ⚠️ 以下角度均为软件推算值（开环估计），非传感器实测。
        // 舵机无位置反馈，实际位置可能因负载/电压/间隙存在偏差。
        double[] nowDeg = armController.getState().getServoNowDegree();   // 软件推算
        double[] targetDeg = activeTraj.get(0).jointDegrees;

        telemetry.addData("State", "MOVING_TO_START (open-loop)");
        telemetry.addData("X=Play", "A=Home (cancel)");
        telemetry.addData("est. deg", "%.1f %.1f %.1f %.1f %.1f %.1f",
                nowDeg[0], nowDeg[1], nowDeg[2], nowDeg[3], nowDeg[4], nowDeg[5]);
        telemetry.addData("target deg", "%.1f %.1f %.1f %.1f %.1f %.1f",
                targetDeg[0], targetDeg[1], targetDeg[2], targetDeg[3], targetDeg[4], targetDeg[5]);
        telemetry.addLine("⚠️ 'est. deg' is software estimate, NOT sensor reading.");
    }

    // ========================================================================
    //  PLAYING 状态
    // ========================================================================

    private void tickPlaying() {
        long nowNano = System.nanoTime();
        double elapsedReal = (nowNano - trajStartNano) / 1e9;
        double curT = elapsedReal * lowSpeedFactor;

        // 超时检测：实际耗时超过预期 × timeoutFactor 则强制停止
        if (timeoutFactor > 0) {
            double maxAllowed = trajTotalDuration / Math.max(lowSpeedFactor, 0.01) * timeoutFactor;
            if (elapsedReal > maxAllowed) {
                telemetry.addData("TIMEOUT", "%.1fs > %.1fs (x%.1f)", elapsedReal, maxAllowed, timeoutFactor);
                finishTrajectory();
                return;
            }
        }

        // 轨迹结束
        //TODO check最后是不是突然“闪”到终点。是的说明中间没跟上
        if (curT >= trajTotalDuration) {
            double[] finalDegs = activeTraj.get(trajN - 1).jointDegrees;
            outputter.setDegree(finalDegs);
            for (int j = 0; j < 6; j++) {
                lastTargetAngles[j] = finalDegs[j];
                lastSentPositions[j] = SixServoArmOutputter.toPosition(j, finalDegs[j]);
            }
            telemetry.addLine("=== Trajectory Complete ===");
            telemetry.addData("Avg send delay (us)", "%.1f",
                    trajLoopCount > 0 ? trajTotalSendDelayNs / 1e3 / trajLoopCount : 0);
            telemetry.addData("Max send delay (us)", "%.1f", trajMaxSendDelayNs / 1e3);
            finishTrajectory();
            return;
        }

        // 定位当前段
        while (trajCurrentSegment + 1 < trajN && trajTimes[trajCurrentSegment + 1] <= curT) {
            trajCurrentSegment++;
        }
        if (trajCurrentSegment + 1 >= trajN) {
            finishTrajectory();
            return;
        }

        // 逐关节插值（使用预构建的插值器，零分配）
        double[] targetDeg = new double[6];
        for (int j = 0; j < 6; j++) {
            targetDeg[j] = trajInterpolators[j].interpolate(curT);
        }

        // 下发
        outputter.setDegree(targetDeg);

        // 日志
        long afterSendNano = System.nanoTime();
        long sendDelayNs = afterSendNano - nowNano;
        trajTotalSendDelayNs += sendDelayNs;
        if (sendDelayNs > trajMaxSendDelayNs) trajMaxSendDelayNs = sendDelayNs;

        for (int j = 0; j < 6; j++) {
            lastTargetAngles[j] = targetDeg[j];
            lastSentPositions[j] = SixServoArmOutputter.toPosition(j, targetDeg[j]);
        }
        trajLoopCount++;

        // 播放中 telemetry
        telemetry.addData("State", "PLAYING");
        telemetry.addData("当前实际时间（经过缩放） / 轨迹总*预期*时间", "%.2f / %.2f s", curT, trajTotalDuration);
        telemetry.addData("lowSpeedFactor", "%.2f", lowSpeedFactor);
        telemetry.addData("阶段(points - 1)?", "%d / %d", trajCurrentSegment, trajN - 1);
        telemetry.addData("send_delay_ms", "%d", sendDelayNs / 1000_000);
        telemetry.addData("Y=ToggleSpeed", "");
    }

    // ========================================================================
    //  轨迹生命周期
    // ========================================================================

    private void generateAndStart() {

        long genStartNano = System.nanoTime();

        Point3D startPt = new Point3D(startX, startY, startZ);
        Point3D endPt   = new Point3D(endX, endY, endZ);

        List<StraightLineTrajectoryGenerator.TrajectoryPoint> rawPts =
                StraightLineTrajectoryGenerator.generateLine(
                        startPt, endPt, clipHeadingRadian,RadianAroundArm3 ,SixServoArmOutputter.ClipOpenPosition,numSamples);

        List<StraightLineTrajectoryGenerator.TimedTrajectoryPoint> timedPts =
                StraightLineTrajectoryGenerator.timeScaleByServoSpeed(
                        rawPts, minStepDeg, wakeupDeg);

        long genTimeMs = (System.nanoTime() - genStartNano) / 1_000_000;

        if (timedPts == null || timedPts.size() < 2) {
            telemetry.addLine("Trajectory too short — aborting.");
            return;
        }

        // 初始化播放状态
        activeTraj = timedPts;
        trajN = timedPts.size();
        trajTotalDuration = timedPts.get(trajN - 1).timeSeconds;

        trajTimes = new double[trajN];
        trajJointDegs = new double[6][trajN];
        trajInterpolators = new LinearInterpolation[6];  // 一次性构建，不再每帧 new
        for (int i = 0; i < trajN; i++) {
            trajTimes[i] = timedPts.get(i).timeSeconds;
            double[] degs = timedPts.get(i).jointDegrees;
            for (int j = 0; j < 6; j++) {
                trajJointDegs[j][i] = degs[j];
            }
        }
        for (int j = 0; j < 6; j++) {
            trajInterpolators[j] = new LinearInterpolation(trajTimes, trajJointDegs[j]);
        }

        trajStartNano = 0; // 将在到达起点后重置
        trajCurrentSegment = 0;
        trajLoopCount = 0;
        trajTotalSendDelayNs = 0;
        trajMaxSendDelayNs = 0;

        // 先下发起点角度，让舵机开始移动；用户确认到位后按 B 开始播放
        double[] startDegs = timedPts.get(0).jointDegrees;
        outputter.setDegree(startDegs);

        telemetry.addLine("=== Moving arm to start ===");
        telemetry.addData("Points", trajN);
        telemetry.addData("Gen time (ms)", "%d", genTimeMs);
        telemetry.addData("Duration (s)", "%.2f", trajTotalDuration / lowSpeedFactor);
        telemetry.addLine("Press B when arm is at start position.");

        mode = Mode.MOVING_TO_START;
    }

    private void finishTrajectory() {
        telemetry.addLine("Trajectory playback ended.");
        mode = Mode.IDLE;
        activeTraj = null;
    }

    // ========================================================================
    //  回 Home
    // ========================================================================

    private void goHome() {
        mode = Mode.IDLE;  // 打断正在播放的轨迹
        telemetry.addLine("Going HOME...");
        telemetry.addData("XYZ", "(%.1f,%.1f,%.1f)", homeX, homeY, homeZ);
        telemetry.update();

        Point3D homePt = new Point3D(homeX, homeY, homeZ);
        double[] radians = armController.getCalculator().calculateToServo(homePt, clipHeadingRadian);
        double[] homeDegs = new double[6];
        for (int j = 0; j < 4; j++) {
            homeDegs[j] = Math.toDegrees(radians[j]);
        }
        homeDegs[4] = RadianAroundArm3;
        homeDegs[5] = SixServoArmOutputter.toDegree(5, SixServoArmOutputter.ClipOpenPosition);
        outputter.setDegree(homeDegs);

        // ⚠️ ifFinishedMoving() 基于软件推算，非传感器反馈。
        // 此处等待仅保证模型认为已到位，不代表物理舵机真实到达。
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && !armController.getState().ifFinishedMoving() && t.seconds() < 3.0) {
            telemetry.addData("Moving to home (est.)", "%.1f s", t.seconds());
            telemetry.update();
            sleepSafe(80);
        }
        telemetry.addLine(armController.getState().ifFinishedMoving()
                ? "HOME (model says reached)."
                : "HOME timeout.");
    }

    // ========================================================================
    //  辅助
    // ========================================================================

    private void sleepForCycle() {
        long targetNs = (long) (1e9 / controlFreq);
        long sleepNs = targetNs - 2_000_000; // 预留 2ms 给逻辑执行
        if (sleepNs > 500_000) {
            sleepSafe(sleepNs / 1_000_000);
        }
    }

    private void sleepSafe(long ms) {
        if (ms <= 0) return;
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
