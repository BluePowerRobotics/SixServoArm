package org.firstinspires.ftc.teamcode.controllers.sixservoarm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;

import java.util.Arrays;

/**
 * SixServoArmTuner — 机械臂舵机参数标定 OpMode。
 *
 * <h3>Dashboard 可调参数</h3>
 * <ul>
 *   <li><b>deltaDegree</b> — 每次按方向键时舵机转动的角度（度）</li>
 *   <li><b>targetDegrees[0..5]</b> — 6 个舵机的目标角度，可直接在 Dashboard 上修改</li>
 * </ul>
 *
 * <h3>手柄操作</h3>
 * <ul>
 *   <li><b>A / B</b> — 切换当前选中的舵机（0–5）</li>
 *   <li><b>D-Pad 上 / 下</b> — 将选中舵机角度 +/- deltaDegree，同时自动开始计时</li>
 *   <li><b>Right Bumper</b> — 确认选中舵机已到位（人眼看），锁定耗时</li>
 *   <li><b>Left Bumper</b> — 取消选中舵机计时（重新计时）</li>
 *   <li><b>Y</b> — 将选中舵机重置到零点（zeroPosition）</li>
 *   <li><b>X</b> — 将所有舵机重置到零点</li>
 * </ul>
 *
 * <p>零点角度取自 {@code SixServoArmOutputter.servoZeroPositionDegree}，
 * 角度范围受 {@code SixServoArmOutputter.servoRangeDegree} 约束。</p>
 */
@Config
@TeleOp(name = "SixServoArmTuner", group = "TEST")
public class SixServoArmTuner extends LinearOpMode {

    /** 每次方向键按下时，选中舵机转动的角度（度） */
    public static double deltaDegree = 5.0;

    /** 6 个舵机的目标角度（度），可直接在 Dashboard 修改 */
    public static double[] targetDegrees = new double[6];

    /** 当前选中的舵机索引 (0–5) */
    private int selectedServo = 0;

    /** 机械臂控制器 */
    private SixServoArmController controller;

    /** 帧率计时 */
    private long lastTimeMs, nowTimeMs;
    private long lastSetTimeMs;

    /** 每舵机移动计时（手动确认模式）：D-Pad 按下时开始计时，Right Bumper 确认到位 */
    private final long[] moveStartTimeMs = new long[6];
    private final double[] lastMoveElapsedSec = new double[6];
    /** 上一次写入的目标角度，用于检测 Dashboard 直接修改 */
    private final double[] prevTargetDegrees = new double[6];

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- 初始化 ----
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        InstanceTelemetry.init(telemetry);

        controller = new SixServoArmController(hardwareMap);

        // 将零点角度加载为默认 targetDegrees（仅在首次为 0 时初始化，避免覆盖 Dashboard 已设值）
        for (int i = 0; i < 6; i++) {
            if (targetDegrees[i] == 0.0) {
                targetDegrees[i] = SixServoArmOutputter.servoZeroPositionDegree[i];
            }
            prevTargetDegrees[i] = targetDegrees[i];
        }

        waitForStart();
        lastTimeMs = nowTimeMs = System.currentTimeMillis();

        // ---- 主循环 ----
        while (opModeIsActive()) {
            handleGamepad();
            applyServoTargets();
            updateTelemetry();
        }
    }

    /**
     * 处理手柄输入。
     */
    private void handleGamepad() {
        // 切换选中舵机
        if (gamepad1.aWasReleased()) {
            selectedServo = (selectedServo + 1) % 6;
        }
        if (gamepad1.bWasReleased()) {
            selectedServo = (selectedServo - 1 + 6) % 6;
        }

        // 选中舵机 ± deltaDegree（同时自动开始计时）
        if (gamepad1.dpadUpWasReleased()) {
            addToServo(selectedServo, deltaDegree);
            startTimer(selectedServo);
        }
        if (gamepad1.dpadDownWasReleased()) {
            addToServo(selectedServo, -deltaDegree);
            startTimer(selectedServo);
        }

        // 确认选中舵机已到位 → 锁定耗时
        if (gamepad1.rightBumperWasReleased()) {
            confirmArrival(selectedServo);
        }

        // 取消计时（重新开始）
        if (gamepad1.leftBumperWasReleased()) {
            startTimer(selectedServo);
        }

        // 重置选中舵机到零点
        if (gamepad1.yWasReleased()) {
            targetDegrees[selectedServo] = SixServoArmOutputter.servoZeroPositionDegree[selectedServo];
        }

        // 重置全部舵机到零点
        if (gamepad1.xWasReleased()) {
            for (int i = 0; i < 6; i++) {
                targetDegrees[i] = SixServoArmOutputter.servoZeroPositionDegree[i];
            }
        }
    }

    /** 启动指定舵机的计时器（D-Pad 或 Left Bumper 触发）。 */
    private void startTimer(int index) {
        moveStartTimeMs[index] = System.currentTimeMillis();
        lastMoveElapsedSec[index] = 0;
    }

    /** 人工确认到位，锁定耗时。 */
    private void confirmArrival(int index) {
        if (moveStartTimeMs[index] > 0 && lastMoveElapsedSec[index] == 0) {
            lastMoveElapsedSec[index] = (System.currentTimeMillis() - moveStartTimeMs[index]) / 1000.0;
        }
    }

    /**
     * 给指定舵机的目标角度增加增量，并钳制在合法范围内。
     */
    private void addToServo(int index, double delta) {
        double zero = SixServoArmOutputter.servoZeroPositionDegree[index];
        double range = SixServoArmOutputter.servoRangeDegree[index];
        targetDegrees[index] = clamp(targetDegrees[index] + delta, zero, zero + range);
    }

    /**
     * 将 targetDegrees 写入舵机（受 50ms 节流），
     * 仅检测 Dashboard 直接修改目标值时自动开始计时。
     */
    private void applyServoTargets() {
        nowTimeMs = System.currentTimeMillis();
        if (nowTimeMs - lastSetTimeMs > 50) {
            for (int i = 0; i < 6; i++) {
                // 检测 Dashboard 直接修改（非手柄触发）
                if (Math.abs(targetDegrees[i] - prevTargetDegrees[i]) > 0.01) {
                    startTimer(i);
                    prevTargetDegrees[i] = targetDegrees[i];
                }
                controller.getOutputter().setDegree(i, targetDegrees[i]);
            }
            lastSetTimeMs = nowTimeMs;
        }
    }

    /**
     * 更新 Dashboard / DS 遥测。
     */
    private void updateTelemetry() {
        lastTimeMs = nowTimeMs;
        nowTimeMs = System.currentTimeMillis();

        telemetry.addData("FPS", String.format("%.1f", 1000.0 / Math.max(1, nowTimeMs - lastTimeMs)));
        telemetry.addLine();

        telemetry.addData("Selected Servo", selectedServo);
        telemetry.addData("Delta Degree", String.format("%.2f°", deltaDegree));
        telemetry.addLine();

        // 逐舵机显示信息
        for (int i = 0; i < 6; i++) {
            double zero = SixServoArmOutputter.servoZeroPositionDegree[i];
            double range = SixServoArmOutputter.servoRangeDegree[i];
            double pos = SixServoArmOutputter.toPosition(i, targetDegrees[i]);
            String marker = (i == selectedServo) ? " ◀" : "";

            // 移动耗时：已确认到位则显示固定值，计时中显示实时计时，否则 —
            String timeStr;
            if (lastMoveElapsedSec[i] > 0) {
                timeStr = String.format("✔ %.2fs", lastMoveElapsedSec[i]);
            } else if (moveStartTimeMs[i] > 0) {
                double elapsed = (System.currentTimeMillis() - moveStartTimeMs[i]) / 1000.0;
                timeStr = String.format("⏳ %.2fs  (RB确认)", elapsed);
            } else {
                timeStr = "—";
            }

            telemetry.addData(
                    String.format("Servo %d%s", i, marker),
                    String.format("Deg: %7.2f  Pos: %.3f  [%.1f~%.1f]  %s",
                            targetDegrees[i], pos, zero, zero + range, timeStr)
            );
        }

        telemetry.addLine();
        telemetry.addData("Now Degrees", Arrays.toString(controller.getState().getServoNowDegree()));
        telemetry.addData("Target Degrees", Arrays.toString(controller.getState().getServoTargetDegree()));
        telemetry.addLine();

        telemetry.addLine("A/B: select | D-Up/Down: ±delta° + timer start");
        telemetry.addLine("RB: confirm arrival | LB: restart timer");
        telemetry.addLine("Y: reset selected → zero | X: reset ALL → zero");

        telemetry.update();
    }

    /**
     * 将值钳制在 [min, max] 范围内。
     */
    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}

