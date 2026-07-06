任务（高优先级）

在现有 SixServoArm 仓库基础上，新增功能：机器人机械臂能绘制一条三维直线（由外部给定起点与终点），（保留后期升级为曲线或更复杂图形的能力）。使用仓库内已存在的逆运动学（SixServoArmCalculator.calculateToServo）进行关节角计算，按“最慢舵机能力”做 time-scaling 并播放到硬件。

必须复用的代码/类（不要重写）

SixServoArmCalculator.calculateToServo(Point3D, double) — 用作逆运动学（返回4个弧度值）。

SixServoArmOutputter（toDegree/toPosition/setRadian/setPosition）— 用来把角度映射到 servo.setPosition(...)。

SixServoArmState.servoSpeed[] — 用作计算每个舵机名义速度（vmax = 60 / servoSpeed）。

严格要求：在运行时对 time→角度表做线性插值时，必须使用已有的线性插值工具类： TeamCode\src\main\java\org\firstinspires\ftc\teamcode\utility\LinearInterpolation\LinearInterpolator.java

新增文件与函数（明确签名）

TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controllers/sixservoarm/StraightLineTrajectoryGenerator.java

public static List<TrajectoryPoint> generateLine(Point3D start, Point3D end, double clipHeadingRadian, int numSamples)

TrajectoryPoint 包含角度数组（deg）。

public static List<TimedTrajectoryPoint> timeScaleByServoSpeed(List<TrajectoryPoint> pts)

使用 SixServoArmState.servoSpeed 做 time-scaling（最慢舵机决定段时间）。

TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controllers/sixservoarm/TrajectoryPlayerOpMode.java

OpMode（LinearOpMode）：加载 time→θ 表（或 CSV），播放轨迹：

控制循环用 System.nanoTime() 为时基；

在运行时对每个舵机使用 LinearInterpolation 做时间插值以求 θ(t)；

下发命令用 SixServoArmOutputter.setRadian/setDegree/setPosition；

记录日志：expectedTime, actualSendTime, targetAngles[], sentPositions[]；

支持参数：controlFreq（默认 25 Hz）、minStepDeg（默认 0.5°）、lowSpeedFactor（默认 0.5）。

CSV 与测试向导（必带）

轨迹 CSV header: time_s,theta0_deg,...,theta5_deg。

README 增加运行步骤（如何生成 CSV、如何上传、如何在 lowSpeed 运行、如何检查 telemetry 日志）。

关键设计要点（必须在实现中体现）

Time-scaling（必须实现）

离线：离散化直线 → 对每点调用 Calculator.calculateToServo(...) → 转为度 → 计算相邻点 Δθ → t_seg = max_i(|Δθ_i|/vmax_i) → 累计构成 times[]。

静摩擦与回差（必须处理）

方法1：在生成表或运行时应用 minStepDeg（可配置，建议 0.5°）；对小增量合并或跳过；方法2：在方向改变处可配置唤醒偏置（wakeupDeg，默认 2.0°）。
两个方法分别实现，我会在实机上运行比较效果

时序与延迟（必须处理）

使用 System.nanoTime()；运行时按当前时间在 times[] 中查区间并用 LinearInterpolation 对每个关节做线性插值得到 θ(t)；不要依赖 Thread.sleep 精准时序；记录实际发送时间用于诊断。


无闭环反馈（必须缓解）
累计漂移：准备一个可单独执行的命令，回 home（home 可在配置里定义），做复位校准，以及超时检测与安全停止接口。