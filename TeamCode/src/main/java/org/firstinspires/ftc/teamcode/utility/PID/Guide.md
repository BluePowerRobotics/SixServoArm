# PID 控制器使用指南

## 概述

`PID` 目录包含FTC机器人控制所需的PID控制器系列类，支持比例-积分-微分控制、SVA前馈控制以及多slot配置。

---

## 类说明

### 1. PIDController

基础PID控制器，用于根据目标值和测量值计算控制输出。

```java
// 创建PID控制器
PIDController pid = new PIDController(kP, kI, kD);
PIDController pid = new PIDController(kP, kI, kD, maxI); // 带积分上限

// 每帧调用
double output = pid.calculate(setpoint, measurement, dt);

// 重置积分和微分状态
pid.reset();

// 运行时修改参数
pid.setPID(newKP, newKI, newKD);
pid.setMaxI(newMaxI);
```

**参数说明：**
- `kP`: 比例系数，增大响应速度
- `kI`: 积分系数，消除稳态误差
- `kD`: 微分系数，抑制超调
- `maxI`: 积分上限，防止积分饱和
- `dt`: 时间间隔（秒），两次调用之间的时间差

### 2. SVAController

SVA（静态摩擦、速度、加速度）前馈控制器，用于计算电机控制的前馈项。

```java
// 创建SVA控制器
SVAController sva = new SVAController(kS, kV, kA);

// 计算前馈输出
double feedforward = sva.calculate(velocity, acceleration);

// 运行时修改参数
sva.setSVA(newKS, newKV, newKA);
```

**参数说明：**
- `kS`: 静态摩擦系数
- `kV`: 速度系数
- `kA`: 加速度系数

### 3. PIDSVAController

结合PID和SVA前馈的高级控制器，支持多slot配置，可根据不同场景切换参数。

```java
// 创建PIDSVA控制器，配置slot0
SlotConfig slot0 = new SlotConfig()
    .withKP(0.1).withKI(0.01).withKD(0.001)
    .withMaxI(0.5)
    .withKS(0.05).withKV(0.12).withKA(0.01)
    .withOutputLimits(-1.0, 1.0);

PIDSVAController controller = new PIDSVAController().withSlot0(slot0);

// 配置多个slot
SlotConfig slot1 = new SlotConfig()
    .withKP(0.2).withKI(0.0).withKD(0.002)
    .withKS(0.03).withKV(0.10).withKA(0.005)
    .withOutputLimits(-0.8, 0.8);
controller.withSlot(1, slot1);

// 切换slot
controller.setSlot(1);

// 简单位置闭环
double output = controller.calculate(setpoint, measurement, dt, false);

// 简单速度闭环（setpoint作为前馈速度项）
double output = controller.calculate(setpoint, measurement, dt, true);

// 完整PIDSVA闭环
double output = controller.calculate(setpoint, measurement, velocity, acceleration, dt);

// 重置状态
controller.reset();
```

### 4. SlotConfig

建造者模式配置类，用于设置PID和SVA参数。

```java
SlotConfig config = new SlotConfig()
    .withKP(0.1)        // 比例系数
    .withKI(0.01)       // 积分系数
    .withKD(0.001)      // 微分系数
    .withMaxI(0.5)      // 积分上限
    .withKS(0.05)       // 静态摩擦系数
    .withKV(0.12)       // 速度系数
    .withKA(0.01)       // 加速度系数
    .withOutputLimits(-1.0, 1.0); // 输出限幅
```

---

## 典型用法示例

### 电机位置控制

```java
PIDSVAController motorController = new PIDSVAController().withSlot0(
    new SlotConfig()
        .withKP(0.05).withKI(0.0).withKD(0.001)
        .withKS(0.1).withKV(0.15).withKA(0.02)
        .withOutputLimits(-1.0, 1.0)
);

// 在循环中
double targetPos = 1000; // 目标编码器位置
double currentPos = motor.getCurrentPosition();
double currentVel = motor.getVelocity(); // 需自行计算
double output = motorController.calculate(targetPos, currentPos, currentVel, 0, dt);
motor.setPower(output);
```

### 电机速度控制

```java
PIDSVAController velocityController = new PIDSVAController().withSlot0(
    new SlotConfig()
        .withKP(0.01).withKI(0.001).withKD(0.0)
        .withKS(0.05).withKV(0.12).withKA(0.01)
        .withOutputLimits(-1.0, 1.0)
);

// 在循环中
double targetVel = 500; // 目标速度（编码器tick/s）
double currentVel = motor.getVelocity();
double output = velocityController.calculate(targetVel, currentVel, dt, true);
motor.setPower(output);
```