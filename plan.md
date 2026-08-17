# 蜂鸣器驱动 · 架构设计方案

> 项目：RM2027（STM32H723 + FreeRTOS + 自研 PYRo 框架）
> 文档定位：蜂鸣器（被动蜂鸣器，PWM 变频发声）驱动的架构与实现方案。

## 0. 背景与现状

- 硬件已预留蜂鸣器 PWM：`CubeMX/Core/Src/tim.c` 中 `htim3` 配置为 PWM 输出
  - 通道：`TIM3_CH4` → 引脚 **PB1**
  - `Prescaler = 24 - 1`，`Period = 65535`，`Pulse = 0`（默认静音），`TIM_OCMODE_PWM1`
- 时钟树（`SystemClock_Config`）：
  - HSI 64MHz → PLL → SYSCLK 480MHz，AHB 240MHz，APB1 120MHz
  - TIM3 位于 APB1，**定时器时钟 = 240MHz**
  - 除以 `Prescaler = 24` 后，**计数时钟 = 10MHz**
  - `Period = 10_000_000 / 频率 - 1` 即可设置音调
- 项目分层：`CubeMX/`（HAL 生成代码，仅在 USER CODE 区改动）、`PYRo/Peripheral/`（底层外设驱动）、`PYRo/Component/`（组件级业务驱动）、`PYRo/Core/`（任务/锁/内存/通用类型）。

---

## 1. 总体架构：两层（贴合 PYRo 现有分层）

| 层 | 职责 | 对应现有例子 |
|---|---|---|
| **Peripheral（硬件抽象）** | 封装 HAL TIM PWM：频率、占空比、启停 | `pyro_pwm_drv` + `pyro_bsp_pwm` |
| **Component（业务/播放逻辑）** | 音符、旋律、节奏、速度、异步后台播放、暂停/循环 | `pyro_dji_motor_drv`（基于 CAN）、RC（基于 UART） |

硬件绑定被隔离在 BSP 层，播放逻辑可复用、可替换。

---

## 2. 文件布局（新增）

```
PYRo/Peripheral/PWM/
├── pyro_pwm_drv.h/.cpp           # pwm_drv_t（通用 PWM 驱动，封装 HAL TIM）
├── pyro_bsp_pwm.h/.cpp           # bsp_pwm（多 PWM 实例管理，绑定具体通道）
└── README.md

PYRo/Component/Buzzer/
├── pyro_buzzer_player.h/.cpp     # buzzer_player_t（音符/旋律/节奏/异步任务，复用 pwm_drv_t）
└── README.md
```

---

## 3. 类设计与接口映射

### 3.1 `pwm_drv_t`（Peripheral/PWM，通用 PWM 驱动）

pwm_drv_t 是**独立于具体设备（蜂鸣器/舵机/电调）的通用外设驱动**：只负责定时器 PWM 通道的周期/频率、脉宽/占空比与启停，不包含任何设备业务语义。
同时覆盖「频率型」（蜂鸣器/电调）与「脉宽型」（舵机）两类 PWM 场景：

```cpp
class pwm_drv_t {
    friend class bsp_pwm;                         // 私有构造，仅 BSP 可建
public:
    status_t init();                              // HAL_TIM_PWM_Start
    status_t deinit();                            // HAL_TIM_PWM_Stop
    status_t start();                             // 使能输出
    status_t stop();                              // 关闭输出
    bool is_running() const;

    // —— 频率型 ——
    status_t set_frequency(uint32_t hz);          // 重算 ARR + 保持 duty 的 CCR
    uint32_t get_frequency() const;

    // —— 占空比 ——
    status_t set_duty_cycle(uint8_t percent);     // 0-100
    uint8_t  get_duty_cycle() const;

    // —— 脉宽型 ——
    status_t set_period(uint32_t period);         // 直接设 ARR
    status_t set_pulse(uint32_t pulse);           // 直接设 CCR
private:
    explicit pwm_drv_t(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t timer_clk_hz);
    void update_ccr();                            // period * duty / 100 → CCR
    TIM_HandleTypeDef* _htim;
    uint32_t _channel;
    uint32_t _timer_clk_hz;                       // 10MHz
    uint32_t _period, _pulse;
    uint8_t  _duty;                               // 初始 0（安全静音），由调用方设置
    uint32_t _freq_hz;
};
```

内部用 `HAL_TIM_PWM_Start/Stop` + `__HAL_TIM_SET_AUTORELOAD` / `__HAL_TIM_SET_COMPARE`（或 `HAL_TIM_PWM_ConfigChannel`）。

### 3.2 `bsp_pwm`（Peripheral/PWM，硬件绑定）

```cpp
class bsp_pwm {
public:
    enum which_pwm
    {
        pwm_tim3_ch4,                 // 蜂鸣器（TIM3_CH4 / PB1）
        // 未来扩展：pwm_tim3_ch2、pwm_tim1_ch1 ...
    };

    static pwm_drv_t& get_tim3_ch4(); // 绑定 &htim3 + TIM_CHANNEL_4 + 10MHz
    static pwm_drv_t* get_pwm(which_pwm which);
    static status_t init_all();       // 触发各实例首次构造并 init
};
```

### 3.3 `buzzer_player_t`（Component，播放逻辑层，含后台任务）

```cpp
enum class note_t : uint8_t { C, CS, D, DS, E, F, FS, G, GS, A, AS, B };

struct rhythm_step_t { note_t note; uint8_t octave; uint16_t beats; bool rest; };

class buzzer_player_t {
public:
    explicit buzzer_player_t(pwm_drv_t& pwm);   // 传入 bsp_pwm::get_tim3_ch4()
    ~buzzer_player_t();

    status_t beep(uint32_t frequency, uint32_t duration_ms);
    status_t beep(uint32_t duration_ms);                          // 默认 4000Hz

    status_t start();
    status_t stop();
    bool is_playing() const;

    status_t play_note(uint16_t frequency, uint32_t duration_ms); // NOTE_C4 常量走这里
    status_t play_note(note_t note, uint8_t octave, uint32_t duration_ms);

    status_t play_melody(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_melody_blocking(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_melody_async(const uint16_t notes[], const uint32_t durations[], uint32_t len);

    status_t play_rhythm(const rhythm_step_t pattern[], uint32_t len);

    status_t set_volume(uint8_t percent);   // 0-100 → duty
    uint8_t  get_volume() const;
    status_t mute();                        // 静音（duty=0 或 stop）
    status_t unmute();

    void set_tempo(uint16_t bpm);
    uint16_t get_tempo() const;

    status_t pause();                       // 挂起后台任务
    status_t resume();                      // 恢复后台任务
    void set_loop(bool enable);

    static uint32_t note_to_freq(note_t note, uint8_t octave);  // 12-TET, A4=440
private:
    pwm_drv_t& _pwm;                    // 所有发声转成 _pwm.set_frequency/start/stop
    // 内部后台任务：继承 pyro::task_base_t，管理异步播放队列
    class player_task_t;                // 持有 FreeRTOS 命令队列
};
```

预定义常量（`constexpr uint16_t`，C4 八度示例，可按需扩展全音域）：

```cpp
constexpr uint16_t NOTE_C4 = 262, NOTE_D4 = 294, NOTE_E4 = 330, NOTE_F4 = 349,
                   NOTE_G4 = 392, NOTE_A4 = 440, NOTE_B4 = 494; // ...
```

---

## 4. 关键实现要点

1. **频率计算**：TIM3 定时器时钟 = 240MHz（APB1×2），`Prescaler=24-1` → 计数时钟 **10MHz**。
   `pwm_drv_t::set_frequency(hz)` 里 `ARR = 10'000'000 / hz - 1`，`CCR = ARR * duty / 100`（duty 由调用方设置，蜂鸣器推荐 50%）。
   当前 CubeMX 的 `Period=65535` 对应约 152Hz，仅作初始值。

2. **音符频率**：`note_to_freq(n, octave)` 用十二平均律 `440 * 2^((n - 9)/12 + (octave-4))`，整数化即可。

3. **异步播放（`play_melody_async`/`beep` 非阻塞）**：`player_task_t` 继承 `pyro::task_base_t`（`init()` + `run_loop()`），
   内部用一个 FreeRTOS `QueueHandle_t` 接收命令（`PLAY_NOTE / PLAY_MELODY / PAUSE / RESUME / STOP / SET_LOOP`）。
   `run_loop` 阻塞读队列 → 逐音符 `_pwm.set_frequency` + `_pwm.start` + `vTaskDelay(pdMS_TO_TICKS(duration))`。
   `pause()` 用 `vTaskSuspend`，`resume()` 用 `vTaskResume`（`task_base_t` 的 `_loop_task_handle` 可访问）。

4. **阻塞接口**：`beep` 与 `play_melody`/`play_melody_blocking` 直接在调用线程里逐音符延时，不经过后台任务。

5. **音量/静音**：`set_volume` 映射到 `set_duty_cycle`；`mute()` 记下当前音量并置 duty=0，`unmute()` 恢复。
   ⚠️ 被动蜂鸣器靠谐振，占空比对音量的影响有限且非线性的，这是硬件物理限制，会在文档里注明。

---

## 5. 需要同步改动的现有文件

1. `PYRo/CMakeLists.txt`：
   - `target_sources` 增加 `Peripheral/PWM/pyro_pwm_drv.cpp`、`Peripheral/PWM/pyro_bsp_pwm.cpp`、`Component/Buzzer/pyro_buzzer_player.cpp`
   - `target_include_directories` 增加 `Peripheral/PWM`、`Component/Buzzer`
2. `PYRo/Core/Config/pyro_core_config.h`：加 `#define PYRO_PWM_TIM3_CH4 pyro::bsp_pwm::get_tim3_ch4()`（对齐 `PYRO_UART1` 风格；蜂鸣器播放器是组件，由 robot 层直接引用该宏构造）

---

## 6. 需确认的假设

1. **`play_melody`（无后缀）默认阻塞还是异步？** 建议默认**阻塞**（安全默认），`play_melody_async` 显式后台。
2. **`beep(freq, dur)` 是阻塞还是异步？** 建议 `beep` 为**阻塞**一次性发声（简单场景），异步走 `play_note`/`play_melody_async`。
3. **`play_rhythm` 的 pattern 结构**：采用 `{note, octave, beats, rest}` 拍子模型（受 `tempo` 控制）。

---

## 7. 实施步骤

1. 编写 `PYRo/Peripheral/PWM/pyro_pwm_drv.h/.cpp`（通用 PWM 驱动，与设备解耦）
2. 编写 `PYRo/Peripheral/PWM/pyro_bsp_pwm.h/.cpp`（多 PWM 实例管理，绑定 htim3 + CH4）
3. 编写 `PYRo/Component/Buzzer/pyro_buzzer_player.h/.cpp`（复用 pwm_drv_t）
4. 更新 `PYRo/CMakeLists.txt`（source + include）
5. 更新 `PYRo/Core/Config/pyro_core_config.h`（加 `PYRO_PWM_TIM3_CH4` 宏）
6. 补充两个 README.md
7. 编译验证