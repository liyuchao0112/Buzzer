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
    friend class bsp_pwm;                         // 单例约束：实例仅由 BSP 局部静态创建（见 3.2.1）
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

#### 3.1.1 实现细节（pyro_pwm_drv.cpp 草案）

```cpp
// 构造函数：保存硬件绑定，读取 CubeMX 初始 ARR/CCR，反推初始占空比
pwm_drv_t::pwm_drv_t(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timer_clk_hz)
    : _htim(htim), _channel(channel), _timer_clk_hz(timer_clk_hz),
      _period(__HAL_TIM_GET_AUTORELOAD(htim)),
      _pulse(__HAL_TIM_GET_COMPARE(htim, channel)),
      _duty((uint8_t)(_pulse * 100u / (_period + 1u))),
      _freq_hz(_timer_clk_hz / (_period + 1u))
{
}

status_t pwm_drv_t::init()
{
    _running = true;
    return (HAL_TIM_PWM_Start(_htim, _channel) == HAL_OK) ? PYRO_OK : PYRO_ERROR;
}
status_t pwm_drv_t::deinit()
{
    _running = false;
    HAL_TIM_PWM_Stop(_htim, _channel);
    return PYRO_OK;
}
status_t pwm_drv_t::start()   { return init(); }
status_t pwm_drv_t::stop()    { return deinit(); }
bool     pwm_drv_t::is_running() const { return _running; }

// 频率型：ARR = 时钟/频率 - 1，同时保持当前占空比
status_t pwm_drv_t::set_frequency(uint32_t hz)
{
    if (hz == 0u || hz > _timer_clk_hz) return PYRO_PARAM_ERROR;
    _freq_hz = hz;
    _period  = _timer_clk_hz / hz - 1u;
    __HAL_TIM_SET_AUTORELOAD(_htim, _period);
    update_ccr();
    return PYRO_OK;
}
uint32_t pwm_drv_t::get_frequency() const { return _freq_hz; }

// 占空比：0-100 → CCR = ARR * duty / 100
status_t pwm_drv_t::set_duty_cycle(uint8_t percent)
{
    if (percent > 100u) return PYRO_PARAM_ERROR;
    _duty = percent;
    update_ccr();
    return PYRO_OK;
}
uint8_t pwm_drv_t::get_duty_cycle() const { return _duty; }

// 脉宽型：直接设 ARR/CCR（舵机等）；与频率型共用实例时不要混用
status_t pwm_drv_t::set_period(uint32_t period)
{
    if (period == 0u) return PYRO_PARAM_ERROR;
    _period = period;
    __HAL_TIM_SET_AUTORELOAD(_htim, _period);
    update_ccr();
    return PYRO_OK;
}
status_t pwm_drv_t::set_pulse(uint32_t pulse)
{
    _pulse = pulse;
    __HAL_TIM_SET_COMPARE(_htim, _channel, _pulse);
    return PYRO_OK;
}

void pwm_drv_t::update_ccr()
{
    _pulse = (uint32_t)_period * _duty / 100u;
    __HAL_TIM_SET_COMPARE(_htim, _channel, _pulse);
}
```

> 头文件补充私有成员 `volatile bool _running = false;`。`init`/`deinit` 与 `start`/`stop` 为同一语义的别名（蜂鸣器层用 `start`/`stop`，整体初始化用 `init`）。频率型与脉宽型不要在同一实例上混用 `set_frequency` 与 `set_period`。

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

#### 3.2.1 实现细节（pyro_bsp_pwm.cpp 草案）

```cpp
#include "pyro_bsp_pwm.h"
#include "tim.h"          // extern TIM_HandleTypeDef htim3

namespace pyro {

// 定时器计数时钟：= TIM3 定时器时钟(240MHz) / Prescaler(24) = 10MHz
// 跨机器人需调整时可提为宏入 pyro_core_config.h（见第 4 节「常量定义位置」）
constexpr uint32_t kTim3TimerClkHz = 10'000'000u;

pwm_drv_t &bsp_pwm::get_tim3_ch4()
{
    // 局部静态单例：首次调用触发构造
    static pwm_drv_t instance(&htim3, TIM_CHANNEL_4, kTim3TimerClkHz);
    return instance;
}

pwm_drv_t *bsp_pwm::get_pwm(which_pwm which)
{
    switch (which)
    {
    case pwm_tim3_ch4: return &get_tim3_ch4();
    default:           return nullptr;
    }
}

status_t bsp_pwm::init_all()
{
    return get_tim3_ch4().init();   // 触发构造 + 启动蜂鸣器 PWM（默认 duty=0 静音）
}

} // namespace pyro
```

> 新增 PWM 实例步骤：① `which_pwm` 增加枚举值 → ② 新增 `get_xxx()` 局部静态单例 → ③ `get_pwm()` 的 switch 增加分支 → ④ `init_all()` 补一行。句柄/通道/计数时钟只在 BSP 层集中维护。

### 3.3 `buzzer_player_t`（Component，播放逻辑层，含后台任务）

```cpp
enum class note_t : uint8_t { C, CS, D, DS, E, F, FS, G, GS, A, AS, B };

struct rhythm_step_t { note_t note; uint8_t octave; uint16_t beats; bool rest; };

class buzzer_player_t {
public:
    static buzzer_player_t& get_instance();       // 单例入口（Meyers 局部静态）

    buzzer_player_t(const buzzer_player_t&)            = delete;   // 禁止拷贝
    buzzer_player_t& operator=(const buzzer_player_t&) = delete;

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
    buzzer_player_t();                  // 私有构造：内部绑定 bsp_pwm::get_tim3_ch4()
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

#### 3.3.1 实现细节（pyro_buzzer_player.cpp 草案）

**① 音符频率（12-TET，A4 = 440Hz）**

```cpp
// note_t 序号：C=0, C#=1, ..., A=9, ..., B=11
uint32_t buzzer_player_t::note_to_freq(note_t note, uint8_t octave)
{
    const int32_t semi = (int32_t)note - 9 + (int32_t)(octave - 4) * 12; // 相对 A4 的半音数
    return (uint32_t)std::lround(440.0 * std::pow(2.0, semi / 12.0));
}
```

> 预定义常量用上式计算后写死为 `constexpr`（避免运行期浮点）：`NOTE_C4=262, NOTE_CS4=277, NOTE_D4=294, ...`；如需全音域可生成 C3~B6 表。

**② 命令队列与数据结构**

```cpp
// —— 以下为业务常量，定义于 pyro_buzzer_player.h（类内 static constexpr）——
static constexpr uint32_t PLAYER_MAX_MELODY_LEN = 32;   // 异步旋律最大音符数
static constexpr uint32_t PLAYER_QUEUE_LEN      = 4;    // 命令队列深度（FreeRTOS heap）
static constexpr uint16_t PLAYER_DEFAULT_FREQ   = 4000; // beep 默认频率 Hz

enum class player_cmd_t : uint8_t
{
    PLAY_TONE,     // 单音：{note=频率, duration_ms}
    PLAY_MELODY,   // 旋律：内部已拷贝 notes/durations
    PLAY_RHYTHM,   // 节奏型：内部已拷贝
    PAUSE, RESUME, STOP, SET_LOOP,
};

struct player_cmd_msg_t
{
    player_cmd_t type;
    uint16_t     note;          // 频率 Hz（PLAY_TONE）
    uint16_t     duration_ms;   // 单音时长 / 复用为旋律长度
    uint8_t      loop;          // SET_LOOP: 0/1
    uint8_t      reserved;
    uint16_t     notes[PLAYER_MAX_MELODY_LEN];
    uint16_t     durations[PLAYER_MAX_MELODY_LEN]; // ≈ 8 + 64 + 64 = 136B
};
```

> **生命周期安全**：`play_melody_async`/`play_rhythm` 会把调用方数组**拷贝进命令消息**再入队，后台任务不持有调用方指针，避免调用方栈数组在播放中失效；长度超 `PLAYER_MAX_MELODY_LEN` 返回 `PYRO_PARAM_ERROR`。

**③ 后台任务（player_task_t）**

```cpp
class player_task_t : public pyro::task_base_t
{
  public:
    player_task_t(buzzer_player_t *owner)
        : task_base_t("BuzzerPlayer", 256, 256, priority_t::LOW), _owner(owner) {}

    status_t post(const player_cmd_msg_t &cmd)   // 非阻塞入队
    {
        return (xQueueSend(_cmd_queue, &cmd, 0) == pdPASS) ? PYRO_OK : PYRO_BUSY;
    }

  protected:
    status_t init() override
    {
        _cmd_queue = xQueueCreate(PLAYER_QUEUE_LEN, sizeof(player_cmd_msg_t));
        return _cmd_queue ? PYRO_OK : PYRO_NO_MEMORY;
    }

    void run_loop() override   // 核心：20ms 轮询队列 + 逐音符推进
    {
        player_cmd_msg_t cmd;
        while (1)
        {
            if (_owner->_state.paused)            // 暂停态仍消费队列，响应 RESUME/STOP
            {
                if (xQueueReceive(_cmd_queue, &cmd, 20) == pdPASS)
                    _owner->handle_command(cmd);
                continue;
            }
            if (xQueueReceive(_cmd_queue, &cmd, 20) == pdPASS)
                _owner->handle_command(cmd);
            _owner->update_playback();            // 音符切换 / 循环 / 停止
        }
    }

  private:
    QueueHandle_t _cmd_queue = nullptr;
    buzzer_player_t *_owner;
};
```

> 轮询周期 20ms 决定音符切换精度（±20ms），对蜂鸣器旋律足够；需要更精确时可调小轮询周期。

**④ 播放状态（buzzer_player_t 私有成员）**

```cpp
struct play_state_t
{
    bool     playing;             // 有活动播放任务
    bool     paused;
    bool     loop;
    uint8_t  idx;                 // 当前音符下标
    uint32_t note_start_tick;     // 当前音符开始 tick
    uint16_t notes[PLAYER_MAX_MELODY_LEN];    // 内部拷贝（异步接口填充）
    uint16_t durations[PLAYER_MAX_MELODY_LEN];
    uint32_t len;
} _state;
// 其他成员：pwm_drv_t &_pwm; player_task_t _task; uint8_t _volume=50, _muted=false;
//          uint16_t _tempo_bpm=120, _beat_ms=125;   // 120BPM → 四分音符 125ms
```

**⑤ 命令处理与逐音符推进**

```cpp
void buzzer_player_t::handle_command(const player_cmd_msg_t &cmd)
{
    switch (cmd.type)
    {
    case player_cmd_t::PLAY_TONE:
        _state.loop = false;
        _state.len = 1;
        _state.notes[0] = cmd.note; _state.durations[0] = cmd.duration_ms;
        _state.playing = true; _state.idx = 0;
        _state.note_start_tick = xTaskGetTickCount();
        _pwm.set_frequency(_state.notes[0]); _pwm.start();
        break;
    case player_cmd_t::PLAY_MELODY:
    case player_cmd_t::PLAY_RHYTHM:
        memcpy(_state.notes, cmd.notes, sizeof(_state.notes));
        memcpy(_state.durations, cmd.durations, sizeof(_state.durations));
        _state.len = cmd.duration_ms;          // 复用字段存长度
        _state.playing = true; _state.idx = 0;
        _state.note_start_tick = xTaskGetTickCount();
        _pwm.set_frequency(_state.notes[0]); _pwm.start();
        break;
    case player_cmd_t::STOP:     _state.playing = false; _pwm.stop(); break;
    case player_cmd_t::SET_LOOP: _state.loop = cmd.loop; break;
    default: break;                            // PAUSE/RESUME 只置标志
    }
}

void buzzer_player_t::update_playback()
{
    if (!_state.playing) return;
    const uint32_t now = xTaskGetTickCount();
    if (now - _state.note_start_tick < _state.durations[_state.idx]) return;

    _pwm.stop();                                 // 当前音符结束
    if (++_state.idx >= _state.len)
    {
        if (_state.loop) _state.idx = 0;         // 循环重头播放
        else             { _state.playing = false; return; }
    }
    _pwm.set_frequency(_state.notes[_state.idx]);
    _pwm.start();
    _state.note_start_tick = now;
}
```

**⑥ 各方法实现逻辑**

```cpp
// 阻塞：调用线程内完成（beep 默认 4000Hz）
status_t buzzer_player_t::beep(uint32_t frequency, uint32_t duration_ms)
{
    _pwm.set_frequency(frequency); _pwm.start();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    _pwm.stop();
    return PYRO_OK;
}
status_t buzzer_player_t::beep(uint32_t duration_ms) { return beep(PLAYER_DEFAULT_FREQ, duration_ms); }
status_t buzzer_player_t::play_note(uint16_t f, uint32_t ms) { return beep(f, ms); }
status_t buzzer_player_t::play_note(note_t n, uint8_t o, uint32_t ms)
{ return beep(note_to_freq(n, o), ms); }

status_t buzzer_player_t::play_melody_blocking(const uint16_t notes[], const uint32_t durs[], uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) beep(notes[i], durs[i]);
    return PYRO_OK;
}
status_t buzzer_player_t::play_melody(...) { return play_melody_blocking(notes, durs, len); }

// 非阻塞：拷贝 + 入队，立即返回
status_t buzzer_player_t::play_melody_async(const uint16_t notes[], const uint32_t durs[], uint32_t len)
{
    if (len > PLAYER_MAX_MELODY_LEN) return PYRO_PARAM_ERROR;
    player_cmd_msg_t cmd = {};
    cmd.type = player_cmd_t::PLAY_MELODY;
    cmd.duration_ms = (uint16_t)len;             // 复用字段存长度
    for (uint32_t i = 0; i < len; i++)
    {
        cmd.notes[i]     = notes[i];
        cmd.durations[i] = (uint16_t)durs[i];
    }
    return _task.post(cmd);
}

status_t buzzer_player_t::play_rhythm(const rhythm_step_t pattern[], uint32_t len)
{
    if (len > PLAYER_MAX_MELODY_LEN) return PYRO_PARAM_ERROR;
    player_cmd_msg_t cmd = {};
    cmd.type = player_cmd_t::PLAY_RHYTHM;
    cmd.duration_ms = (uint16_t)len;
    for (uint32_t i = 0; i < len; i++)
    {
        cmd.notes[i]     = pattern[i].rest ? 0u : note_to_freq(pattern[i].note, pattern[i].octave);
        cmd.durations[i] = (uint16_t)(pattern[i].beats * _beat_ms);   // beats → ms
    }
    return _task.post(cmd);
}

// 播放控制：只置标志 + 入队唤醒，不改 _state 播放数据
status_t buzzer_player_t::pause()  { _state.paused = true;  player_cmd_msg_t c{}; c.type = player_cmd_t::PAUSE;  return _task.post(c); }
status_t buzzer_player_t::resume() { _state.paused = false; player_cmd_msg_t c{}; c.type = player_cmd_t::RESUME; return _task.post(c); }
status_t buzzer_player_t::stop()   { player_cmd_msg_t c{}; c.type = player_cmd_t::STOP; return _task.post(c); }
void buzzer_player_t::set_loop(bool en)
{ player_cmd_msg_t c{}; c.type = player_cmd_t::SET_LOOP; c.loop = en ? 1u : 0u; _task.post(c); }

// 速度：四分音符时长 = 60s / bpm
void buzzer_player_t::set_tempo(uint16_t bpm)
{
    _tempo_bpm = (bpm == 0u) ? 120u : bpm;
    _beat_ms   = 60000u / _tempo_bpm / 4u;
}
uint16_t buzzer_player_t::get_tempo() const { return _tempo_bpm; }

// 音量/静音
status_t buzzer_player_t::set_volume(uint8_t percent)
{
    if (percent > 100u) return PYRO_PARAM_ERROR;
    _volume = percent;
    return _muted ? PYRO_OK : _pwm.set_duty_cycle(_volume);  // 静音中不改输出
}
uint8_t  buzzer_player_t::get_volume() const { return _volume; }
status_t buzzer_player_t::mute()   { _muted = true;  return _pwm.set_duty_cycle(0u); }
status_t buzzer_player_t::unmute() { _muted = false; return _pwm.set_duty_cycle(_volume); }
bool     buzzer_player_t::is_playing() const { return _state.playing; }

// 单例入口：Meyers 局部静态，C++11 起线程安全
buzzer_player_t &buzzer_player_t::get_instance()
{
    static buzzer_player_t instance;
    return instance;
}

// 私有构造函数：自给自足，内部绑定 PWM 单例并启动后台任务
buzzer_player_t::buzzer_player_t()
    : _pwm(bsp_pwm::get_tim3_ch4()), _task(this)
{
    _task.start();
}
```

---

## 4. 关键实现要点

1. **频率计算**：TIM3 定时器时钟 = 240MHz（APB1×2），`Prescaler=24-1` → 计数时钟 **10MHz**。
   `pwm_drv_t::set_frequency(hz)` 里 `ARR = 10'000'000 / hz - 1`，`CCR = ARR * duty / 100`（duty 由调用方设置，蜂鸣器推荐 50%）。
   当前 CubeMX 的 `Period=65535` 对应约 152Hz，仅作初始值。

2. **音符频率**：`note_to_freq(n, octave)` 用十二平均律 `440 * 2^((n - 9)/12 + (octave-4))`，整数化即可。

3. **异步播放（`play_melody_async`/`beep` 非阻塞）**：`player_task_t` 继承 `pyro::task_base_t`（`init()` + `run_loop()`），
   内部用 FreeRTOS `QueueHandle_t` 接收命令（`PLAY_TONE / PLAY_MELODY / PLAY_RHYTHM / PAUSE / RESUME / STOP / SET_LOOP`）。
   `run_loop` 以 20ms 轮询队列并逐音符推进（非阻塞）：`_pwm.set_frequency` + `_pwm.start`，到时 `_pwm.stop` 播下一音符。
   `pause()/resume()` 通过「置位 paused 标志 + 入队唤醒」实现，暂停态任务仍轮询队列以响应 `RESUME/STOP`。

4. **阻塞接口**：`beep` 与 `play_melody`/`play_melody_blocking` 直接在调用线程里逐音符延时，不经过后台任务。

5. **音量/静音**：`set_volume` 映射到 `set_duty_cycle`；`mute()` 记下当前音量并置 duty=0，`unmute()` 恢复。
   ⚠️ 被动蜂鸣器靠谐振，占空比对音量的影响有限且非线性的，这是硬件物理限制，会在文档里注明。

6. **并发与生命周期**：
   - 异步接口必须**内部拷贝**音符数组（上限 `PLAYER_MAX_MELODY_LEN`），防止调用方栈数组在播放中失效。
   - `_state` 主要被后台任务独占访问；调用线程仅通过队列下发命令 + 原子置位 `paused` 标志（bool 写具备原子性）。
   - `pwm_drv_t` 可能被「阻塞接口调用线程」与「后台任务」同时访问；约定同一时刻只用一种模式（阻塞 or 异步），或后续在驱动内加锁。
   - 后台任务栈 256 字足够（无大数组）；命令队列 4 × 136B ≈ 544B 分配于 FreeRTOS 堆。
   - `task_base_t::start()` 为双阶段创建；构造函数内 `start()` 后立即 `play_melody_async` 安全（命令先入队，任务创建后按序消费）。

7. **常量定义位置约定**：
   - **硬件相关**（定时器计数时钟、句柄、通道）→ 只在 **BSP 层**（`pyro_bsp_pwm.h/.cpp`）维护，驱动核心构造时传入、不写死。
   - 定时器计数时钟默认值用 BSP 内 `constexpr`；跨机器人可调时提为宏放 `pyro_core_config.h`（如 `PYRO_PWM_TIM3_CLK_HZ`）。
   - **业务相关**（`PLAYER_MAX_MELODY_LEN`、`PLAYER_QUEUE_LEN`、`PLAYER_DEFAULT_FREQ`、`NOTE_*` 音符表）→ 定义于 `pyro_buzzer_player.h`（类内 `static constexpr`）。

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