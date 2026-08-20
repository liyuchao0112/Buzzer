# 蜂鸣器驱动 · 架构设计方案

> 项目：RM2027（STM32H723 + FreeRTOS + 自研 PYRo 框架）
> 定位：被动蜂鸣器（PWM 变频发声）驱动，复用通用 PWM 库 `pwm_drv_t`。

## 0. 背景与现状

- 蜂鸣器为**被动蜂鸣器**，由 PWM 变频驱动发声。
- 现有 PWM 库已实现（`PYRo/Peripheral/PWM/`）：
  - `pwm_drv_t`（`pyro_pwm_drv`）：通用 PWM 驱动，`TIM_CLK_HZ = 240MHz`，`PSC=60`，
    接口 `init/deinit/start/stop`、`set_frequency`、`set_duty_cycle(float 0~1)`、`get_*`。
  - `bsp_pwm`（`pyro_bsp_pwm`）：`which_pwm { pwm_tim12_ch2 }`，`get_pwm()` / `init_all()`，
    绑定 `htim12` / `TIM_CHANNEL_2`。
- 蜂鸣器组件 `buzzer_drv_t` 位于 `PYRo/Component/Buzzer/pyro_buzzer_drv`，复用 `pwm_drv_t`。
- 项目分层：`CubeMX/`（HAL 生成）、`PYRo/Peripheral/`（外设驱动）、`PYRo/Component/`（业务组件）、`PYRo/Core/`（任务/锁/通用类型）。

## 1. 总体架构：两层

| 层 | 职责 | 文件 |
|---|---|---|
| **Peripheral/PWM** | 通用 PWM 驱动：频率、占空比、启停 | `pyro_pwm_drv` + `pyro_bsp_pwm` |
| **Component/Buzzer** | 播放逻辑：音符/旋律/节奏/异步/控制 | `pyro_buzzer_drv` |

硬件绑定隔离在 BSP 层，播放逻辑复用通用 PWM 驱动，可替换硬件。

## 2. 文件布局

```
PYRo/Peripheral/PWM/pyro_pwm_drv.{h,cpp}      # pwm_drv_t（已实现）
PYRo/Peripheral/PWM/pyro_bsp_pwm.{h,cpp}      # bsp_pwm（已实现）
PYRo/Component/Buzzer/pyro_buzzer_drv.{h,cpp} # buzzer_drv_t（当前实现）
PYRo/CMakeLists.txt                            # 已包含上述 .cpp 与 include 路径
```

## 3. 类设计

### 3.1 `pwm_drv_t`（Peripheral/PWM，已实现，复用）

通用 PWM 驱动，与具体设备解耦。核心接口：

```cpp
class pwm_drv_t {
    friend class bsp_pwm;                 // 单例：仅 BSP 可创建
  public:
    status_t init();   status_t deinit();   // PWM 使能/失能
    status_t start();  status_t stop();     // 输出启停
    bool is_running() const;
    status_t set_frequency(uint32_t f);     // 频率型：改 ARR
    uint32_t get_frequency() const;
    status_t set_duty_cycle(float duty);    // 0~1，改 CCR
    float get_duty_cycle() const;
  private:
    explicit pwm_drv_t(TIM_HandleTypeDef* htim, uint32_t channel);  // 私有
    status_t update();
    TIM_HandleTypeDef* _htim; uint32_t _channel;
    uint32_t _psc{60}, _arr{1000}, _cmp{500};
    bool _running; uint32_t _freq{4000}; float _duty{0.5f};
};
```

> 频率换算：`ARR = TIM_CLK_HZ(240MHz) / PSC(60) / f - 1`；`CMP = duty * ARR`。详见 `PYRo/Peripheral/PWM/pyro_pwm_drv.cpp`。


### 3.2 `bsp_pwm`（Peripheral/PWM，已实现，复用）

```cpp
class bsp_pwm {
  public:
    enum which_pwm {
        pwm_tim12_ch2,                 // 蜂鸣器（TIM12_CH2）
    };
    static pwm_drv_t* get_pwm(which_pwm which);
    static status_t init_all();
};
```

> 绑定 `htim12` / `TIM_CHANNEL_2`，经 `get_pwm(pwm_tim12_ch2)` 获取单例。详见 `PYRo/Peripheral/PWM/pyro_bsp_pwm.cpp`。


### 3.3 `buzzer_drv_t`（Component/Buzzer，当前实现）

单例 + **无队列**后台播放（新播放覆盖式、暂停静音）。公开接口：

```cpp
class buzzer_drv_t {
  public:
    static buzzer_drv_t& get_instance();   // 单例

    // 阻塞：提交并等待播放完成
    status_t beep(uint32_t frequency, uint32_t duration_ms);
    status_t beep(uint32_t duration_ms);              // 默认 4000Hz
    status_t play_note(uint16_t frequency, uint32_t duration_ms);
    status_t play_note(note_t note, uint8_t octave, uint32_t duration_ms);
    status_t play_melody(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_melody_blocking(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_rhythm(const rhythm_step_t pattern[], uint32_t len);
    // 非阻塞
    status_t play_melody_async(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_rtttl(const char* rtttl);        // RTTTL 铃声（非阻塞）
    // 控制
    status_t start();  status_t stop();  bool is_playing() const;
    status_t pause();  status_t resume();  void set_loop(bool);
    // 音量/速度
    status_t set_volume(uint8_t percent); uint8_t get_volume() const;
    status_t mute(); status_t unmute();
    void set_tempo(uint16_t bpm); uint16_t get_tempo() const;
    // 工具
    static uint32_t note_to_freq(note_t note, uint8_t octave);
};
```

音符/节奏类型（定义于 `pyro_buzzer_drv.h`）：

```cpp
enum class note_t : uint8_t { C, CS, D, DS, E, F, FS, G, GS, A, AS, B };
struct rhythm_step_t { note_t note; uint8_t octave; uint16_t beats; bool rest; };
constexpr uint16_t NOTE_C4=262, NOTE_D4=294, NOTE_E4=330, NOTE_F4=349,
                   NOTE_G4=392, NOTE_A4=440, NOTE_B4=494;
```

> 私有成员：`pwm_drv_t* _pwm`（经 `bsp_pwm::get_pwm(pwm_tim12_ch2)`）、`player_task_t _task`（后台任务）、`pyro::mutex_t _mutex`、输入区 `_state`、运行区 `_run_*`、阻塞等待者 `_waiter{task,sem}`（信号量）。详见 `PYRo/Component/Buzzer/pyro_buzzer_drv.{h,cpp}`。


#### 3.3.1 无队列核心机制

**① 提交播放（覆盖式）**

```cpp
// 加锁写入待播内容（覆盖式），若已有等待者则定向唤醒它；锁随函数栈帧自动释放
status_t buzzer_drv_t::store_play(const uint16_t notes[], const uint32_t durations[], uint32_t len) {
    pyro::scoped_mutex_t lock(_mutex);
    if (_waiter.sem != nullptr) {            // 有旧等待者被覆盖 → 定向唤醒它
        xSemaphoreGive(_waiter.sem);
        _waiter.task = nullptr;
        _waiter.sem  = nullptr;
    }
    _state.pending = true;                   // 覆盖：最新内容直接写入
    _state.len = len;
    for (uint32_t i = 0; i < len; i++) {
        _state.notes[i] = notes[i];
        _state.durations[i] = (uint16_t)durations[i];
    }
    return PYRO_OK;
}

status_t buzzer_drv_t::submit_play(const uint16_t notes[], const uint32_t durations[],
                                   uint32_t len, bool wait_complete) {
    if (len > BUZZER_MAX_MELODY_LEN) return PYRO_PARAM_ERROR;
    if (_task.is_self()) {                    // 后台任务内调用：直接同步播，防死锁
        for (uint32_t i = 0; i < len; i++) {
            if (notes[i] != 0u) { _pwm->set_frequency(notes[i]); _pwm->start(); }
            vTaskDelay(pdMS_TO_TICKS(durations[i]));
            _pwm->stop();
        }
        return PYRO_OK;
    }
    const status_t r = store_play(notes, durations, len);  // 加锁写入（覆盖），并释放被覆盖的等待者
    if (r != PYRO_OK) return r;
    _task.notify();                           // 任务通知唤醒后台
    if (wait_complete) {                      // 阻塞：用专用信号量等"播完或被覆盖"
        SemaphoreHandle_t my_sem = xSemaphoreCreateBinary();
        if (my_sem == nullptr) return PYRO_NO_MEMORY;
        { pyro::scoped_mutex_t lock(_mutex);
          _waiter.task = xTaskGetCurrentTaskHandle();
          _waiter.sem  = my_sem; }
        xSemaphoreTake(my_sem, portMAX_DELAY);
        xSemaphoreDelete(my_sem);
    }
    return PYRO_OK;
}
```

**② 后台任务 run_loop（暂停静音 + 停止 + 推进）**

```cpp
void buzzer_drv_t::player_task_t::run_loop() {
    while (1) {
        if (_owner->_state.paused) {          // 暂停：完全静音，阻塞等恢复
            _owner->_pwm->stop();
            _owner->_playing = false;
            _owner->_need_start = true;
            if (_owner->_state.need_stop) {   // 暂停中也能响应 stop
                _owner->_state.need_stop = false;
                _owner->notify_done();        // 停止：定向唤醒等待者
            }
            xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
            continue;
        }
        if (_owner->_state.need_stop) {       // 停止请求
            _owner->_state.need_stop = false;
            _owner->_pwm->stop(); _owner->_playing = false; _owner->notify_done();
        }
        _owner->check_pending();              // 有最新内容则覆盖播放
        _owner->update_playback();            // 逐音符推进
        xTaskNotifyWait(0, 0, nullptr, 20);   // 等通知或 20ms 超时
    }
}
```

> 轮询周期 20ms 决定音符切换精度（±20ms）；任务通知可在 20ms 内即时响应新播放/控制。

**③ 检查待播内容（覆盖）与逐音符推进**

```cpp
// 加锁读取待播内容；有则拷入 _run_* 并返回 true（锁随函数栈帧自动释放）
bool buzzer_drv_t::take_pending() {
    pyro::scoped_mutex_t lock(_mutex);
    if (!_state.pending) return false;
    _state.pending = false;
    _run_len = _state.len;
    memcpy(_run_notes, _state.notes, sizeof(_run_notes));
    memcpy(_run_durations, _state.durations, sizeof(_run_durations));
    return true;
}

void buzzer_drv_t::check_pending() {
    if (!take_pending()) return;             // 无新内容
    _idx = 0; _playing = true; _need_start = true;   // 新内容覆盖当前
    _pwm->stop();                                    // 停旧音
}

void buzzer_drv_t::update_playback() {
    if (!_playing) return;
    const uint32_t now = xTaskGetTickCount();
    if (_need_start) {                         // 新播放/暂停恢复：启动当前音符
        if (_run_notes[_idx] != 0u) { _pwm->set_frequency(_run_notes[_idx]); _pwm->start(); }
        _note_start_tick = now; _need_start = false; return;
    }
    if (now - _note_start_tick < _run_durations[_idx]) return;
    _pwm->stop();
    if (++_idx >= _run_len) {
        if (_state.loop) _idx = 0;             // 循环重头播放
        else { _playing = false; notify_done(); return; }
    }
    if (_run_notes[_idx] != 0u) { _pwm->set_frequency(_run_notes[_idx]); _pwm->start(); }
    _note_start_tick = now;
}
```

**④ RTTTL 铃声解析（play_rtttl）**

格式：`名称:d=默认时值,o=默认八度,b=BPM:音符序列`

1. **名称**：第一个 `:` 前（忽略）
2. **控制区**：`d=` 默认时值、`o=` 默认八度、`b=` BPM
3. **音符序列**：逗号分隔的 token

音符 token：`[时值][音名][#][.]`

| 部分 | 含义 |
|---|---|
| 时值（可选数字） | 1=全音符…32=三十二分，缺省用控制区 `d` |
| 音名 | `C D E F G A B` 或 `P`（休止符），可带 `#` 升半音 |
| `.`（可选） | 附点，时值 ×1.5 |

**时值换算**

```
一拍   = 60000 / bpm  ms
音符 X = 一拍 × 4 / X          （4=四分→1拍，8=八分→0.5拍）
附点   = 音符 × 3/2
```

**频率**：`note_to_freq(rtttl_note(音名, #), o)`；休止 `P` → freq=0（静音计时）。

**示例**：`"Jinglebells:d=4,o=5,b=140:e,e,e,2e,e,e,e,2g"`
- `e`  → E5，默认时值 4 → 1 拍 ≈ 428.6ms
- `2e` → E5，二分音符 → ≈ 857ms
- `2g` → G5，二分音符 → ≈ 857ms

实现：解析整段到 `freq[]/dur[]` → `submit_play(..., false)` 非阻塞提交。





---

## 4. 关键实现要点

1. **无队列覆盖播放**：播放请求用「最新值槽 + 任务通知」，新请求直接覆盖当前内容并从头播，无积压、无命令序号。
2. **暂停完全静音**：后台任务暂停分支立即 `_pwm->stop()` 并阻塞等恢复；暂停中收到播放只写 `_state` 不发声，恢复后 `_need_start` 重启当前音符。
3. **完成等待（信号量，非轮询）**：阻塞接口建专用二进制信号量并登记 `_waiter`；后台播完或被覆盖时 `notify_done()` 定向 `xSemaphoreGive`，等待者 `xSemaphoreTake` 阻塞。被覆盖者**立即返回**，无轮询计数、无任务通知串扰（信号量是独立对象）。
4. **线程安全**：输入区 `_state`（notes/durations/len/pending）用 `pyro::mutex_t` 保护；控制标志（paused/loop/need_stop）用 `volatile`（单字节原子）；运行区 `_run_*`/`_playing` 后台任务独占。
5. **PWM 唯一写者**：所有 `_pwm->` 操作只在后台任务上下文，调用线程仅写 `_state` + 任务通知，无并发写冲突。
6. **自锁防护**：后台任务内调用阻塞接口时 `_task.is_self()` 检测 → 直接同步播放，避免等自己完成而死锁。
7. **音符频率**：`note_to_freq` 十二平均律（A4=440）；阻塞接口=提交+等待，非阻塞 async=提交即返回。

---

## 5. 使用示例

```cpp
#include "pyro_buzzer_drv.h"

auto &bz = pyro::buzzer_drv_t::get_instance();

bz.beep(4000, 200);                             // 阻塞鸣叫 4000Hz 200ms
bz.play_note(pyro::note_t::C, 4, 300);          // 阻塞播 C4 300ms

uint16_t notes[] = { NOTE_C4, NOTE_E4, NOTE_G4 };
uint32_t durs[]  = { 300, 300, 300 };
bz.play_melody_async(notes, durs, 3);            // 异步播旋律（新播放覆盖旧）

bz.set_tempo(120);
pyro::rhythm_step_t pat[] = {
    { pyro::note_t::C, 4, 1, false },
    { pyro::note_t::C, 4, 1, false },
    { pyro::note_t::G, 4, 2, false },
};
bz.play_rhythm(pat, 3);                          // 阻塞播节奏型

bz.play_rtttl("Jinglebells:d=4,o=5,b=140:e,e,e,2e,e,e,e,2g");  // RTTTL 铃声（非阻塞）

bz.pause(); bz.resume(); bz.stop(); bz.set_loop(true);
```

---

## 6. 构建集成

- `PYRo/CMakeLists.txt` 已包含 `Peripheral/PWM/pyro_pwm_drv.cpp`、`Peripheral/PWM/pyro_bsp_pwm.cpp`、`Component/Buzzer/pyro_buzzer_drv.cpp`，以及 `Peripheral/PWM`、`Component/Buzzer` 的 include 路径。
- 无需额外配置即可编译。
