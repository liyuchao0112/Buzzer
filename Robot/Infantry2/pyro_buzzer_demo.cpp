/**
 * @file pyro_buzzer_demo.cpp
 * @brief 蜂鸣器（被动蜂鸣器，PWM 变频）使用例程 —— Infantry2
 *
 * 演示 buzzer_drv_t 的常用接口：
 *   - 基础鸣叫      beep()
 *   - 音符          play_note() / note_to_freq()
 *   - 旋律          play_melody() / play_melody_async() / play_melody_blocking()
 *   - 节奏型        play_rhythm() + set_tempo()
 *   - RTTTL 铃声    play_rtttl()
 *   - 控制          stop/pause/resume/set_loop、音量 set_volume/mute/unmute
 *
 * 实现为一个独立 FreeRTOS 任务（task_base_t），循环播放一组示例便于实机验证。
 * 调用 buzzer_demo_start() 即可启动；如需改为遥控器/按键/事件触发，
 * 可直接调用 pyro_buzzer_demo_play_once()，无需常驻演示任务。
 */
#include "pyro_buzzer_drv.h"
#include "pyro_task.h"

#include "FreeRTOS.h"
#include "task.h"

namespace pyro {

// ========================= 单次完整演示（阻塞/非阻塞混合） =========================
// 除蜂鸣器自身播放任务上下文外，任意线程均可安全调用。
void pyro_buzzer_demo_play_once()
{
    auto& bz = pyro::buzzer_drv_t::get_instance();

    // 1) 基础鸣叫（阻塞）：4000Hz，200ms
    bz.beep(4000, 200);

    // 2) 单音符（阻塞）：C 大调和弦琶音 C4 -> E4 -> G4
    bz.play_note(pyro::note_t::C, 4, 300);
    bz.play_note(pyro::note_t::E, 4, 300);
    bz.play_note(pyro::note_t::G, 4, 300);

    // 3) 旋律（阻塞）："小星星" 前奏
    const uint16_t notes[] = { NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4,
                               NOTE_A4, NOTE_A4, NOTE_G4 };
    const uint32_t durs[]  = { 300, 300, 300, 300, 300, 300, 600 };
    bz.play_melody(notes, durs, 7);

    // 4) 异步旋律（非阻塞，立即返回，后台播放；新播放覆盖旧播放）
    const uint16_t an[] = { NOTE_E4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_G4 };
    const uint32_t ad[] = { 200, 200, 200, 200, 400 };
    bz.play_melody_async(an, ad, 5);

    // 5) 节奏型（阻塞）：配合 tempo 控制拍长
    bz.set_tempo(120);
    const rhythm_step_t pat[] = {
        { note_t::C, 4, 1, false },
        { note_t::E, 4, 1, false },
        { note_t::G, 4, 1, false },
        { note_t::C, 5, 2, false },
    };
    bz.play_rhythm(pat, 4);

    // 6) RTTTL 铃声（非阻塞，解析后异步播放）
    bz.play_rtttl(false, "Intro:d=4,o=5,b=160:e,e,e,2e,2g,2e,2g");

    // 7) 控制接口示例：音量 / 静音 / 取消 / 循环
    bz.set_volume(80);
    bz.mute();     vTaskDelay(pdMS_TO_TICKS(100));
    bz.unmute();   vTaskDelay(pdMS_TO_TICKS(100));
    bz.set_volume(50);
    bz.set_loop(false);   // 确认非循环模式，避免阻塞接口被循环卡住
}

void play() {
    auto& bz = pyro::buzzer_drv_t::get_instance();
    
    bz.set_volume(5);
    // bz.play_rtttl(true, "CANON:b=170,o=6,d=4:g,64p,8e,8f,g,64p,8e,8f,8g,8g5,8a5,8b5,8c,8d,8e,8f,e,64p,8c,8d,e,64p,8e5,8f5,8g5,8a5,8g5,8f5,8g5,8c,8b5,c");
}

// ========================= 常驻演示任务 =========================
class buzzer_demo_task_t final : public pyro::task_base_t
{
  public:
    buzzer_demo_task_t()
        : task_base_t("buzzer_demo", 256, 512, priority_t::NORMAL) {}

  protected:
    status_t init() override { return PYRO_OK; }

    void run_loop() override
    {
        for (;;) {
            play();
            vTaskDelay(pdMS_TO_TICKS(10000));   // 播完休息 1s 再循环
        }
    }
};

// ========================= 对外启动入口 =========================
// 在初始化阶段（如 pyro_init_thread）调用一次即可启动演示。
void buzzer_demo_start()
{
    static buzzer_demo_task_t demo_task;
    demo_task.start();
}

} // namespace pyro
