// Robot do line PID voi cam bien 6 kenh (analog hoac digital) + driver motor bat ky (motor_driver.h).
//  - Chan co ADC1 (GPIO1-10): doc analog (on dinh khi bat Wi-Fi)
//  - Chan ADC2 (GPIO11-20): doc analog, khi Wi-Fi chiem ADC2 thi giu gia tri cu
//  - Chan khong co ADC (21, 38, 41, 42, 47, 48...): doc digital 0/1 (dung chan DO cua module)
//  - Hieu chinh (quay tai cho quet vach) luu NVS, nap lai khi khoi dong
//  - Lenh giong noi: do line, dung do line, hieu chinh cam bien, toc do do line, chinh PID, doc cam bien
#ifndef _MULTI_LINE_FOLLOWER_H_
#define _MULTI_LINE_FOLLOWER_H_

#include "line_pid.h"
#include "motor_driver.h"
#include "rb_robot.h"
#include "mcp_server.h"

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>

#include <atomic>
#include <cstdlib>
#include <string>

struct LineConfig {
    int pins[line::kMaxSensors] = {-1, -1, -1, -1, -1, -1};  // cam bien 0 = TRAI nhat
    line::Params params;
};

class LineFollower {
public:
    LineFollower(MotorDriver* motor, RbRobot* robot, const LineConfig& cfg)
        : motor_(motor), robot_(robot), cfg_(cfg) {
        InitSensors();
        LoadCalibration();
        if (robot_) robot_->SetStopHook([this]() { Stop(); });
        AddTools();
    }

    // Hieu chinh ngay (chan ~3 s): quay tai cho de quet vach
    std::string CalibrateNow() { return Calibrate(); }

    int SensorCount() const { return n_; }
    bool IsRunning() const { return running_; }

    void Start() {
        if (running_ || n_ < 2) return;
        if (robot_) robot_->SetExternalControl(true);
        running_ = true;
        xTaskCreate([](void* a) { static_cast<LineFollower*>(a)->Loop(); }, "line_pid", 4096, this, 5, &task_);
    }

    void Stop() {
        running_ = false;
        for (int i = 0; i < 40 && task_ != nullptr; i++) vTaskDelay(pdMS_TO_TICKS(5));
        motor_->Stop();
        if (robot_) robot_->SetExternalControl(false);
    }

    // Doc 6 cam bien (raw: analog 0..4095, digital 0/1)
    void ReadRaw(int* raw) {
        for (int i = 0; i < n_; i++) {
            Sensor& s = sensors_[i];
            if (s.adc) {
                int v = 0;
                if (adc_oneshot_read(s.adc, s.channel, &v) == ESP_OK) s.last = v;
                raw[i] = s.last;
            } else {
                raw[i] = gpio_get_level((gpio_num_t)s.pin);
            }
        }
    }

private:
    static constexpr const char* TAG = "LineFollower";
    static constexpr const char* kNvsNs = "line_cal";
    struct Sensor {
        int pin = -1;
        adc_oneshot_unit_handle_t adc = nullptr;
        adc_channel_t channel = ADC_CHANNEL_0;
        int last = 0;
    };

    MotorDriver* motor_;
    RbRobot* robot_;
    LineConfig cfg_;
    Sensor sensors_[line::kMaxSensors];
    int n_ = 0;
    line::Calib calib_;
    std::atomic<bool> running_{false};
    std::atomic<bool> calibrating_{false};
    TaskHandle_t task_ = nullptr;
    adc_oneshot_unit_handle_t units_[2] = {nullptr, nullptr};
    bool unit_failed_[2] = {false, false};

    adc_oneshot_unit_handle_t Unit(adc_unit_t u) {
        int i = u == ADC_UNIT_1 ? 0 : 1;
        if (units_[i] == nullptr && !unit_failed_[i]) {
            adc_oneshot_unit_init_cfg_t cfg = {};
            cfg.unit_id = u;
            cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
            if (adc_oneshot_new_unit(&cfg, &units_[i]) != ESP_OK) {
                ESP_LOGW(TAG, "ADC%d dang duoc dung boi thiet bi khac -> cam bien doc digital", i + 1);
                units_[i] = nullptr;
                unit_failed_[i] = true;
            }
        }
        return units_[i];
    }

    void InitSensors() {
        int analog = 0;
        for (int i = 0; i < line::kMaxSensors; i++) {
            int pin = cfg_.pins[i];
            if (pin < 0) continue;
            Sensor& s = sensors_[n_++];
            s.pin = pin;
            adc_unit_t unit;
            adc_channel_t ch;
            if (adc_oneshot_io_to_channel(pin, &unit, &ch) == ESP_OK) {
                auto h = Unit(unit);
                if (h != nullptr) {
                    adc_oneshot_chan_cfg_t cc = {};
                    cc.atten = ADC_ATTEN_DB_12;
                    cc.bitwidth = ADC_BITWIDTH_DEFAULT;
                    if (adc_oneshot_config_channel(h, ch, &cc) == ESP_OK) {
                        s.adc = h;
                        s.channel = ch;
                        analog++;
                        continue;
                    }
                }
            }
            gpio_config_t io = {};
            io.mode = GPIO_MODE_INPUT;
            io.pin_bit_mask = 1ULL << pin;
            gpio_config(&io);
        }
        ESP_LOGI(TAG, "%d cam bien line (%d analog, %d digital)", n_, analog, n_ - analog);
    }

    // Mac dinh truoc khi hieu chinh: analog 0..4095, digital 0..1
    void DefaultCalibration() {
        for (int i = 0; i < line::kMaxSensors; i++) {
            calib_.min[i] = 0;
            calib_.max[i] = (i < n_ && sensors_[i].adc) ? 4095 : 1;
        }
    }

    void LoadCalibration() {
        DefaultCalibration();
        nvs_handle_t h;
        if (nvs_open(kNvsNs, NVS_READONLY, &h) != ESP_OK) return;
        line::Calib c;
        size_t len = sizeof(c);
        if (nvs_get_blob(h, "calib", &c, &len) == ESP_OK && len == sizeof(c)) {
            // chi dung neu khop kieu cam bien (digital max = 1)
            bool ok = true;
            for (int i = 0; i < n_; i++) {
                bool digital = sensors_[i].adc == nullptr;
                if (digital != (c.max[i] <= 1)) ok = false;
            }
            if (ok) calib_ = c;
        }
        nvs_close(h);
    }

    void SaveCalibration() {
        nvs_handle_t h;
        if (nvs_open(kNvsNs, NVS_READWRITE, &h) != ESP_OK) return;
        nvs_set_blob(h, "calib", &calib_, sizeof(calib_));
        nvs_commit(h);
        nvs_close(h);
    }

    // Quay tai cho trai/phai ~3 s de quet vach, ghi min/max moi cam bien
    std::string Calibrate() {
        if (n_ < 2) return "Chua gan cam bien line";
        bool was_running = running_;
        if (was_running) Stop();
        if (robot_) robot_->SetExternalControl(true);
        calibrating_ = true;
        line::Calib c;
        c.Reset(1 << 30, -1);
        int raw[line::kMaxSensors];
        int64_t t0 = esp_timer_get_time();
        const int spd = 35;
        while (esp_timer_get_time() - t0 < 3200000) {
            int64_t ms = (esp_timer_get_time() - t0) / 1000;
            int phase = (ms / 400) % 4;  // trai, phai, phai, trai -> ve cho cu
            if (phase == 0 || phase == 3) motor_->SpeedRun(-spd, spd);
            else motor_->SpeedRun(spd, -spd);
            ReadRaw(raw);
            for (int i = 0; i < n_; i++) {
                c.min[i] = std::min(c.min[i], raw[i]);
                c.max[i] = std::max(c.max[i], raw[i]);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        motor_->Stop();
        calibrating_ = false;
        if (robot_) robot_->SetExternalControl(false);
        std::string bad;
        for (int i = 0; i < n_; i++) {
            int span = c.max[i] - c.min[i];
            bool digital = sensors_[i].adc == nullptr;
            if (span < (digital ? 1 : 300)) bad += " " + std::to_string(i);
        }
        if (!bad.empty()) {
            return "Hieu chinh chua dat: cam bien" + bad +
                   " khong thay doi. Dat robot len vach roi thu lai (kiem tra day, do cao cam bien)";
        }
        calib_ = c;
        SaveCalibration();
        return "Da hieu chinh xong " + std::to_string(n_) + " cam bien";
    }

    void Loop() {
        line::Pid pid;
        pid.Reset();
        int raw[line::kMaxSensors], val[line::kMaxSensors];
        int last = (n_ - 1) * 500;
        int64_t lost_since = 0;
        int prev_l = 0, prev_r = 0;
        const float center = (n_ - 1) * 500.0f;
        ESP_LOGI(TAG, "Bat dau do line: Kp %.4f Ki %.5f Kd %.3f, toc do %d%%", cfg_.params.kp, cfg_.params.ki,
                 cfg_.params.kd, cfg_.params.base_speed);
        TickType_t wake = xTaskGetTickCount();
        while (running_) {
            ReadRaw(raw);
            line::Normalize(raw, n_, calib_, cfg_.params.white_line, val);
            bool on_line = false;
            int pos = line::Position(val, n_, cfg_.params.threshold, last, on_line);
            int64_t now = esp_timer_get_time();
            if (on_line) {
                lost_since = 0;
            } else if (lost_since == 0) {
                lost_since = now;
            } else if (now - lost_since > 2000000) {
                ESP_LOGW(TAG, "Mat vach qua 2 s -> dung");
                break;
            }
            // goc vuong: quay tai cho 180 ms ve phia co vach
            int corner = cfg_.params.corners ? line::Corner(val, n_) : 0;
            if (corner != 0) {
                motor_->SpeedRun(corner * 47, -corner * 47);
                prev_l = corner * 47;
                prev_r = -corner * 47;
                vTaskDelay(pdMS_TO_TICKS(180));
                wake = xTaskGetTickCount();
                continue;
            }
            float corr = pid.Update(cfg_.params, pos - center);
            int l, r;
            line::Mix(cfg_.params, corr, l, r);
            l = line::Slew(l, prev_l, cfg_.params.slew);
            r = line::Slew(r, prev_r, cfg_.params.slew);
            prev_l = l;
            prev_r = r;
            motor_->SpeedRun(l, r);
            vTaskDelayUntil(&wake, std::max<TickType_t>(1, pdMS_TO_TICKS(10)));
        }
        motor_->Stop();
        running_ = false;
        if (robot_) robot_->SetExternalControl(false);
        task_ = nullptr;
        vTaskDelete(nullptr);
    }

    static bool ParseFloat(const std::string& s, float& out) {
        if (s.empty()) return false;
        char* end = nullptr;
        std::string t = s;
        for (auto& ch : t)
            if (ch == ',') ch = '.';
        float v = strtof(t.c_str(), &end);
        if (end == t.c_str()) return false;
        out = v;
        return true;
    }

    void AddTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.line.start",
                    "Bat che do do line / chay theo vach / bam vach / line follower. Robot tu chay theo vach den.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        if (n_ < 2) return std::string("Chua gan cam bien line");
                        Start();
                        return std::string("Dang do line");
                    });
        mcp.AddTool("self.line.stop", "Dung do line, tat do line, thoi chay theo vach.", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        Stop();
                        return true;
                    });
        mcp.AddTool("self.line.calibrate",
                    "Hieu chinh cam bien line / can chinh cam bien do line. Dat robot de cam bien nam tren vach, "
                    "robot se quay trai phai khoang 3 giay.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue { return Calibrate(); });
        mcp.AddTool("self.line.set_speed", "Toc do do line (phan tram 20-100): do line nhanh len, cham lai.",
                    PropertyList({Property("speed", kPropertyTypeInteger, 20, 100)}),
                    [this](const PropertyList& p) -> ReturnValue {
                        cfg_.params.base_speed = p["speed"].value<int>();
                        return "Toc do do line " + std::to_string(cfg_.params.base_speed) + "%";
                    });
        mcp.AddTool("self.line.set_pid",
                    "Chinh he so PID do line. kp, ki, kd la so thap phan (vd kp 0.04, ki 0, kd 0.1); "
                    "bo trong = giu nguyen. Robot lac qua: giam kp hoac tang kd. Cua khong kip: tang kp.",
                    PropertyList({Property("kp", kPropertyTypeString, std::string("")),
                                  Property("ki", kPropertyTypeString, std::string("")),
                                  Property("kd", kPropertyTypeString, std::string(""))}),
                    [this](const PropertyList& p) -> ReturnValue {
                        ParseFloat(p["kp"].value<std::string>(), cfg_.params.kp);
                        ParseFloat(p["ki"].value<std::string>(), cfg_.params.ki);
                        ParseFloat(p["kd"].value<std::string>(), cfg_.params.kd);
                        char buf[96];
                        snprintf(buf, sizeof(buf), "Kp %.4f, Ki %.5f, Kd %.3f", cfg_.params.kp, cfg_.params.ki,
                                 cfg_.params.kd);
                        return std::string(buf);
                    });
        mcp.AddTool("self.line.read_sensors", "Doc gia tri 6 cam bien line (de kiem tra day noi).", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        int raw[line::kMaxSensors], val[line::kMaxSensors];
                        ReadRaw(raw);
                        line::Normalize(raw, n_, calib_, cfg_.params.white_line, val);
                        std::string r;
                        for (int i = 0; i < n_; i++) {
                            r += "S" + std::to_string(i) + "(GPIO" + std::to_string(sensors_[i].pin) +
                                 (sensors_[i].adc ? ")=" : " digital)=") + std::to_string(raw[i]) + "/" +
                                 std::to_string(val[i]) + "  ";
                        }
                        return r;
                    });
    }
};

#endif  // _MULTI_LINE_FOLLOWER_H_
