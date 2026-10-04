#include "ld2402.h"
#include "ld2402_web.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <unordered_map>

namespace esphome {
namespace ld2402 {

static const char *TAG = "ld2402";

// ═══════════════════════════════════════════════════════════════════
//  工具函数：构建完整命令帧
// ═══════════════════════════════════════════════════════════════════

static std::vector<uint8_t> build_frame(uint16_t cmd,const uint8_t *val = nullptr,size_t val_len = 0) {
    std::vector<uint8_t> f;
    for (auto b : CMD_HEADER) f.push_back(b);
    uint16_t dlen = 2 + (uint16_t)val_len;
    f.push_back(dlen & 0xFF);
    f.push_back((dlen >> 8) & 0xFF);
    f.push_back(cmd & 0xFF);
    f.push_back((cmd >> 8) & 0xFF);
    for (size_t i = 0; i < val_len; i++) f.push_back(val[i]);
    for (auto b : CMD_FOOTER) f.push_back(b);
    return f;
}

// 5.2.2.使能配置命令：每次配置前必须先发送，成功后才能继续发送其他配置命令，配置完成后发送结束配置命令退出配置模式
void LD2402Component::enqueue_enable_cmd_(std::function<void(const std::vector<uint8_t>&)> cb) {
    CmdQueueItem item;
    uint8_t val[] = {0x01, 0x00};
    item.payload       = build_frame(CMD_ENABLE_CONFIG, val, 2);
    item.is_config_cmd = true;
    if (cb) item.callback = cb;
    enqueue_cmd_(item);
}

// 5.2.2.结束配置命令：退出配置模式
void LD2402Component::enqueue_end_cmd_() {
    CmdQueueItem item;
    item.payload       = build_frame(CMD_END_CONFIG);
    item.is_config_cmd = true;
    item.callback      = [](const std::vector<uint8_t>&){};
    enqueue_cmd_(item);
}

// ═══════════════════════════════════════════════════════════════════
//  生命周期
// ═══════════════════════════════════════════════════════════════════

void LD2402Component::setup() {
    ESP_LOGI(TAG, "LD2402 setup");
    register_web_handler_();

    cmd_read_firmware([this](const std::string &ver) {
        firmware_ver_ = ver;
        if (firmware_sensor_) firmware_sensor_->publish_state(ver);
        ESP_LOGI(TAG, "Firmware: %s", ver.c_str());
    });

    cmd_read_sn([this](const std::string &sn) {
        sn_str_ = sn;
        ESP_LOGI(TAG, "SN: %s", sn.c_str());
    });

    if (work_mode_sensor_) work_mode_sensor_->publish_state("正常");
}

void LD2402Component::loop() {
    process_rx_();
    pump_cmd_queue_();
#if !defined(USE_ESP32) && defined(USE_ARDUINO)
    if (this->sse_.count() > 0) {
        const uint32_t now = millis();
        const uint32_t interval_ms = this->engineer_mode_ ? 200 : 500;
        if (now - this->last_sse_ms_ >= interval_ms) {
            this->last_sse_ms_ = now;
            this->push_sse_update_();
        }
    }
#endif
}

// ═══════════════════════════════════════════════════════════════════
//  UART 发送
// ═══════════════════════════════════════════════════════════════════

void LD2402Component::send_raw_(const std::vector<uint8_t> &frame) {
    write_array(frame.data(), frame.size());
}

// 直接发送 item.payload（已是完整帧）
void LD2402Component::send_cmd_frame_(const CmdQueueItem &item) {
    // 发送前先清空接收缓冲区，避免工程模式下数据帧堆积干扰 ACK 接收
    process_rx_();
    write_array(item.payload.data(), item.payload.size());
}

// ═══════════════════════════════════════════════════════════════════
//  接收解析
// ═══════════════════════════════════════════════════════════════════

void LD2402Component::process_rx_() {
    while (available()) {
        uint8_t b;
        read_byte(&b);
        parse_byte_(b);
    }
}

void LD2402Component::parse_byte_(uint8_t b) {
    // ── 超时保护：非 IDLE 状态超过 150ms 强制回 IDLE ──
    if (parse_state_ != ParseState::IDLE && check_parse_timeout_()) {
        ESP_LOGW(TAG, "Parse timeout in state %d, reset to IDLE", (int) parse_state_);
        parse_state_ = ParseState::IDLE;
        frame_buf_.clear();
        line_buf_.clear();
    }
    parse_state_ms_ = millis();

    switch (parse_state_) {
        case ParseState::IDLE:
            hdr_idx_ = 0;
            cmd_hdr_match_ = 0;
            dat_hdr_match_ = 0;
            frame_buf_.clear();
            if (b == CMD_HEADER[0]) {
                parse_state_ = ParseState::CMD_HEADER;
                hdr_idx_ = 1;
            } else if (engineer_mode_ && b == DATA_HEADER[0]) {
                parse_state_ = ParseState::DAT_HEADER;
                hdr_idx_ = 1;
            } else {
                if (b == '\n') {
                    if (!line_buf_.empty() && line_buf_.back() == '\r')
                        line_buf_.pop_back();
                    if (!line_buf_.empty())
                        parse_normal_line_(line_buf_);
                    line_buf_.clear();
                } else if (b >= 0x20 || b == '\r') {
                    line_buf_ += (char)b;
                }
            }
            break;
        case ParseState::CMD_HEADER:
            if (b == CMD_HEADER[hdr_idx_]) {
                hdr_idx_++;
                if (hdr_idx_ == 4) {
                    parse_state_ = ParseState::CMD_LENGTH;
                    frame_len_ = 0;
                    frame_recv_ = 0;
                }
            } else {
                parse_state_ = ParseState::IDLE;
                line_buf_.clear();
            }
            break;
        case ParseState::CMD_LENGTH:
            if (frame_recv_ == 0) {
                frame_len_ = b;
                frame_recv_ = 1;
            } else {
                frame_len_ |= ((uint16_t)b << 8);
                frame_buf_.clear();
                frame_recv_ = 0;
                if (frame_len_ == 0 || frame_len_ > 256) {
                    parse_state_ = ParseState::IDLE;
                } else {
                    parse_state_ = ParseState::CMD_DATA;
                }
            }
            break;
        case ParseState::CMD_DATA:
            // ── 同样检查是否出现了 DATA 头标记 ──
            if (engineer_mode_ && scan_data_header_(b)) {
                ESP_LOGD(TAG, "DATA header detected in CMD_DATA, aborting CMD frame");
                frame_buf_.clear();
                frame_len_ = 0;
                frame_recv_ = 0;
                parse_state_ = ParseState::DAT_LENGTH;
                break;
            }
            frame_buf_.push_back(b);
            frame_recv_++;
            if (frame_recv_ >= frame_len_) {
                hdr_idx_ = 0;
                parse_state_ = ParseState::CMD_FOOTER;
            }
            break;
        case ParseState::CMD_FOOTER:
            if (b == CMD_FOOTER[hdr_idx_]) {
                hdr_idx_++;
                if (hdr_idx_ == 4) {
                    dispatch_cmd_frame_(frame_buf_);
                    parse_state_ = ParseState::IDLE;
                }
            } else {
                parse_state_ = ParseState::IDLE;
            }
            break;
        case ParseState::DAT_HEADER:
            if (b == DATA_HEADER[hdr_idx_]) {
                hdr_idx_++;
                if (hdr_idx_ == 4) {
                    parse_state_ = ParseState::DAT_LENGTH;
                    frame_len_ = 0;
                    frame_recv_ = 0;
                }
            } else {
                parse_state_ = ParseState::IDLE;
            }
            break;
        case ParseState::DAT_LENGTH:
            if (frame_recv_ == 0) {
                frame_len_ = b;
                frame_recv_ = 1;
            } else {
                frame_len_ |= ((uint16_t)b << 8);
                frame_buf_.clear();
                frame_recv_ = 0;
                parse_state_ = ParseState::DAT_DATA;
            }
            break;
        case ParseState::DAT_DATA:
            // ── 关键修复：扫描是否出现了 CMD 头标记 ──
            // 工程模式下 DATA 帧和 CMD 回复交错在同一字节流中，
            // 如果当前帧的长度计算有误（帧同步丢失），CMD 回复的
            // 头字节会被当成 DATA 的 payload 吞掉，导致命令永远收不到 ACK。
            if (scan_cmd_header_(b)) {
                ESP_LOGD(TAG, "CMD header detected in DAT_DATA, aborting DATA frame");
                frame_buf_.clear();
                frame_len_ = 0;
                frame_recv_ = 0;
                parse_state_ = ParseState::CMD_LENGTH;
                break;
            }
            frame_buf_.push_back(b);
            frame_recv_++;
            if (frame_recv_ >= frame_len_) {
                hdr_idx_ = 0;
                parse_state_ = ParseState::DAT_FOOTER;
            }
            break;
        case ParseState::DAT_FOOTER:
            if (b == DATA_FOOTER[hdr_idx_]) {
                hdr_idx_++;
                if (hdr_idx_ == 4) {
                    dispatch_data_frame_(frame_buf_);
                    parse_state_ = ParseState::IDLE;
                }
            } else {
                parse_state_ = ParseState::IDLE;
            }
            break;
    }
}

// ── 检查解析是否卡死（非 IDLE 超过 150ms 无字节到达）──
bool LD2402Component::check_parse_timeout_() {
    return (millis() - parse_state_ms_) > 150;
}

// ── 4字节移位寄存器检测 CMD 头 FD FC FB FA ──
bool LD2402Component::scan_cmd_header_(uint8_t b) {
    if (cmd_hdr_match_ < 4) {
        if (b == CMD_HEADER[cmd_hdr_match_]) {
            cmd_hdr_match_++;
            if (cmd_hdr_match_ == 4) {
                cmd_hdr_match_ = 0;
                return true;
            }
        } else {
            // 0xFD 自身也可能是下一帧的起始
            cmd_hdr_match_ = (b == CMD_HEADER[0]) ? 1 : 0;
        }
    }
    return false;
}

// ── 4字节移位寄存器检测 DATA 头 F4 F3 F2 F1 ──
bool LD2402Component::scan_data_header_(uint8_t b) {
    if (dat_hdr_match_ < 4) {
        if (b == DATA_HEADER[dat_hdr_match_]) {
            dat_hdr_match_++;
            if (dat_hdr_match_ == 4) {
                dat_hdr_match_ = 0;
                return true;
            }
        } else {
            dat_hdr_match_ = (b == DATA_HEADER[0]) ? 1 : 0;
        }
    }
    return false;
}

void LD2402Component::parse_normal_line_(const std::string &line) {
    if (line == "OFF") {
        detection_result_ = 0x00;
        target_distance_  = 0;
        if (presence_sensor_) presence_sensor_->publish_state(false);
        if (distance_sensor_)  distance_sensor_->publish_state(0);
        if (interference_sensor_) interference_sensor_->publish_state(false);

    } else if (line.rfind("distance:", 0) == 0) {
        std::string num = line.substr(9);
        size_t end = num.find_first_not_of("0123456789");
        if (end != std::string::npos) num = num.substr(0, end);
        if (!num.empty()) {
            float dist = std::stof(num);
            detection_result_ = 0x01;

            // ✅ 收到 distance: 帧本身 = 有人，不管数值是否为 0
            if (presence_sensor_) presence_sensor_->publish_state(true);

            // ✅ 距离只在有效时更新，dist==0 保留上次有效值
            if (dist > 0) {
                target_distance_ = (uint16_t)dist;
                if (distance_sensor_) distance_sensor_->publish_state(dist);
            }
        }
    }
}


void LD2402Component::dispatch_cmd_frame_(const std::vector<uint8_t> &data) {
    if (data.size() < 4) return;
    on_ack_(data);
}

void LD2402Component::dispatch_data_frame_(const std::vector<uint8_t> &data) {
    if (data.size() < 131) return;

    detection_result_ = data[0];
    target_distance_  = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

    size_t offset = 3;
    for (int i = 0; i < NUM_GATES; i++) {
        motion_energy_[i] = (uint32_t)data[offset]
                          | ((uint32_t)data[offset+1] << 8)
                          | ((uint32_t)data[offset+2] << 16)
                          | ((uint32_t)data[offset+3] << 24);
        offset += 4;
    }
    for (int i = 0; i < NUM_GATES; i++) {
        micro_energy_[i] = (uint32_t)data[offset]
                         | ((uint32_t)data[offset+1] << 8)
                         | ((uint32_t)data[offset+2] << 16)
                         | ((uint32_t)data[offset+3] << 24);
        offset += 4;
    }

    bool has_presence = (detection_result_ != 0x00);
    // 无人时距离归零，与正常模式 HA 实体行为一致
    if (!has_presence) target_distance_ = 0;

    if (presence_sensor_) presence_sensor_->publish_state(has_presence);
    if (distance_sensor_) distance_sensor_->publish_state(has_presence ? (float)target_distance_ : 0.0f);
    if (interference_sensor_) interference_sensor_->publish_state(interference_);
}

// ═══════════════════════════════════════════════════════════════════
//  命令队列
// ═══════════════════════════════════════════════════════════════════

void LD2402Component::enqueue_cmd_(CmdQueueItem item) {
    cmd_queue_.push_back(std::move(item));
}

void LD2402Component::on_ack_(const std::vector<uint8_t> &data) {
    if (!cmd_in_flight_) return;
    auto cb = std::move(cmd_in_flight_->callback);
    cmd_in_flight_.reset();
    if (cb) cb(data);
}

void LD2402Component::pump_cmd_queue_() {
    if (cmd_in_flight_) {
        uint32_t elapsed = millis() - cmd_sent_ms_;
        uint32_t timeout = cmd_in_flight_->is_config_cmd
                         ? 5000
                         : cmd_in_flight_->timeout_ms;

        if (elapsed > timeout) {
            if (cmd_in_flight_->retry_count < 2) {
                ESP_LOGW(TAG, "Command timeout, retry %d/2", cmd_in_flight_->retry_count + 1);
                cmd_in_flight_->retry_count++;
                send_cmd_frame_(*cmd_in_flight_);
                cmd_sent_ms_ = millis();
            } else {
                std::string hex;
                for (uint8_t b : cmd_in_flight_->payload) {
                    char buf[4];
                    snprintf(buf, sizeof(buf), "%02X ", b);
                    hex += buf;
                }
                ESP_LOGW(TAG, "Command failed after 3 attempts, payload: %s", hex.c_str());
                if (cmd_in_flight_->on_timeout)
                    cmd_in_flight_->on_timeout();
                cmd_in_flight_.reset();
            }
        }
        return;
    }

    if (cmd_queue_.empty()) return;

    cmd_in_flight_ = std::make_unique<CmdQueueItem>(std::move(cmd_queue_.front()));
    cmd_queue_.pop_front();
    cmd_in_flight_->retry_count = 0;
    send_cmd_frame_(*cmd_in_flight_);
    cmd_sent_ms_ = millis();
}

// ═══════════════════════════════════════════════════════════════════
//  公开命令接口
// ═══════════════════════════════════════════════════════════════════

void LD2402Component::cmd_read_firmware(std::function<void(const std::string&)> cb) {
    enqueue_enable_cmd_([this, cb](const std::vector<uint8_t> &ack) {
        if (ack.size() < 4 || ack[2] != 0) return;

        CmdQueueItem fw;
        fw.payload       = build_frame(CMD_READ_FW);
        fw.is_config_cmd = true;
        fw.callback = [this, cb](const std::vector<uint8_t> &d) {
            if (d.size() >= 7 && d[2] == 0) {
                uint16_t vlen = (uint16_t)d[4] | ((uint16_t)d[5] << 8);
                if (d.size() >= (size_t)(6 + vlen)) {
                    std::string ver(d.begin() + 6, d.begin() + 6 + vlen);
                    cb(ver);
                }
            }
            enqueue_end_cmd_();
        };
        enqueue_cmd_(fw);
    });
}

void LD2402Component::cmd_read_sn(std::function<void(const std::string&)> cb) {
    enqueue_enable_cmd_([this, cb](const std::vector<uint8_t> &ack) {
        if (ack.size() < 4 || ack[2] != 0) return;

        CmdQueueItem sn;
        sn.payload       = build_frame(CMD_READ_SN_STR);
        sn.is_config_cmd = true;
        sn.callback = [this, cb](const std::vector<uint8_t> &d) {
            if (d.size() >= 7 && d[2] == 0) {
                uint16_t slen = (uint16_t)d[4] | ((uint16_t)d[5] << 8);
                if (d.size() >= (size_t)(6 + slen)) {
                    std::string s(d.begin() + 6, d.begin() + 6 + slen);
                    cb(s);
                }
            }
            enqueue_end_cmd_();
        };
        enqueue_cmd_(sn);
    });
}

void LD2402Component::cmd_set_engineer_mode(bool enable, std::function<void(bool)> cb) {
    enqueue_enable_cmd_([this, enable, cb](const std::vector<uint8_t> &ack) {
        if (ack.size() < 4 || ack[2] != 0) { cb(false); return; }

        uint32_t mode_val = enable ? MODE_ENGINEER : MODE_NORMAL;
        uint8_t mv[6] = {
            0x00, 0x00,
            (uint8_t)(mode_val & 0xFF),
            (uint8_t)((mode_val >> 8) & 0xFF),
            (uint8_t)((mode_val >> 16) & 0xFF),
            (uint8_t)((mode_val >> 24) & 0xFF)
        };

        CmdQueueItem mode;
        mode.payload       = build_frame(CMD_SET_MODE, mv, 6);
        mode.is_config_cmd = true;
        mode.callback = [this, enable, cb](const std::vector<uint8_t> &d) {
            bool ok = (d.size() >= 4 && d[2] == 0);
            if (ok) {
                engineer_mode_ = enable;
                parse_state_   = ParseState::IDLE;
                line_buf_.clear();
                if (work_mode_sensor_)
                    work_mode_sensor_->publish_state(enable ? "工程" : "正常");
            }
            cb(ok);

            enqueue_end_cmd_();
        };
        enqueue_cmd_(mode);
    });
}

void LD2402Component::cmd_read_gate_thresholds(bool micro,std::function<void(const std::vector<uint32_t>&)> cb) {
    uint16_t base = micro ? 0x0030 : 0x0010;
    std::vector<uint8_t> val;
    for (int i = 0; i < NUM_GATES; i++) {
        uint16_t id = base + i;
        val.push_back(id & 0xFF);
        val.push_back((id >> 8) & 0xFF);
    }

    enqueue_enable_cmd_([this, val, micro, cb](const std::vector<uint8_t>&) {
        CmdQueueItem rd;
        rd.payload       = build_frame(CMD_READ_PARAMS, val.data(), val.size());
        rd.is_config_cmd = true;
        rd.timeout_ms    = 1000;
        rd.callback = [this, micro, cb](const std::vector<uint8_t> &d) {
            if (d.size() < 4 || d[2] != 0) { cb({}); }
            else {
                std::vector<uint32_t> vals;
                size_t offset = 4;
                while (offset + 3 < d.size()) {
                    uint32_t v = (uint32_t)d[offset]
                               | ((uint32_t)d[offset+1] << 8)
                               | ((uint32_t)d[offset+2] << 16)
                               | ((uint32_t)d[offset+3] << 24);
                    vals.push_back(v);
                    offset += 4;
                }
                for (int i = 0; i < NUM_GATES && i < (int)vals.size(); i++){
                    if (micro) {
                        micro_thresholds_[i] = vals[i];
                    } else {
                        motion_thresholds_[i] = vals[i];
                    }
                }
                cb(vals);
            }
            enqueue_end_cmd_();
        };
        enqueue_cmd_(rd);
    });
}

void LD2402Component::cmd_write_gate_threshold(bool micro, uint8_t gate, uint32_t raw_val,std::function<void(bool)> cb) {
    uint16_t param_id = (micro ? 0x0030 : 0x0010) + gate;
    std::vector<std::pair<uint16_t, uint32_t>> params = {{param_id, raw_val}};
    cmd_write_params_batch(params, cb);
}

// 5.2.7.配置传感器参数命令：批量写入多个参数，参数格式为 [param_id(2字节), param_value(4字节)]，成功后需要发送保存命令才能生效
void LD2402Component::cmd_write_params_batch(const std::vector<std::pair<uint16_t,uint32_t>> &params,std::function<void(bool)> cb) {
    std::vector<uint8_t> val;
    for (auto &p : params) {
        val.push_back(p.first & 0xFF);
        val.push_back((p.first >> 8) & 0xFF);
        val.push_back(p.second & 0xFF);
        val.push_back((p.second >> 8) & 0xFF);
        val.push_back((p.second >> 16) & 0xFF);
        val.push_back((p.second >> 24) & 0xFF);
    }

    enqueue_enable_cmd_([this, val, cb](const std::vector<uint8_t>&) {
        CmdQueueItem wr;
        wr.payload       = build_frame(CMD_WRITE_PARAMS, val.data(), val.size());
        wr.is_config_cmd = true;
        wr.timeout_ms    = 2000;
        wr.callback = [this, cb](const std::vector<uint8_t> &d) {
            bool ok = d.size() >= 4 && d[2] == 0;
            cb(ok);
            // 只退出配置，不保存
            enqueue_end_cmd_();
        };
        wr.on_timeout = [this, cb]() {
            ESP_LOGW(TAG, "Write params timeout");
            cb(false);
            enqueue_end_cmd_();
        };
        enqueue_cmd_(wr);
    });
}

void LD2402Component::cmd_save_params(std::function<void(bool)> cb) {
    enqueue_enable_cmd_([this, cb](const std::vector<uint8_t> &ack) {
        if (ack.size() < 4 || ack[2] != 0) { cb(false); return; }

        CmdQueueItem save;
        save.payload       = build_frame(CMD_SAVE_PARAMS);
        save.is_config_cmd = true;
        save.timeout_ms    = 3000;
        save.callback = [this, cb](const std::vector<uint8_t> &d) {
            bool ok = (d.size() >= 4 && d[2] == 0);
            ESP_LOGI(TAG, "Save to flash %s", ok ? "OK" : "FAIL");
            cb(ok);
            enqueue_end_cmd_();
        };
        save.on_timeout = [this, cb]() {
            ESP_LOGW(TAG, "Save to flash timeout");
            cb(false);
            enqueue_end_cmd_();
        };
        enqueue_cmd_(save);
    });
}

void LD2402Component::cmd_auto_threshold(uint16_t trig, uint16_t hold, uint16_t micro_coeff,std::function<void(bool)> cb) {
    enqueue_enable_cmd_([this, trig, hold, micro_coeff, cb](const std::vector<uint8_t>&) {
        uint8_t val[6] = {
            (uint8_t)(trig & 0xFF),        (uint8_t)((trig >> 8) & 0xFF),
            (uint8_t)(hold & 0xFF),        (uint8_t)((hold >> 8) & 0xFF),
            (uint8_t)(micro_coeff & 0xFF), (uint8_t)((micro_coeff >> 8) & 0xFF),
        };
        CmdQueueItem at;
        at.payload       = build_frame(CMD_AUTO_THRESHOLD, val, 6);
        at.is_config_cmd = true;
        at.timeout_ms    = 500;
        at.callback = [this, cb](const std::vector<uint8_t> &d) {
            bool ok = (d.size() >= 4 && d[2] == 0);
            cb(ok);
            enqueue_end_cmd_();
        };
        enqueue_cmd_(at);
    });
}

void LD2402Component::cmd_auto_progress() {
    enqueue_enable_cmd_([this](const std::vector<uint8_t>&) {
        CmdQueueItem ap;
        ap.payload       = build_frame(CMD_AUTO_PROGRESS);
        ap.is_config_cmd = true;
        ap.callback = [this](const std::vector<uint8_t> &d) {
            int progress = 0;
            if (d.size() >= 6 && d[2] == 0)
                progress = (int)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
            progress_ = (uint16_t)progress;
            enqueue_end_cmd_();
        };
        enqueue_cmd_(ap);
    });
}

void LD2402Component::cmd_auto_gain(std::function<void(bool)> cb) {
    enqueue_enable_cmd_([this, cb](const std::vector<uint8_t>&) {
        CmdQueueItem ag;
        ag.payload       = build_frame(CMD_AUTO_GAIN);
        ag.is_config_cmd = true;
        ag.timeout_ms    = 5000;
        ag.callback = [this, cb](const std::vector<uint8_t> &d) {
            bool ok = (d.size() >= 4 && d[2] == 0);
            cb(ok);
            enqueue_end_cmd_();
        };
        enqueue_cmd_(ag);
    });
}

void LD2402Component::cmd_read_param(uint16_t id,std::function<void(uint32_t)> cb) {
    enqueue_enable_cmd_([this, id, cb](const std::vector<uint8_t>&) {
        uint8_t val[2] = {(uint8_t)(id & 0xFF), (uint8_t)((id >> 8) & 0xFF)};
        CmdQueueItem rp;
        rp.payload       = build_frame(CMD_READ_PARAMS, val, 2);
        rp.is_config_cmd = true;
        rp.callback = [this, cb](const std::vector<uint8_t> &d) {
            uint32_t v = 0;
            if (d.size() >= 8 && d[2] == 0) {
                v = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
                  | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            }
            cb(v);
            enqueue_end_cmd_();
        };
        enqueue_cmd_(rp);
    });
}

// ═══════════════════════════════════════════════════════════════════
//  Web 服务器（使用 ESPHome 内置 web_server_base）
// ═══════════════════════════════════════════════════════════════════

void LD2402Component::register_web_handler_() {
    if (this->web_registered_)
        return;
    if (web_server_base::global_web_server_base == nullptr) {
        ESP_LOGW(TAG, "ESPHome web server is not available; LD2402 web UI disabled");
        return;
    }
    web_server_base::global_web_server_base->init();
#if !defined(USE_ESP32) && defined(USE_ARDUINO)
    this->sse_.onConnect([](AsyncEventSourceClient *client) { client->send("{\"connected\":true}"); });
    web_server_base::global_web_server_base->add_handler(&this->sse_);
#endif
    web_server_base::global_web_server_base->add_handler(this);
    ESP_LOGI(TAG, "LD2402 web UI registered at /config");
    this->web_registered_ = true;
}

#if !defined(USE_ESP32) && defined(USE_ARDUINO)
void LD2402Component::push_sse_update_() {
    if (this->engineer_mode_) {
        char buf[1150];
        int len = snprintf(buf, sizeof(buf),
            "{\"result\":%d,\"dist\":%d,\"interference\":%d,\"motion\":[",
            this->detection_result_, this->target_distance_,
            this->interference_);

        for (int i = 0; i < NUM_GATES; i++) {
            len += snprintf(buf + len, sizeof(buf) - len,
                "%.2f%s", raw_to_db(this->motion_energy_[i]),
                i < NUM_GATES - 1 ? "," : "");
        }
        len += snprintf(buf + len, sizeof(buf) - len, "],\"micro\":[");
        for (int i = 0; i < NUM_GATES; i++) {
            len += snprintf(buf + len, sizeof(buf) - len,
                "%.2f%s", raw_to_db(this->micro_energy_[i]),
                i < NUM_GATES - 1 ? "," : "");
        }
        len += snprintf(buf + len, sizeof(buf) - len, "]}");
        this->sse_.send(buf);
    } else {
        char buf[160];
        snprintf(buf, sizeof(buf), "{\"result\":%d,\"dist\":%d,\"interference\":%d}",
            this->detection_result_, this->target_distance_,
            this->interference_);
        this->sse_.send(buf);
    }
}
#endif
bool LD2402Component::canHandle(AsyncWebServerRequest *request) const {
    if (request->method() != HTTP_GET && request->method() != HTTP_POST)
        return false;
#ifdef USE_ESP32
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    std::string url(request->url_to(url_buf));
    return url == "/config" || url == "/config/" || url == "/config/api/info" || url == "/config/api/cmd";
#else
    String url = request->url();
    return url == ESPHOME_F("/config") || url == ESPHOME_F("/config/") || url == ESPHOME_F("/config/api/info") ||
           url == ESPHOME_F("/config/api/cmd");
#endif
}

void LD2402Component::handleRequest(AsyncWebServerRequest *request) {
#ifdef USE_ESP32
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    std::string url(request->url_to(url_buf));
#else
    String url = request->url();
#endif
    if (request->method() == HTTP_GET && (url == "/config" || url == "/config/")) {
        this->handle_web_root_(request);
    } else if (request->method() == HTTP_GET && url == "/config/api/info") {
        this->handle_web_info_(request);
    } else if (request->method() == HTTP_POST && url == "/config/api/cmd") {
        this->handle_web_cmd_(request);
    } else {
        request->send(404, "text/plain", "Not found");
    }
}

void LD2402Component::handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index,
                                 size_t total) {
#ifndef USE_ESP32
    if (request->method() != HTTP_POST || total == 0 || total > 2048)
        return;
    if (index == 0 && request->_tempObject == nullptr) {
        request->_tempObject = calloc(total + 1, sizeof(uint8_t));
        if (request->_tempObject == nullptr) {
            request->abort();
            return;
        }
    }
    if (request->_tempObject != nullptr) {
        auto *buffer = static_cast<uint8_t *>(request->_tempObject);
        memcpy(buffer + index, data, len);
    }
#endif
}

void LD2402Component::handle_web_root_(AsyncWebServerRequest *request) {
#ifdef USE_ESP8266
    AsyncWebServerResponse *response =
        request->beginResponse_P(200, "text/html; charset=utf-8", reinterpret_cast<const uint8_t *>(LD2402_WEB_HTML),
                                 LD2402_WEB_HTML_SIZE);
#else
    AsyncWebServerResponse *response = request->beginResponse(
        200, "text/html; charset=utf-8", reinterpret_cast<const uint8_t *>(LD2402_WEB_HTML), LD2402_WEB_HTML_SIZE);
#endif
    response->addHeader(ESPHOME_F("Cache-Control"), ESPHOME_F("no-cache"));
    request->send(response);
}

void LD2402Component::handle_web_info_(AsyncWebServerRequest *request) {
    char buf[550];
    snprintf(buf, sizeof(buf),
        "{\"fw\":\"%s\",\"sn\":\"%s\","
        "\"engineer\":%s,\"progress\":\"%d\",\"interference\":%d,"
        "\"result\":%d,\"dist\":%d,\"motion_th\":[",
        this->firmware_ver_.c_str(), this->sn_str_.c_str(),
        this->engineer_mode_ ? "true" : "false", this->progress_, this->interference_,
        this->detection_result_, this->target_distance_);

    std::string json(buf);
    for (int i = 0; i < NUM_GATES; i++) {
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "%lu%s",static_cast<unsigned long>(this->motion_thresholds_[i]),i < NUM_GATES - 1 ? "," : "");
        json += tmp;
    }
    json += "],\"micro_th\":[";
    for (int i = 0; i < NUM_GATES; i++) {
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "%lu%s",static_cast<unsigned long>(this->micro_thresholds_[i]),i < NUM_GATES - 1 ? "," : "");
        json += tmp;
    }
    json += "],\"motion\":[";
    for (int i = 0; i < NUM_GATES; i++) {
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "%.2f%s", raw_to_db(this->motion_energy_[i]), i < NUM_GATES - 1 ? "," : "");
        json += tmp;
    }
    json += "],\"micro\":[";
    for (int i = 0; i < NUM_GATES; i++) {
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "%.2f%s", raw_to_db(this->micro_energy_[i]), i < NUM_GATES - 1 ? "," : "");
        json += tmp;
    }
    json += "],\"max_distance\":" + std::to_string(this->max_distance_gates_)
          + ",\"timeout\":"       + std::to_string(this->absence_timeout_) + "}";

    request->send(200, "application/json", json.c_str());
}

void LD2402Component::handle_web_cmd_(AsyncWebServerRequest *request) {
    std::string body;
#ifdef USE_ESP32
    body = request->arg("plain");
#else
    if (request->_tempObject != nullptr) {
        body = static_cast<const char *>(request->_tempObject);
    }
#endif
    if (body.empty()) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"No body\"}");
        return;
    }

    auto get_str_val = [](const std::string &s, const char *key) -> std::string {
        std::string k = std::string("\"") + key + "\":\"";
        auto pos = s.find(k);
        if (pos == std::string::npos) return "";
        pos += k.size();
        auto end = s.find('"', pos);
        if (end == std::string::npos) return "";
        return s.substr(pos, end - pos);
    };
    auto get_num_val = [](const std::string &s, const char *key) -> double {
        std::string k = std::string("\"") + key + "\":";
        auto pos = s.find(k);
        if (pos == std::string::npos) return 0;
        pos += k.size();
        return std::stod(s.substr(pos));
    };

    std::string cmd = get_str_val(body, "cmd");
    auto send_ok = [request]() {
        request->send(200, "application/json", "{\"ok\":true}");
    };

    using CmdHandler = std::function<void()>;
    std::unordered_map<std::string, CmdHandler> handlers = {
        {"set_engineer", [&]() {
            bool enable = get_num_val(body, "value") != 0;
            this->cmd_set_engineer_mode(enable, [](bool) {});
            send_ok();
        }},
        {"read_thresholds", [&]() {
            bool micro = get_num_val(body, "micro") != 0;
            this->cmd_read_gate_thresholds(micro, [](const std::vector<uint32_t>&) {});
            send_ok();
        }},
        {"write_threshold", [&]() {
            bool     micro = get_num_val(body, "micro") != 0;
            int      gate  = (int)get_num_val(body, "gate");
            uint32_t raw   = (uint32_t)get_num_val(body, "value");
            this->cmd_write_gate_threshold(micro, (uint8_t)gate, raw, [](bool) {});
            send_ok();
        }},
        {"auto_threshold", [&]() {
            uint16_t trig  = (uint16_t)get_num_val(body, "trig");
            uint16_t hold  = (uint16_t)get_num_val(body, "hold");
            uint16_t micro = (uint16_t)get_num_val(body, "micro");
            this->cmd_auto_threshold(trig, hold, micro, [](bool) {});
            send_ok();
        }},
        {"auto_progress", [&]() {
            this->cmd_auto_progress();
            char buf[48];
            snprintf(buf, sizeof(buf), "{\"ok\":true,\"progress\":%d}", this->progress_);
            request->send(200, "application/json", buf);
        }},
        {"auto_gain", [&]() {
            this->cmd_auto_gain([](bool) {});
            send_ok();
        }},
        {"save_flash", [&]() {
            this->cmd_save_params([](bool ok) {
                ESP_LOGI("ld2402", "Flash save result: %s", ok ? "OK" : "FAIL");
            });
            send_ok();
        }},
        {"set_max_distance", [&]() {
            uint32_t gates = (uint32_t)get_num_val(body, "value");
            gates = std::min((uint32_t)17, std::max((uint32_t)1, gates));
            this->max_distance_gates_ = gates;
            uint32_t param_val = gates * 7;
            if (param_val > 100) param_val = 100;
            this->cmd_write_params_batch({{0x0001, param_val}}, [](bool) {});
            send_ok();
        }},
        {"set_timeout", [&]() {
            uint32_t secs = (uint32_t)get_num_val(body, "value");
            secs = std::min((uint32_t)65535, std::max((uint32_t)0, secs));
            this->absence_timeout_ = secs;
            this->cmd_write_params_batch({{0x0004, secs}}, [](bool) {});
            send_ok();
        }},
        {"read_info", [&]() {
            this->cmd_read_firmware([this](const std::string &ver) {
                this->firmware_ver_ = ver;
                if (this->firmware_sensor_) this->firmware_sensor_->publish_state(ver);
            });
            this->cmd_read_sn([this](const std::string &sn) { this->sn_str_ = sn; });
            this->cmd_read_gate_thresholds(false, [](const std::vector<uint32_t>&) {});
            this->cmd_read_gate_thresholds(true,  [](const std::vector<uint32_t>&) {});
            this->cmd_read_param(0x0001, [this](uint32_t v) {if (v >= 7 && v <= 100) this->max_distance_gates_ = v / 7;});
            this->cmd_read_param(0x0004, [this](uint32_t v) { this->absence_timeout_ = v; });
            this->cmd_read_param(0x0005, [this](uint32_t v) { this->interference_ = v; });
            send_ok();
        }},
    };

    auto it = handlers.find(cmd);
    if (it != handlers.end()) {
        it->second();
    } else {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"Unknown command\"}");
    }
}

}  // namespace ld2402
}  // namespace esphome
