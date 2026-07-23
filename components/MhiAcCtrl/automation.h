#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/version.h"
#include "mhi_ac_ctrl.h"

namespace esphome {
namespace mhi_ac_ctrl {

template <typename... Ts>
class SetVerticalVanesAction : public Action<Ts...> {
 public:
  explicit SetVerticalVanesAction(MhiAcCtrl* parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(int, position);

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override
#else
  void play(Ts... x) override
#endif
  {
    if (this->parent_ == nullptr) {
      return;
    }

    const int position = this->position_.value(x...);
    if (position < 1 || position > 5) {
      return;
    }

    this->parent_->request_vertical_vane_command(static_cast<uint8_t>(position));
  }

 protected:
  MhiAcCtrl* parent_;
};

template <typename... Ts>
class SetHorizontalVanesAction : public Action<Ts...> {
 public:
  explicit SetHorizontalVanesAction(MhiAcCtrl* parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(int, position);

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override
#else
  void play(Ts... x) override
#endif
  {
    if (this->parent_ == nullptr) {
      return;
    }

    const int position = this->position_.value(x...);
    if (position >= 1 && position <= 8) {
      this->parent_->request_horizontal_vane_command(static_cast<uint8_t>(position));
    }
  }

 protected:
  MhiAcCtrl* parent_;
};

template <typename... Ts>
class SetExternalRoomTemperatureAction : public Action<Ts...> {
 public:
  explicit SetExternalRoomTemperatureAction(MhiAcCtrl* parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(float, temperature);

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override
#else
  void play(Ts... x) override
#endif
  {
    if (this->parent_ != nullptr) {
      this->parent_->set_external_room_temperature(this->temperature_.value(x...));
    }
  }

 protected:
  MhiAcCtrl* parent_;
};


template <typename... Ts>
class ArmProtocolTraceAction : public Action<Ts...> {
 public:
  explicit ArmProtocolTraceAction(MhiAcCtrl* parent) : parent_(parent) {}

  void set_label(const std::string& label) { this->label_ = label; }

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override
#else
  void play(Ts... x) override
#endif
  {
    (void) sizeof...(x);
    if (this->parent_ != nullptr) {
      this->parent_->arm_protocol_trace(this->label_);
    }
  }

 protected:
  MhiAcCtrl* parent_;
  std::string label_{"manual"};
};

template <typename... Ts>
class MarkProtocolTraceAction : public Action<Ts...> {
 public:
  explicit MarkProtocolTraceAction(MhiAcCtrl* parent) : parent_(parent) {}

  void set_label(const std::string& label) { this->label_ = label; }

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override
#else
  void play(Ts... x) override
#endif
  {
    (void) sizeof...(x);
    if (this->parent_ != nullptr) {
      this->parent_->mark_protocol_trace_result(this->label_);
    }
  }

 protected:
  MhiAcCtrl* parent_;
  std::string label_{};
};

template <typename... Ts>
class DumpProtocolTraceAction : public Action<Ts...> {
 public:
  explicit DumpProtocolTraceAction(MhiAcCtrl* parent) : parent_(parent) {}

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override
#else
  void play(Ts... x) override
#endif
  {
    (void) sizeof...(x);
    if (this->parent_ != nullptr) {
      this->parent_->dump_protocol_trace();
    }
  }

 protected:
  MhiAcCtrl* parent_;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
