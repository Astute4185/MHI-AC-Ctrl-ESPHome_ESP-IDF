#pragma once

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiTransportPins {
  int sck{-1};
  int mosi{-1};
  int miso{-1};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome