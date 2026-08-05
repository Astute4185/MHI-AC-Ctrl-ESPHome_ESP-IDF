#pragma once

#ifdef USE_ESP_IDF
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#else
#include <mutex>
#endif

#include <cstddef>
#include <cstdint>

#include "mhi_frame_catalog.h"
#include "mhi_frame_sync.h"
#include "mhi_stats.h"
#include "mhi_transport.h"
#include "mhi_worker_decoded_store.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiRxServiceResult {
  uint32_t chunks{0U};
  uint32_t frames{0U};
};

struct MhiCommandCandidateInfo {
  bool catalog_valid{false};
  uint32_t catalog_sequence{0U};
  uint32_t catalog_update_ms{0U};
  bool worker_valid{false};
  uint32_t worker_sequence{0U};
  uint32_t worker_update_ms{0U};
};

// Owns RX framing, cataloguing and the worker-decoded handoff store.
// The controller decides where decoding is applied, while this runtime keeps
// transport ingestion and cross-context storage consistent between main-loop
// and worker execution.
class MhiRxRuntime {
 public:
  void configure(MhiStats* stats, bool enable_33_byte_frames);
  void reset();

  MhiRxServiceResult service(IMhiRxSource& source, std::size_t max_chunks, bool store_command_candidate);

  bool take_latest_extended_status(MhiCatalogedFrame& out);
  bool take_latest_status(MhiCatalogedFrame& out);
  bool take_latest_command_candidate(MhiCatalogedFrame& out);
  bool take_next_opdata(MhiCatalogedFrame& out);
  bool take_latest_unknown(MhiCatalogedFrame& out);
  void clear_command_candidate();
  MhiCommandCandidateInfo command_candidate_info() const;

  MhiCatalogStats catalog_stats() const;

  bool decode_cataloged_frames_to_worker_store(bool command_confirmation_pending);

  bool take_worker_command_candidate(MhiDecodedStatusSnapshot& out);
  bool take_worker_extended_status(MhiDecodedStatusSnapshot& out);
  bool take_worker_status(MhiDecodedStatusSnapshot& out);
  bool take_worker_opdata(MhiDecodedOpDataSnapshot& out);
  bool take_worker_unknown(MhiWorkerUnknownSnapshot& out);
  void on_worker_publish_batch();

  MhiWorkerDecodedStoreStats worker_store_stats() const;

 private:
  bool ingest_frame_(const MhiFrameBuffer& frame, bool store_command_candidate);
  bool decode_cataloged_frame_to_worker_store_(const MhiCatalogedFrame& cataloged_frame,
                                               bool command_candidate = false);

  void lock_catalog_() const;
  void unlock_catalog_() const;
  void lock_worker_store_() const;
  void unlock_worker_store_() const;

  MhiFrameSync frame_sync_{};
  MhiFrameCatalog frame_catalog_{};
  MhiWorkerDecodedStore worker_decoded_store_{};
  uint32_t frame_catalog_sequence_{0U};
  MhiStats* stats_{nullptr};

#ifdef USE_ESP_IDF
  mutable portMUX_TYPE frame_catalog_mux_ = portMUX_INITIALIZER_UNLOCKED;
  mutable portMUX_TYPE worker_decoded_store_mux_ = portMUX_INITIALIZER_UNLOCKED;
#else
  mutable std::mutex frame_catalog_mux_{};
  mutable std::mutex worker_decoded_store_mux_{};
#endif
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
