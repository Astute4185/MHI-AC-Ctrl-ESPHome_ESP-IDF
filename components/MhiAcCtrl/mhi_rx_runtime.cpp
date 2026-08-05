#include "mhi_rx_runtime.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "mhi_opdata_decoder.h"
#include "mhi_status_decoder.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const RX_DIAG_TAG = "mhi.diag";

void MhiRxRuntime::configure(MhiStats* stats, bool enable_33_byte_frames) {
  stats_ = stats;
  frame_sync_.set_stats(stats_);
  frame_sync_.set_mode(MhiFrameSyncMode::MOSI_ONLY);
  frame_sync_.set_33_byte_frames_enabled(enable_33_byte_frames);
  reset();
}

void MhiRxRuntime::reset() {
  frame_sync_.reset();

  lock_catalog_();
  frame_catalog_.reset();
  frame_catalog_sequence_ = 0U;
  unlock_catalog_();

  lock_worker_store_();
  worker_decoded_store_.reset();
  unlock_worker_store_();
}

MhiRxServiceResult MhiRxRuntime::service(IMhiRxSource& source, std::size_t max_chunks, bool store_command_candidate) {
  MhiRxServiceResult result{};
  uint8_t buffer[kMhiMaxFrameBytes]{};
  MhiFrameBuffer frame{};

  for (std::size_t chunk = 0U; chunk < max_chunks; chunk++) {
    const std::size_t len = source.read_rx(buffer, sizeof(buffer));
    if (len == 0U) {
      break;
    }

    result.chunks++;
    frame_sync_.push_bytes(buffer, len);

    while (frame_sync_.pop_frame(frame)) {
      if (stats_ != nullptr) {
        stats_->on_valid_frame(millis());
      }
      ingest_frame_(frame, store_command_candidate);
      result.frames++;
    }
  }

  return result;
}

bool MhiRxRuntime::ingest_frame_(const MhiFrameBuffer& frame, bool store_command_candidate) {
  lock_catalog_();
  const uint32_t sequence = ++frame_catalog_sequence_;
  const MhiCatalogIngestResult result =
      frame_catalog_.ingest_mosi_frame(frame.view(), sequence, millis(), store_command_candidate);
  unlock_catalog_();

  if (!result.stored) {
    ESP_LOGVV(RX_DIAG_TAG, "catalog: dropped kind=%s key=0x%04x len=%u", mhi_frame_kind_to_string(result.kind),
              static_cast<unsigned int>(result.opdata_key), static_cast<unsigned int>(frame.len));
    return false;
  }

  if (result.overwritten) {
    ESP_LOGVV(RX_DIAG_TAG, "catalog: overwritten kind=%s key=0x%04x sequence=%lu",
              mhi_frame_kind_to_string(result.kind), static_cast<unsigned int>(result.opdata_key),
              static_cast<unsigned long>(sequence));
  }

  return true;
}

bool MhiRxRuntime::take_latest_extended_status(MhiCatalogedFrame& out) {
  lock_catalog_();
  const bool taken = frame_catalog_.take_latest_extended_status(out);
  unlock_catalog_();
  return taken;
}

bool MhiRxRuntime::take_latest_status(MhiCatalogedFrame& out) {
  lock_catalog_();
  const bool taken = frame_catalog_.take_latest_status(out);
  unlock_catalog_();
  return taken;
}

bool MhiRxRuntime::take_latest_command_candidate(MhiCatalogedFrame& out) {
  lock_catalog_();
  const bool taken = frame_catalog_.take_latest_command_candidate(out);
  unlock_catalog_();
  return taken;
}

bool MhiRxRuntime::take_next_opdata(MhiCatalogedFrame& out) {
  lock_catalog_();
  const bool taken = frame_catalog_.take_next_opdata(out);
  unlock_catalog_();
  return taken;
}

bool MhiRxRuntime::take_latest_unknown(MhiCatalogedFrame& out) {
  lock_catalog_();
  const bool taken = frame_catalog_.take_latest_unknown(out);
  unlock_catalog_();
  return taken;
}

void MhiRxRuntime::clear_command_candidate() {
  lock_catalog_();
  frame_catalog_.clear_command_candidate();
  unlock_catalog_();

  lock_worker_store_();
  worker_decoded_store_.clear_command_candidate();
  unlock_worker_store_();
}

MhiCommandCandidateInfo MhiRxRuntime::command_candidate_info() const {
  MhiCommandCandidateInfo info{};

  lock_catalog_();
  info.catalog_valid = frame_catalog_.command_candidate_valid();
  info.catalog_sequence = frame_catalog_.command_candidate_sequence();
  info.catalog_update_ms = frame_catalog_.command_candidate_update_ms();
  unlock_catalog_();

  lock_worker_store_();
  info.worker_valid = worker_decoded_store_.command_candidate_valid();
  info.worker_sequence = worker_decoded_store_.command_candidate_sequence();
  info.worker_update_ms = worker_decoded_store_.command_candidate_update_ms();
  unlock_worker_store_();

  return info;
}

MhiCatalogStats MhiRxRuntime::catalog_stats() const {
  lock_catalog_();
  const MhiCatalogStats stats = frame_catalog_.stats();
  unlock_catalog_();
  return stats;
}

bool MhiRxRuntime::decode_cataloged_frames_to_worker_store(bool command_confirmation_pending) {
  bool decoded_anything = false;
  MhiCatalogedFrame cataloged{};

  if (command_confirmation_pending && take_latest_command_candidate(cataloged)) {
    decoded_anything = decode_cataloged_frame_to_worker_store_(cataloged, true) || decoded_anything;
  }

  if (take_latest_extended_status(cataloged)) {
    decoded_anything = decode_cataloged_frame_to_worker_store_(cataloged) || decoded_anything;
  }

  if (take_latest_status(cataloged)) {
    decoded_anything = decode_cataloged_frame_to_worker_store_(cataloged) || decoded_anything;
  }

  while (take_next_opdata(cataloged)) {
    decoded_anything = decode_cataloged_frame_to_worker_store_(cataloged) || decoded_anything;
  }

  while (take_latest_unknown(cataloged)) {
    decoded_anything = decode_cataloged_frame_to_worker_store_(cataloged) || decoded_anything;
  }

  return decoded_anything;
}

bool MhiRxRuntime::decode_cataloged_frame_to_worker_store_(const MhiCatalogedFrame& cataloged_frame,
                                                           bool command_candidate) {
  const MhiFrameView view = cataloged_frame.frame.view();

  if (cataloged_frame.kind == MhiFrameKind::STATUS || cataloged_frame.kind == MhiFrameKind::EXTENDED_STATUS) {
    MhiDecodedStatus decoded{};
    if (!MhiStatusDecoder::decode_mosi(view, decoded)) {
      return false;
    }

    lock_worker_store_();
    worker_decoded_store_.store_status(decoded, cataloged_frame.frame, cataloged_frame.sequence,
                                       cataloged_frame.last_update_ms,
                                       cataloged_frame.kind == MhiFrameKind::EXTENDED_STATUS, command_candidate);
    unlock_worker_store_();
    return true;
  }

  if (cataloged_frame.kind == MhiFrameKind::OPDATA) {
    MhiDecodedOpData decoded{};
    if (!MhiOpDataDecoder::decode_mosi(view, decoded)) {
      return false;
    }

    lock_worker_store_();
    worker_decoded_store_.merge_opdata(decoded, cataloged_frame.frame, cataloged_frame.sequence,
                                       cataloged_frame.last_update_ms);
    unlock_worker_store_();
    return true;
  }

  lock_worker_store_();
  worker_decoded_store_.store_unknown(cataloged_frame.frame, cataloged_frame.sequence, cataloged_frame.last_update_ms);
  unlock_worker_store_();
  return true;
}

bool MhiRxRuntime::take_worker_command_candidate(MhiDecodedStatusSnapshot& out) {
  lock_worker_store_();
  const bool taken = worker_decoded_store_.take_command_candidate(out);
  unlock_worker_store_();
  return taken;
}

bool MhiRxRuntime::take_worker_extended_status(MhiDecodedStatusSnapshot& out) {
  lock_worker_store_();
  const bool taken = worker_decoded_store_.take_extended_status(out);
  unlock_worker_store_();
  return taken;
}

bool MhiRxRuntime::take_worker_status(MhiDecodedStatusSnapshot& out) {
  lock_worker_store_();
  const bool taken = worker_decoded_store_.take_status(out);
  unlock_worker_store_();
  return taken;
}

bool MhiRxRuntime::take_worker_opdata(MhiDecodedOpDataSnapshot& out) {
  lock_worker_store_();
  const bool taken = worker_decoded_store_.take_opdata(out);
  unlock_worker_store_();
  return taken;
}

bool MhiRxRuntime::take_worker_unknown(MhiWorkerUnknownSnapshot& out) {
  lock_worker_store_();
  const bool taken = worker_decoded_store_.take_unknown(out);
  unlock_worker_store_();
  return taken;
}

void MhiRxRuntime::on_worker_publish_batch() {
  lock_worker_store_();
  worker_decoded_store_.on_publish_batch();
  unlock_worker_store_();
}

MhiWorkerDecodedStoreStats MhiRxRuntime::worker_store_stats() const {
  lock_worker_store_();
  const MhiWorkerDecodedStoreStats stats = worker_decoded_store_.stats();
  unlock_worker_store_();
  return stats;
}

void MhiRxRuntime::lock_catalog_() const {
#ifdef USE_ESP_IDF
  portENTER_CRITICAL(&frame_catalog_mux_);
#else
  frame_catalog_mux_.lock();
#endif
}

void MhiRxRuntime::unlock_catalog_() const {
#ifdef USE_ESP_IDF
  portEXIT_CRITICAL(&frame_catalog_mux_);
#else
  frame_catalog_mux_.unlock();
#endif
}

void MhiRxRuntime::lock_worker_store_() const {
#ifdef USE_ESP_IDF
  portENTER_CRITICAL(&worker_decoded_store_mux_);
#else
  worker_decoded_store_mux_.lock();
#endif
}

void MhiRxRuntime::unlock_worker_store_() const {
#ifdef USE_ESP_IDF
  portEXIT_CRITICAL(&worker_decoded_store_mux_);
#else
  worker_decoded_store_mux_.unlock();
#endif
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
