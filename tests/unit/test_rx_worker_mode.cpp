#include "mhi_test_common.h"

namespace mhi_unit_tests {

void rx_worker_mode_resolves_auto_by_core_count() {
  EXPECT_FALSE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::AUTO, 0U));
  EXPECT_FALSE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::AUTO, 1U));
  EXPECT_TRUE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::AUTO, 2U));
}

void rx_worker_mode_allows_explicit_override() {
  EXPECT_TRUE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::ENABLED, 1U));
  EXPECT_TRUE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::ENABLED, 2U));

  EXPECT_FALSE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::DISABLED, 1U));
  EXPECT_FALSE(mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode::DISABLED, 2U));
}

void rx_worker_mode_reports_config_names() {
  EXPECT_TRUE(std::string(mhi_rx_worker_mode_to_string(MhiRxWorkerMode::AUTO)) == "auto");
  EXPECT_TRUE(std::string(mhi_rx_worker_mode_to_string(MhiRxWorkerMode::ENABLED)) == "true");
  EXPECT_TRUE(std::string(mhi_rx_worker_mode_to_string(MhiRxWorkerMode::DISABLED)) == "false");
}

}  // namespace mhi_unit_tests
