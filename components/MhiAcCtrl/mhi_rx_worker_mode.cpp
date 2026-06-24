#include "mhi_rx_worker_mode.h"

// Intentionally header-only.
// ESPHome does not reliably compile standalone helper .cpp files from this external component
// unless they are pulled into the generated build. Keeping these helpers inline avoids link-only
// failures in compile tests while preserving host unit-test coverage.
