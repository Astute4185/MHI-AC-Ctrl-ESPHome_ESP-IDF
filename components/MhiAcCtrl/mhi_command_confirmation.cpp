#include "mhi_command_confirmation.h"

// Intentionally header-only for ESPHome external component builds.
// Some ESPHome copy/compile paths can miss newly added root .cpp files in cached
// compile-test trees, so the implementation lives in the header to avoid linker
// failures while keeping this translation unit harmless for host tests.
