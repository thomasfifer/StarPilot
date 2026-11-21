#pragma once

#include "openpilot/selfdrive/pandad/aol_protocol.h"

// Only the SPA receiver layout (param 0) has a native AOL policy; C1 (param 2) has none.
inline bool volvo_aol_param(uint16_t param) {
  return param == 0U;
}

inline constexpr AolSafetyProfile VOLVO_AOL_PROFILE{36U, volvo_aol_param, true};
