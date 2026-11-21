#pragma once

#include "openpilot/selfdrive/pandad/aol_protocol.h"

// Native AOL policies: SPA receiver layout (param 0) and V60 EUCD (param 8). C1 (param 2) has none.
inline bool volvo_aol_param(uint16_t param) {
  return param == 0U || param == 8U;
}

inline constexpr AolSafetyProfile VOLVO_AOL_PROFILE{36U, volvo_aol_param, true};
