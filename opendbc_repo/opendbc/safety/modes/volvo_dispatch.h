#pragma once

#include "opendbc/safety/modes/volvo_c1.h"
#include "opendbc/safety/modes/volvo_cma.h"

// safetyParam: 0 = CMA/SPA (XC40 Recharge, S60 Recharge, Polestar 2), 2 = C1 (V40).
// safety.h admits only these two values for SAFETY_VOLVO.
#define VOLVO_FLAG_C1 2U

static bool volvo_dispatch_c1;

static safety_config volvo_dispatch_init(uint16_t param) {
  volvo_dispatch_c1 = param == VOLVO_FLAG_C1;
  return volvo_dispatch_c1 ? volvo_c1_init(param) : volvo_cma_init(param);
}

static void volvo_dispatch_rx(const CANPacket_t *msg) {
  if (volvo_dispatch_c1) {
    volvo_c1_rx_hook(msg);
  } else {
    volvo_cma_rx_hook(msg);
  }
}

static bool volvo_dispatch_tx(const CANPacket_t *msg) {
  return volvo_dispatch_c1 ? volvo_c1_tx_hook(msg) : volvo_cma_tx_hook(msg);
}

// C1 has no dynamic forwarding; only CMA/SPA hands replaced frames back to stock.
static bool volvo_dispatch_fwd(int bus_num, int addr) {
  return !volvo_dispatch_c1 && volvo_cma_fwd_hook(bus_num, addr);
}

// C1 declares no checksum or quality checks; every C1 RX check ignores them.
static uint32_t volvo_dispatch_get_checksum(const CANPacket_t *msg) {
  return volvo_dispatch_c1 ? 0U : volvo_cma_get_checksum(msg);
}

static uint32_t volvo_dispatch_compute_checksum(const CANPacket_t *msg) {
  return volvo_dispatch_c1 ? 0U : volvo_cma_compute_checksum(msg);
}

static bool volvo_dispatch_quality(const CANPacket_t *msg) {
  return volvo_dispatch_c1 || volvo_cma_quality(msg);
}

const safety_hooks volvo_dispatch_hooks = {
  .init = volvo_dispatch_init,
  .rx = volvo_dispatch_rx,
  .tx = volvo_dispatch_tx,
  .fwd = volvo_dispatch_fwd,
  .get_checksum = volvo_dispatch_get_checksum,
  .compute_checksum = volvo_dispatch_compute_checksum,
  .get_quality_flag_valid = volvo_dispatch_quality,
};
