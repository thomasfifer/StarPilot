from opendbc.can.packer import CANPacker
from opendbc.car import Bus, CanData
from opendbc.car.volvo.carstate import AOL_MAIN_ENTRY_WINDOW_FRAMES, CarState
from opendbc.car.volvo.helpers import checksum_1_pscm_related_message, checksum_2_pscm_related_message
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC
from opendbc.safety import ALTERNATIVE_EXPERIENCE


def _can_data(msg):
  address, data, bus = msg
  return CanData(address, data, bus)


def _make_state(aol_enabled):
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_XC40_RECHARGE)
  if aol_enabled:
    cp.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
  cs = CarState(cp, None)
  parsers = CarState.get_can_parsers(cp)
  packers = {bus: CANPacker(DBC[cp.carFingerprint][bus]) for bus in (Bus.main, Bus.pt, Bus.party)}
  return cs, parsers, packers


def _step(cs, parsers, packers, t_us, cruise_enabled, pscm_related=None):
  """One cycle of CarState.update(), with baseline traffic on all three buses plus the
  cruise-control message reflecting cruise_enabled on the PT bus. pscm_related optionally
  replaces the all-zero PSCM_RELATED frame with raw bytes."""
  messages = [
    packers[Bus.main].make_can_msg("SPEED", 0, {"SPEED": 0}),
    packers[Bus.main].make_can_msg("LCA_2", 0, {}),
    packers[Bus.main].make_can_msg("GEAR_POSITION", 0, {"GEAR_POSITION": 3}),
    packers[Bus.party].make_can_msg("PSCM", 2, {}),
    packers[Bus.party].make_can_msg("DRIVER_INPUT", 2, {}),
    packers[Bus.party].make_can_msg("EGSM", 2, {}),
    packers[Bus.party].make_can_msg("PSCM_RELATED", 2, {}),
    packers[Bus.pt].make_can_msg("ECM_1", 1, {}),
    packers[Bus.pt].make_can_msg("BUS1_CRUISE_CONTROL", 1, {"CRUISE_CONTROL_SPA_ENABLED": 1 if cruise_enabled else 0}),
  ]
  frames = [_can_data(msg) for msg in messages]
  if pscm_related is not None:
    frames = [f for f in frames if not (f.address == 0x17 and f.src == 2)] + [CanData(0x17, pscm_related, 2)]
  packets = [(t_us, frames)]
  for parser in parsers.values():
    parser.update(packets)
  ret, _ = cs.update(parsers, None)
  return ret


def test_aol_disabled_available_is_always_true():
  """With the AOL alternative-experience bit off, available stays True regardless of taps."""
  cs, parsers, packers = _make_state(aol_enabled=False)
  for t, enabled in ((0, False), (100_000, True), (120_000, False)):
    ret = _step(cs, parsers, packers, t, enabled)
    assert ret.cruiseState.available is True


def test_aol_fast_tap_arms():
  """An off->on->off tap completed within the entry window arms available."""
  cs, parsers, packers = _make_state(aol_enabled=True)
  ret = _step(cs, parsers, packers, 0, False)
  assert ret.cruiseState.available is False

  for i in range(1, 20):
    ret = _step(cs, parsers, packers, i * 10_000, True)
    assert ret.cruiseState.available is False, "must not arm on the rising half alone"

  ret = _step(cs, parsers, packers, 20 * 10_000, False)
  assert ret.cruiseState.available is True


def test_aol_slow_tap_does_not_arm():
  """A tap that takes longer than AOL_MAIN_ENTRY_WINDOW_FRAMES does not arm."""
  cs, parsers, packers = _make_state(aol_enabled=True)
  _step(cs, parsers, packers, 0, False)
  _step(cs, parsers, packers, 10_000, True)

  for i in range(AOL_MAIN_ENTRY_WINDOW_FRAMES + 5):
    ret = _step(cs, parsers, packers, (2 + i) * 10_000, True)
  ret = _step(cs, parsers, packers, 999_000, False)
  assert ret.cruiseState.available is False


def test_aol_brake_induced_cancel_after_disarm_does_not_rearm():
  """
  with a single on-press, then a brake-induced falling edge shortly after, must not
  resurrect the stale timestamp of the *original* arming tap and falsely re-arm.
  """
  cs, parsers, packers = _make_state(aol_enabled=True)
  _step(cs, parsers, packers, 0, False)
  _step(cs, parsers, packers, 10_000, True)
  ret = _step(cs, parsers, packers, 20_000, False)
  assert ret.cruiseState.available is True

  # Single on-press disarms immediately.
  ret = _step(cs, parsers, packers, 30_000, True)
  assert ret.cruiseState.available is False

  # Soon after (well within the original tap's window), brake auto-cancels stock cruise.
  ret = _step(cs, parsers, packers, 40_000, False)
  assert ret.cruiseState.available is False


def _pscm_related(b1, b2):
  """A PSCM_RELATED frame with both checksums computed for the given bytes 1 and 2."""
  return bytes([checksum_1_pscm_related_message(b1, b2), b1, b2, checksum_2_pscm_related_message(b2), 0, 0, 0, 0])


# Counter 5, LCA_ENABLED_ECHO 1, status 8, replica 5 (matches a frame seen on the car).
_PSCM_SUPPORTED = _pscm_related(0x51, 0x85)


def _make_state_with_pscm_registered():
  # The Volvo parsers subscribe lazily on first access, which happens in the first
  # CarState.update(); frames before that aren't parsed.
  cs, parsers, packers = _make_state(aol_enabled=False)
  cs.update(parsers, None)
  return cs, parsers, packers


def test_pscm_supported_status_has_no_steer_fault():
  cs, parsers, packers = _make_state_with_pscm_registered()
  ret = _step(cs, parsers, packers, 0, False, _PSCM_SUPPORTED)
  assert ret.steerFaultTemporary is False


def test_pscm_unsupported_status_is_a_temporary_steer_fault_until_it_recovers():
  cs, parsers, packers = _make_state_with_pscm_registered()
  quality_bit = _pscm_related(0x51 | 0x8, 0x85)  # valid checksums, quality bit set
  for t, frame, fault in ((0, _PSCM_SUPPORTED, False), (10_000, quality_bit, True), (20_000, _PSCM_SUPPORTED, False)):
    ret = _step(cs, parsers, packers, t, False, frame)
    assert ret.steerFaultTemporary is fault, frame.hex()


def test_pscm_never_received_is_a_temporary_steer_fault():
  cs, parsers, packers = _make_state(aol_enabled=False)
  ret, _ = cs.update(parsers, None)
  assert ret.steerFaultTemporary is True
