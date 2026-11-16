from opendbc.can.packer import CANPacker
from opendbc.car import Bus, CanData
from opendbc.car.volvo.carstate import AOL_MAIN_ENTRY_WINDOW_FRAMES, CarState
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC
from opendbc.safety import ALTERNATIVE_EXPERIENCE


def _can_data(msg):
  address, data, bus = msg
  return CanData(address, data, bus)


def _make_cma_state(aol_enabled):
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_XC40_RECHARGE)
  if aol_enabled:
    cp.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
  cs = CarState(cp, None)
  parsers = CarState.get_can_parsers(cp)
  packers = {bus: CANPacker(DBC[cp.carFingerprint][bus]) for bus in (Bus.main, Bus.pt, Bus.party)}
  return cs, parsers, packers


def _step(cs, parsers, packers, t_us, cruise_enabled):
  """One cycle of CarState.update(), with baseline traffic on all three buses plus the
  cruise-control message reflecting cruise_enabled on the PT bus (CMA encoding)."""
  messages = [
    packers[Bus.main].make_can_msg("SPEED", 0, {"SPEED": 0}),
    packers[Bus.main].make_can_msg("LCA_2", 0, {}),
    packers[Bus.main].make_can_msg("GEAR_POSITION", 0, {"GEAR_POSITION": 3}),
    packers[Bus.party].make_can_msg("PSCM", 2, {}),
    packers[Bus.party].make_can_msg("DRIVER_INPUT", 2, {}),
    packers[Bus.party].make_can_msg("EGSM", 2, {}),
    packers[Bus.party].make_can_msg("PSCM_RELATED", 2, {}),
    packers[Bus.pt].make_can_msg("ECM_1", 1, {}),
    packers[Bus.pt].make_can_msg("BUS1_CRUISE_CONTROL", 1, {"CRUISE_CONTROL_ENABLED": 1 if cruise_enabled else 0}),
  ]
  packets = [(t_us, [_can_data(msg) for msg in messages])]
  for parser in parsers.values():
    parser.update(packets)
  ret, _ = cs.update(parsers, None)
  return ret


def test_aol_disabled_available_is_always_true():
  """With the AOL alternative-experience bit off, available stays True regardless of taps."""
  cs, parsers, packers = _make_cma_state(aol_enabled=False)
  for t, enabled in ((0, False), (100_000, True), (120_000, False)):
    ret = _step(cs, parsers, packers, t, enabled)
    assert ret.cruiseState.available is True


def test_aol_fast_tap_arms():
  """An off->on->off tap completed within the entry window arms available."""
  cs, parsers, packers = _make_cma_state(aol_enabled=True)
  ret = _step(cs, parsers, packers, 0, False)
  assert ret.cruiseState.available is False

  for i in range(1, 20):
    ret = _step(cs, parsers, packers, i * 10_000, True)
    assert ret.cruiseState.available is False, "must not arm on the rising half alone"

  ret = _step(cs, parsers, packers, 20 * 10_000, False)
  assert ret.cruiseState.available is True


def test_aol_slow_tap_does_not_arm():
  """A tap that takes longer than AOL_MAIN_ENTRY_WINDOW_FRAMES does not arm."""
  cs, parsers, packers = _make_cma_state(aol_enabled=True)
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
  cs, parsers, packers = _make_cma_state(aol_enabled=True)
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
