"""Always On Lateral: the off->on->off cruise double tap stands in for MAIN (carstate.py side).

The native side is covered by opendbc/safety/tests/test_volvo_cma.py::TestVolvoCMAAol."""
from opendbc.car.volvo.carstate import AOL_ENTRY_WINDOW_FRAMES
from opendbc.car.volvo.tests.test_cma import _make_state, _step


def test_aol_fast_tap_arms():
  """An off->on->off tap completed within the entry window arms available."""
  cs, parsers, packers = _make_state(aol_enabled=True)
  ret = _step(cs, parsers, packers, 0, False)
  assert ret.cruiseState.available is False

  for i in range(1, 20):
    ret = _step(cs, parsers, packers, i * 10_000, True)
    assert ret.cruiseState.available is True, "cruise itself is available while engaged"

  ret = _step(cs, parsers, packers, 20 * 10_000, False)
  assert ret.cruiseState.available is True
  assert cs.aol_armed


def test_aol_slow_tap_does_not_arm():
  """A tap that takes longer than AOL_ENTRY_WINDOW_FRAMES is a normal engagement, not a tap."""
  cs, parsers, packers = _make_state(aol_enabled=True)
  _step(cs, parsers, packers, 0, False)
  _step(cs, parsers, packers, 10_000, True)

  for i in range(AOL_ENTRY_WINDOW_FRAMES + 5):
    ret = _step(cs, parsers, packers, (2 + i) * 10_000, True)
  ret = _step(cs, parsers, packers, 999_000, False)
  assert ret.cruiseState.available is False


def test_aol_any_cruise_edge_disarms():
  """Once armed, a normal engagement disarms it, and the cancel afterwards leaves it off."""
  cs, parsers, packers = _make_state(aol_enabled=True)
  _step(cs, parsers, packers, 0, False)
  _step(cs, parsers, packers, 10_000, True)
  assert _step(cs, parsers, packers, 20_000, False).cruiseState.available is True

  ret = _step(cs, parsers, packers, 30_000, True)
  assert not cs.aol_armed and ret.cruiseState.available is True  # engaged cruise is available

  # Soon after (well within the original tap's window), the brake cancels stock cruise.
  # The stale arming timestamp must not be mistaken for a fresh tap.
  ret = _step(cs, parsers, packers, 40_000, False)
  assert ret.cruiseState.available is False


def test_aol_entry_window_matches_panda():
  import re
  from pathlib import Path
  header = Path(__file__).parents[3] / "safety/modes/volvo_cma.h"
  window_us = int(re.search(r"#define VOLVO_AOL_ENTRY_WINDOW_US (\d+)U", header.read_text()).group(1))
  assert AOL_ENTRY_WINDOW_FRAMES * 10_000 == window_us
