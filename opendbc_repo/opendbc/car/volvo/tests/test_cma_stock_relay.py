"""openpilot replaces each stock frame exactly once, so the counters reach the rack as
stock sent them, and a rack that stops following LCA commands is a steer fault.

On a Polestar 2 drive (route 0000009c--c675d09699), sampling the newest stock frame on
openpilot's own fixed schedule repeated and then skipped LCA_2 counters whenever the
stock 50 Hz clock drifted into phase with openpilot's 100 Hz loop (about every 5
minutes). Each time, the rack stopped following LCA commands (LCA_ENABLED_ECHO 0) with
no other fault, steering silently went away, and the car cancelled cruise seconds later.
"""
import random
from itertools import pairwise

import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.volvo.helpers import RackAcceptanceMonitor
from opendbc.car.volvo.values import CarControllerParams as P
from opendbc.car.volvo.tests.test_cma import CMA, _make_state, _pscm_related, _step, checksummed_lca_2, interface, packet, sources
from opendbc.safety.tests.libsafety import libsafety_py

CYCLE_NS = 10_000_000          # openpilot's 100 Hz loop
STOCK_PERIOD_NS = 20_010_000   # stock 50 Hz, drifting 10 us per frame: sweeps every phase in ~20 s
JITTER_NS = 2_000_000
LCA_2, LCA_5 = 0x69, 0x67


def _stock_arrivals(cycles, seed):
  rng = random.Random(seed)
  t, arrivals = 1_000_000, []
  while t < cycles * CYCLE_NS:
    arrivals.append(t + rng.randint(-JITTER_NS, JITTER_NS))
    t += STOCK_PERIOD_NS
  return arrivals


def _stock_frames(i):
  """The i-th stock LCA_2 and LCA_5 frames: COUNTER_1 +2, COUNTER_2 and LCA_5 COUNTER +4, mod 15."""
  lca_2 = checksummed_lca_2({'COUNTER_1': (2 * i) % 15, 'COUNTER_2': (4 * i) % 15})
  lca_5 = CANPacker('volvo_mid_1').make_can_msg('LCA_5', 0, {'COUNTER': (4 * i) % 15})
  return [lca_2, lca_5]


def _counters(sends):
  lca_2 = [(data[1] & 0xF, data[5] & 0xF) for address, data, bus in sends if address == LCA_2]
  lca_5 = [data[3] >> 4 for address, data, bus in sends if address == LCA_5]
  return lca_2, lca_5


def _run(identity, cycles, *, seed=0, stall_every=0):
  """Drive the real interface and panda safety with stock LCA_2/LCA_5 on their own drifting
  clock. stall_every > 0 merges that many cycles' CAN into one update now and then (a slow
  openpilot frame), so two stock frames land in one update."""
  cp, ci = interface(identity)
  lib = libsafety_py.libsafety
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  arrivals = _stock_arrivals(cycles, seed)
  base = [msg for msg in sources(identity) if msg[0] not in (LCA_2, LCA_5)]
  cc = structs.CarControl.new_message()

  sent, stock_while_active, pending = [], [], []
  next_stock = 0
  for cycle in range(cycles):
    now = cycle * CYCLE_NS
    incoming = list(base)
    while next_stock < len(arrivals) and arrivals[next_stock] <= now:
      incoming += _stock_frames(next_stock)
      next_stock += 1
    lib.set_timer(now // 1000)
    for msg in incoming:
      assert lib.safety_rx_hook(packet(msg))
    lib.safety_tick()
    # Engagement isn't under test here; the LCA_5 rate limits only apply with lateral allowed.
    lib.set_controls_allowed(True)
    pending.append((1_000_000_000 + now, incoming))
    if stall_every and cycle % 50 < stall_every - 1:
      continue  # openpilot is late; this cycle's CAN is handled with the next one

    cc.latActive = cycle >= 20
    # Reverse every 80 ms so the angle is nearly always slewing at the rate limit, including
    # across the 30 ms gaps between stock LCA_5 frames (panda checks each LCA_5 at 50 Hz limits).
    cc.actuators.steeringAngleDeg = 30.0 if (cycle // 8) % 2 == 0 else -30.0
    if cc.latActive:
      stock_while_active += [msg for _, frames in pending for msg in frames if msg[0] in (LCA_2, LCA_5)]
    state = ci.update(pending)
    pending = []
    assert not state.steerFaultTemporary
    _, output = ci.apply(cc.as_reader(), 1_000_000_000 + now)
    for msg in output:
      assert lib.safety_tx_hook(packet(msg)), (identity, cycle, hex(msg[0]), msg[1].hex())
    sent += output
  return arrivals, sent, stock_while_active


def test_drifting_stock_clock_would_repeat_and_skip_with_fixed_schedule_sampling():
  """The schedule used by these tests reproduces the drive: sampling the newest stock frame
  every other cycle sees zero or two new frames in a sample window, i.e. a repeated and then
  a skipped counter."""
  arrivals = _stock_arrivals(3000, 0)
  new_per_sample = [sum(1 for t in arrivals if (c - 2) * CYCLE_NS < t <= c * CYCLE_NS) for c in range(2, 3000, 2)]
  assert new_per_sample.count(0) > 10 and new_per_sample.count(2) > 10


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('seed', [0, 1])
def test_every_stock_frame_is_replaced_exactly_once_in_order(identity, seed):
  _, sent, stock = _run(identity, 3000, seed=seed)
  sent_lca_2, sent_lca_5 = _counters(sent)
  stock_lca_2, stock_lca_5 = _counters(stock)
  assert sent_lca_2 == stock_lca_2 and sent_lca_5 == stock_lca_5
  assert len(sent_lca_2) > 1400
  # no repeated or skipped counter anywhere in what the rack receives
  assert all((b[0] - a[0]) % 15 == 2 and (b[1] - a[1]) % 15 == 4 for a, b in pairwise(sent_lca_2))
  assert all((b - a) % 15 == 4 for a, b in pairwise(sent_lca_5))


@pytest.mark.parametrize('identity', CMA)
def test_late_openpilot_frame_sends_both_stock_frames_in_order(identity):
  # Three cycles' CAN in one update now and then: two stock frames arrive together and
  # both are replaced, oldest first, with LCA_5 angles that panda still accepts.
  _, sent, stock = _run(identity, 1500, stall_every=3)
  assert _counters(sent) == _counters(stock)
  sent_lca_2, _ = _counters(sent)
  assert all((b[1] - a[1]) % 15 == 4 for a, b in pairwise(sent_lca_2))


@pytest.mark.parametrize('identity', CMA)
def test_nothing_is_sent_for_an_address_without_a_new_stock_frame(identity):
  cp, ci = interface(identity)
  cc = structs.CarControl.new_message()
  cc.latActive = True
  base = [msg for msg in sources(identity) if msg[0] not in (LCA_2, LCA_5)]
  for cycle in range(5):
    ci.update([(1_000_000_000 + cycle * CYCLE_NS, base)])
    _, output = ci.apply(cc.as_reader(), 1_000_000_000 + cycle * CYCLE_NS)
    assert {msg[0] for msg in output}.isdisjoint({LCA_2, LCA_5})


# ***** rack acceptance *****

def _monitor_run(monitor, commanding, echo, frames):
  return [monitor.update(commanding, echo) for _ in range(frames)]


def test_rack_echo_is_ignored_during_the_engage_grace_period():
  monitor = RackAcceptanceMonitor()
  assert not any(_monitor_run(monitor, True, 0, P.RACK_ECHO_GRACE_FRAMES))
  assert not any(_monitor_run(monitor, True, 1, 1000))


def test_rack_refusing_while_steering_faults_then_retries():
  monitor = RackAcceptanceMonitor()
  _monitor_run(monitor, True, 1, P.RACK_ECHO_GRACE_FRAMES + 10)
  # the rack drops the echo: a fault after RACK_ECHO_REFUSED_FRAMES, held for RACK_ECHO_FAULT_FRAMES
  faults = _monitor_run(monitor, True, 0, P.RACK_ECHO_REFUSED_FRAMES)
  assert faults == [False] * (P.RACK_ECHO_REFUSED_FRAMES - 1) + [True]
  # openpilot stops steering because of the fault; the fault holds for its full time either way
  held = _monitor_run(monitor, False, 0, P.RACK_ECHO_FAULT_FRAMES - 1)
  assert all(held)
  assert monitor.update(False, 0) is False
  # retrying starts a fresh grace period, and a rack that accepts again clears everything
  assert not any(_monitor_run(monitor, True, 0, P.RACK_ECHO_GRACE_FRAMES))
  assert not any(_monitor_run(monitor, True, 1, 1000))


def test_brief_echo_dropouts_do_not_fault():
  monitor = RackAcceptanceMonitor()
  _monitor_run(monitor, True, 1, P.RACK_ECHO_GRACE_FRAMES + 10)
  for _ in range(20):
    assert not any(_monitor_run(monitor, True, 0, P.RACK_ECHO_REFUSED_FRAMES - 1))
    assert not any(_monitor_run(monitor, True, 1, 1))


def test_echo_zero_while_not_steering_is_not_a_fault():
  # Stock Pilot Assist off and openpilot not steering: the rack's echo is 0, which is normal.
  monitor = RackAcceptanceMonitor()
  assert not any(_monitor_run(monitor, False, 0, 1000))


def test_carstate_reports_rack_refusal_as_temporary_steer_fault():
  cs, parsers, packers = _make_state()
  cs.update(parsers)  # the parsers subscribe lazily on the first update
  accepting, refusing = _pscm_related(0x51, 0x85), _pscm_related(0x50, 0x85)  # LCA_ENABLED_ECHO 1 / 0
  t = 0

  def step(frame, commanding):
    nonlocal t
    cs.lat_commanding = commanding
    t += 10_000_000
    return _step(cs, parsers, packers, t, True, frame).steerFaultTemporary

  assert not any(step(accepting, True) for _ in range(P.RACK_ECHO_GRACE_FRAMES + 10))
  faults = [step(refusing, True) for _ in range(P.RACK_ECHO_REFUSED_FRAMES)]
  assert faults[-1] and not any(faults[:-1])
  # echo 0 alone is otherwise a supported rack status: no fault when openpilot isn't steering
  cs2, parsers2, packers2 = _make_state()
  cs2.update(parsers2)
  for i in range(200):
    assert not _step(cs2, parsers2, packers2, i * 10_000_000, True, refusing).steerFaultTemporary
