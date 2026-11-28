"""Inactive or stalled openpilot hands every replaced address back to the stock frames.

openpilot sends nothing while lateral is inactive, and panda only blocks a stock frame
while it allows lateral and openpilot's replacement for that address is recent
(volvo_cma.h VOLVO_REPLACED_MSGS).
"""
from types import SimpleNamespace
import pytest
from opendbc.car import structs
from opendbc.car.volvo.values import CAR, DBC
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.helpers import checksum_lca_5_message
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.tests.test_cma import CMA, REPLACED, STOCK_BUS, _Actuators, _state, interface, packet, sources


@pytest.mark.parametrize('identity', CMA)
def test_stock_frames_resume_when_openpilot_stops_sending(identity):
  """The panda timing fallback: a replaced address is only blocked while openpilot's
  replacement is recent, so stock frames flow again if openpilot disengages or stalls."""
  cp, ci = interface(identity)
  lib = libsafety_py.libsafety
  lib.set_alternative_experience(0)  # don't inherit another test's AOL marking
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  lib.set_controls_allowed(True)
  for address, bus in STOCK_BUS.items():
    assert lib.safety_fwd_hook(bus, address) != -1, hex(address)

  cc = structs.CarControl.new_message()
  cc.latActive = True
  sent = set()
  for frame in range(4):
    now = 1_000_000_000 + frame * 10_000_000
    lib.set_timer(frame * 10000)
    incoming = sources(identity, frame=frame)
    for msg in incoming:
      lib.safety_rx_hook(packet(msg))
    ci.update([(now, incoming)])
    _, output = ci.apply(cc.as_reader(), now)
    for msg in output:
      assert lib.safety_tx_hook(packet(msg))
      sent.add(msg[0])
  assert sent == {address for address, _ in REPLACED}
  for address in sent:
    assert lib.safety_fwd_hook(STOCK_BUS[address], address) == -1, hex(address)

  # openpilot goes quiet; after the longest replacement timeout every stock frame forwards again.
  lib.set_timer(30000 + 100001)
  for address in sent:
    assert lib.safety_fwd_hook(STOCK_BUS[address], address) != -1, hex(address)


@pytest.mark.parametrize('identity', CMA)
def test_stock_frames_resume_on_the_next_frame_when_panda_cancels(identity):
  """The car cancels cruise: panda withdraws lateral and hands every address back at once,
  while openpilot hasn't seen the cancel yet and is still sending replacements."""
  cp, ci = interface(identity)
  lib = libsafety_py.libsafety
  lib.set_alternative_experience(0)  # don't inherit another test's AOL marking
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  lib.set_controls_allowed(True)
  cc = structs.CarControl.new_message()
  cc.latActive = True

  def cycle(frame):
    now = 1_000_000_000 + frame * 10_000_000
    lib.set_timer(frame * 10000)
    incoming = sources(identity, frame=frame)
    for msg in incoming:
      lib.safety_rx_hook(packet(msg))
    ci.update([(now, incoming)])
    return ci.apply(cc.as_reader(), now)[1]

  for frame in range(4):
    assert all(lib.safety_tx_hook(packet(msg)) for msg in cycle(frame))
  lib.set_controls_allowed(False)
  for address, bus in STOCK_BUS.items():
    assert lib.safety_fwd_hook(bus, address) != -1, hex(address)
  output = cycle(4)
  assert {msg[0] for msg in output} == {address for address, _ in REPLACED}
  assert not any(lib.safety_tx_hook(packet(msg)) for msg in output)


@pytest.mark.parametrize("fingerprint", [CAR.POLESTAR_2, CAR.VOLVO_XC40_RECHARGE])
def test_controller_leaves_lca5_to_panda_forwarding_when_inactive(fingerprint):
  # While inactive, the controller must not TX an LCA_5 substitute at all: panda only
  # blocks the stock frame while openpilot is recently sending one (see
  # volvo_fwd_hook), so the real stock frame flows through untouched.
  cp = CarInterface.get_non_essential_params(fingerprint)
  controller = CarController(DBC[cp.carFingerprint], cp)
  cs = _state()
  cc = SimpleNamespace(latActive=False, actuators=_Actuators())

  for _ in range(4):
    _, can_sends = controller.update(cc, cs, 0)
    assert not any(msg[0] == 0x67 for msg in can_sends)

  # Once active, each stock frame is replaced once with its counter passed straight
  # through, and nothing is sent for a frame with no new stock LCA_5.
  cc.latActive = True
  for counters in ([7], [], [11, 0]):
    cs.stock_frames["LCA_5"] = [{**cs.msg_lca_5, "COUNTER": counter} for counter in counters]
    _, can_sends = controller.update(cc, cs, 0)
    sent = [msg[1] for msg in can_sends if msg[0] == 0x67]
    assert [data[3] >> 4 for data in sent] == counters
    for data in sent:
      assert data[2] == checksum_lca_5_message(data[0], data[1], data[3], data[4], data[5])
