from collections import deque

import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.volvo.helpers import LcaAuthorityEnvelope
from opendbc.car.volvo.values import CAR, CarControllerParams, VolvoC1PlatformConfig
from opendbc.car.volvo.volvocan import (create_c1_cancel, create_c1_pscm_message, create_c1_steering_control, create_lca_message,
                                        create_pscm_message, create_lca_3_message, create_lca_2_message, create_lca_4_message,
                                        create_lca_5_message, create_pscm_related_message)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.is_c1 = isinstance(CAR(CP.carFingerprint).config, VolvoC1PlatformConfig)
    self.packer = CANPacker(dbc_names[Bus.pt] if self.is_c1 else dbc_names[Bus.party])
    self.apply_angle_last = 0.0
    self.c1_torque_samples = deque(maxlen=CarControllerParams.C1_N_ZERO_TORQUE)
    self.c1_recovery_until = -1
    self.lca_5_angle_last = 0.0
    self.lca_auth = LcaAuthorityEnvelope()

  def _finish_update(self, actuators, can_sends):
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends

  def update(self, CC, CS, now_nanos):
    if self.is_c1:
      return self._update_c1(CC, CS)
    return self._update_spa(CC, CS)

  def _update_spa(self, CC, CS):
    actuators = CC.actuators
    CS.lat_commanding = bool(CC.latActive)  # rack acceptance check on the next CarState update

    if not CC.latActive:
      self.apply_angle_last = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
        False, CarControllerParams.ANGLE_LIMITS,
      )
      self.lca_5_angle_last = CS.out.steeringAngleDeg
      self.lca_auth.reset()
      return self._finish_update(actuators, [])

    # Clamp commanded angle
    apply_angle = float(np.clip(
      actuators.steeringAngleDeg,
      CS.out.steeringAngleDeg - CarControllerParams.ANGLE_ERROR,
      CS.out.steeringAngleDeg + CarControllerParams.ANGLE_ERROR,
    ))

    # Rate limit
    apply_angle = apply_std_steer_angle_limits(
      apply_angle, self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      True,
      CarControllerParams.ANGLE_LIMITS,
    )
    self.apply_angle_last = apply_angle

    # Torque-authority envelope for driver override
    authority_pos, authority_neg = self.lca_auth.update(CS.out.steeringTorque)

    frames = CS.stock_frames
    can_sends = []
    # LCA (0x58, 100 Hz), PSCM (0x16) and PSCM_RELATED (0x17) relayed party -> main (100 Hz)
    can_sends += [create_lca_message(self.packer, msg, authority_pos, authority_neg) for msg in frames['LCA']]
    can_sends += [create_pscm_message(self.packer, msg) for msg in frames['PSCM']]
    can_sends += [create_pscm_related_message(self.packer, msg, CS.pilot_assist_engaged) for msg in frames['PSCM_RELATED']]
    # LCA_3 (0x57, ~67 Hz), LCA_2 (0x69, 50 Hz), LCA_4 (0x90, ~29 Hz)
    can_sends += [create_lca_3_message(self.packer, msg) for msg in frames['LCA_3']]
    can_sends += [create_lca_2_message(self.packer, msg) for msg in frames['LCA_2']]
    can_sends += [create_lca_4_message(self.packer, msg) for msg in frames['LCA_4']]
    # LCA_5 (0x67, 50 Hz) carries the angle command; limited per send against the last one
    for msg in frames['LCA_5']:
      self.lca_5_angle_last = apply_std_steer_angle_limits(
        apply_angle, self.lca_5_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
        True, CarControllerParams.LCA_5_ANGLE_LIMITS,
      )
      can_sends.append(create_lca_5_message(self.packer, self.lca_5_angle_last, msg))

    return self._finish_update(actuators, can_sends)

  def _update_c1(self, CC, CS):
    can_sends = []
    actuators = CC.actuators

    if self.frame % 2 == 0:  # stock FSM1 and PSCM1 messages are 50 Hz
      requested_active = CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed
      recovering = requested_active and self.frame < self.c1_recovery_until

      if not requested_active:
        self.c1_torque_samples.clear()
        self.c1_recovery_until = -1
      elif recovering:
        self.c1_torque_samples.clear()
      else:
        if self.c1_recovery_until >= 0:
          self.c1_recovery_until = -1
          self.c1_torque_samples.clear()
        self.c1_torque_samples.append(CS.c1_lka_torque)
        if (len(self.c1_torque_samples) == CarControllerParams.C1_N_ZERO_TORQUE and
            all(torque == 0 for torque in self.c1_torque_samples)):
          self.c1_recovery_until = self.frame + 100
          self.c1_torque_samples.clear()
          recovering = True

      lat_active = requested_active and not recovering
      desired_angle = float(np.clip(
        actuators.steeringAngleDeg,
        CS.out.steeringAngleDeg - CarControllerParams.C1_ANGLE_ERROR,
        CS.out.steeringAngleDeg + CarControllerParams.C1_ANGLE_ERROR,
      ))
      apply_angle = apply_std_steer_angle_limits(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active, CarControllerParams.C1_ANGLE_LIMITS)

      can_sends.append(create_c1_pscm_message(self.packer, CS.c1_msg_pscm))
      can_sends.append(create_c1_steering_control(self.packer, apply_angle, lat_active))
      self.apply_angle_last = apply_angle

    if CC.cruiseControl.cancel and self.frame % 10 == 0:
      can_sends.append(create_c1_cancel(self.packer))

    return self._finish_update(actuators, can_sends)
