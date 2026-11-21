from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.values import CAR, VolvoC1PlatformConfig, VolvoSafetyFlags

TransmissionType = structs.CarParams.TransmissionType

SAFETY_VOLVO = structs.CarParams.SafetyModel.volvo

PSCM_RELATED_ADDR = 0x17
PSCM_RELATED_BUS = 2  # Bus.party


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def update(self, can_packets):
    # CANParser only exposes decoded signals; PSCM_RELATED status validation also
    # needs the raw bytes (checksums and length), so keep the newest frame for CarState.
    can_packets = list(can_packets)
    if not self.CS.is_c1:
      for _, frames in can_packets:
        for address, data, bus in frames:
          if address == PSCM_RELATED_ADDR and bus == PSCM_RELATED_BUS:
            self.CS.pscm_related_raw = bytes(data)
    return super().update(can_packets)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'volvo'
    platform = CAR(candidate).config
    safety_param = VolvoSafetyFlags.C1.value if isinstance(platform, VolvoC1PlatformConfig) else 0
    ret.safetyConfigs = [get_safety_config(SAFETY_VOLVO, safety_param)]
    ret.dashcamOnly = False
    ret.steerActuatorDelay = 0.2 if isinstance(platform, VolvoC1PlatformConfig) else 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = not isinstance(platform, VolvoC1PlatformConfig)
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False
    ret.pcmCruise = True
    if isinstance(platform, VolvoC1PlatformConfig):
      ret.transmissionType = TransmissionType.automatic
    return ret
