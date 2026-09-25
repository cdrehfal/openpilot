"""Fork: with button management, a driver's +/- press is decided by the car (on this Hyundai, + while on the
accelerator jumps to the current speed). openpilot must follow the dash afterwards, not work out its own result
from the dash value at the moment of the press (Sep 25: 40 + 1 = 41 while the car had jumped to 53, and button
management then pulled the car back down, three times)."""
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper

ButtonType = car.CarState.ButtonEvent.Type
AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


def cs(dash_mph, v_mph, gas=False, buttons=()):
  c = car.CarState.new_message()
  c.vEgo = v_mph * CV.MPH_TO_MS
  c.gasPressed = gas
  c.cruiseState.available = True
  c.cruiseState.enabled = True
  c.cruiseState.speed = dash_mph * CV.MPH_TO_MS
  c.cruiseState.speedCluster = dash_mph * CV.MPH_TO_MS
  c.buttonEvents = [car.CarState.ButtonEvent.new_message(type=t, pressed=p) for t, p in buttons]
  return c.as_reader()


def helper(target_mph):
  CP = car.CarParams.new_message(pcmCruise=True)
  CP_SP = custom.CarParamsSP.new_message(pcmCruiseSpeed=False)
  h = VCruiseHelper(CP.as_reader(), CP_SP.as_reader())
  for _ in range(3):  # engage
    h.update_v_cruise(cs(40, 52, gas=True), True, False)
  h.v_cruise_kph = target_mph * CV.MPH_TO_KPH  # Speed Limit Assist had set 60
  h.sla_state = h.prev_sla_state = AssistState.active
  h.speed_limit_final_last_kph = h.prev_speed_limit_final_last_kph = target_mph * CV.MPH_TO_KPH
  return h


def mph(h):
  return round(h.v_cruise_kph * CV.KPH_TO_MPH)


class TestFollowDash:
  def test_plus_while_on_accelerator_follows_the_cars_jump(self):
    h = helper(60)
    h.update_v_cruise(cs(40, 52, gas=True, buttons=[(ButtonType.accelCruise, True)]), True, False)
    h.update_v_cruise(cs(40, 52, gas=True, buttons=[(ButtonType.accelCruise, False)]), True, False)
    for _ in range(10):
      h.update_v_cruise(cs(40, 53, gas=True), True, False)  # the dash updates a moment later
    for _ in range(150):
      h.update_v_cruise(cs(53, 53), True, False)
    assert mph(h) == 53  # was 41 with the v17 logic

  def test_held_press_follows_until_released(self):
    h = helper(60)
    h.update_v_cruise(cs(60, 60, buttons=[(ButtonType.accelCruise, True)]), True, False)
    for d in (61, 62, 63, 64, 65, 70, 75):
      for _ in range(60):
        h.update_v_cruise(cs(d, 60), True, False)
    h.update_v_cruise(cs(75, 60, buttons=[(ButtonType.accelCruise, False)]), True, False)
    for _ in range(150):
      h.update_v_cruise(cs(75, 60), True, False)
    assert mph(h) == 75

  def test_no_press_keeps_openpilots_target(self):
    h = helper(60)
    for _ in range(200):
      h.update_v_cruise(cs(40, 52, gas=True), True, False)
    assert mph(h) == 60  # the dash lagging (button management paused) doesn't pull the target down
