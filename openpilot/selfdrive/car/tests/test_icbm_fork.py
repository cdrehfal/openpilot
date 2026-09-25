"""Fork: button management raises the set speed while the driver is on the accelerator (never lowers it), and
holds curve slowing's target within a unit so the set speed doesn't chatter through a curve."""
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

Source = custom.LongitudinalPlanSP.LongitudinalPlanSource
Send = custom.IntelligentCruiseButtonManagement.SendButtonState


def run(icbm, dash, target, override=False, source=Source.speedLimitAssist, frames=60):
  cs = car.CarState.new_message()
  cs.cruiseState.speedCluster = dash * CV.MPH_TO_MS
  cc = car.CarControl.new_message(enabled=True)
  cc.cruiseControl.override = override
  lp = custom.LongitudinalPlanSP.new_message(vTarget=target * CV.MPH_TO_MS, longitudinalPlanSource=source)
  out = []
  for _ in range(frames):
    icbm.run(cs.as_reader(), cc.as_reader(), lp.as_reader(), False)
    out.append(icbm.cruise_button)
  return out


def make():
  CP = car.CarParams.new_message(pcmCruise=True)
  CP_SP = custom.CarParamsSP.new_message(pcmCruiseSpeed=False)
  return IntelligentCruiseButtonManagement(CP.as_reader(), CP_SP.as_reader())


class TestIcbmFork:
  def test_raises_while_on_accelerator(self):
    icbm = make()
    assert Send.increase in run(icbm, 40, 60, override=True)

  def test_never_lowers_while_on_accelerator(self):
    icbm = make()
    assert Send.decrease not in run(icbm, 60, 45, override=True)
    assert Send.decrease in run(icbm, 60, 45, override=False)

  def test_curve_target_wobble_held(self):
    icbm = make()
    run(icbm, 56, 55.6, source=Source.sccVision)            # curve slowing asks ~56
    assert Send.increase not in run(icbm, 56, 56.4, source=Source.sccVision)
    assert Send.decrease not in run(icbm, 56, 55.2, source=Source.sccVision)
    assert Send.decrease in run(icbm, 56, 53.5, source=Source.sccVision)

  def test_speed_limit_steps_not_held(self):
    icbm = make()
    run(icbm, 60, 60, source=Source.speedLimitAssist)
    assert Send.decrease in run(icbm, 60, 59, source=Source.speedLimitAssist)
