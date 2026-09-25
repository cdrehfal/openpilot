"""Fork: the speed-limit messages say what is happening, with the numbers."""
from unittest import mock
from openpilot.cereal import custom
from openpilot.common.constants import CV
from opendbc.car.structs import car
import openpilot.sunnypilot.selfdrive.selfdrived.events as ev

Source = custom.LongitudinalPlanSP.SpeedLimit.Source
MPH = CV.MPH_TO_MS


def sm(limit, final, source=Source.car, map_here=0, ahead=0, set_mph=None):
  lp = custom.LongitudinalPlanSP.new_message()
  r = lp.speedLimit.resolver
  r.speedLimitLast = limit * MPH
  r.speedLimitFinalLast = final * MPH
  r.speedLimitOffset = 5 * MPH
  r.source = source
  lmd = custom.LiveMapDataSP.new_message(speedLimitValid=map_here > 0, speedLimit=map_here * MPH,
                                         speedLimitAheadValid=ahead > 0, speedLimitAhead=ahead * MPH)
  msgs = {'longitudinalPlanSP': lp.as_reader(), 'liveMapDataSP': lmd.as_reader(), 'controlsState': mock.MagicMock()}
  s = mock.MagicMock()
  s.__getitem__.side_effect = lambda k: msgs[k]
  cs = car.CarState.new_message(vCruiseCluster=(set_mph or 0) * CV.MPH_TO_KPH).as_reader()
  return s, cs


def texts(alert):
  return alert.alert_text_1, alert.alert_text_2


CP = car.CarParams.new_message(openpilotLongitudinalControl=False, pcmCruise=True).as_reader()


class TestSpeedLimitMessages:
  def test_applied_from_sign(self):
    s, cs = sm(55, 60, map_here=55)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("Limit 55: set 60", "sign, map agrees")
    s, cs = sm(70, 75)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("Limit 70: set 75", "sign, not on map")

  def test_applied_easing(self):
    s, cs = sm(64, 69, source=Source.map, map_here=70, ahead=55)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("55 ahead: slowing", "map, 60 at the sign")

  def test_asking(self):
    s, cs = sm(35, 40, set_mph=60)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None)) in (
      ("Sign 35: tap -", "for 40, not on map"), ("Sign 35: tap - for 40", "not on map"))
    s, cs = sm(55, 60, map_here=70, set_mph=75)
    t = texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None))
    assert "55" in t[0] and "map says 70" in t[1]

  def test_asking_direction_uses_the_right_units(self):
    # dash 37, new target 60: tap + (stock compared km/h against mph and said -)
    s, cs = sm(55, 60, set_mph=37)
    assert "tap +" in texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None))[0]
    s, cs = sm(35, 40, set_mph=60)
    assert "tap -" in texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None))[0]

  def test_map_ahead_counts_as_agreeing(self):
    s, cs = sm(35, 40, map_here=25, ahead=35)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None))[1] == "sign, map agrees"
