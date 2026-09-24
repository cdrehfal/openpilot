"""Fork: when Speed Limit Assist may change the set speed by itself, and when it asks for a +/- tap."""
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist


def sla_at(posted_mph: float, set_mph: int, offset_mph: int = 5) -> SpeedLimitAssist:
  sla = SpeedLimitAssist.__new__(SpeedLimitAssist)  # only the threshold logic is under test
  sla.is_metric = False
  sla._speed_limit = posted_mph * CV.MPH_TO_MS
  sla.speed_limit_final_last_conv = round(posted_mph + offset_mph)
  sla.v_cruise_cluster_conv = set_mph
  return sla


class TestAutoApplyThreshold:
  def test_45_and_up_automatic(self):
    for posted, set_speed in ((45, 60), (55, 40), (55, 60), (70, 60), (65, 75)):
      assert not sla_at(posted, set_speed).apply_confirm_speed_threshold, (posted, set_speed)

  def test_below_45_asks_for_tap(self):
    # ATV/trail 35 signs, school zones, town limits
    for posted, set_speed in ((35, 60), (40, 60), (25, 30), (15, 30), (35, 30)):
      assert sla_at(posted, set_speed).apply_confirm_speed_threshold, (posted, set_speed)

  def test_big_drop_asks_for_tap(self):
    assert sla_at(45, 80).apply_confirm_speed_threshold       # 80 -> 50, drop 30
    assert not sla_at(55, 80).apply_confirm_speed_threshold   # 80 -> 60, drop 20
