"""Fork (v34): what the speed-limit pop-up says is decided by Speed Limit Assist from what it is doing.

  - 'slowing' only when the car will slow, 'speeding up' only when it will speed up (Nov drives: 'Slowing for 55'
    on a 50 -> 55 early step; 'Slowing for 25' while the car held 47 mph for 8 s more)
  - no pop-up when re-engaging changes nothing
  - a pop-up within 3 s of the previous one replaces it without a second chime
"""
from unittest import mock

from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
import openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist as A
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist

M = CV.MPH_TO_MS
Kind = custom.LongitudinalPlanSP.SpeedLimit.MessageKind
EventNameSP = custom.OnroadEventSP.EventName
SPEED_LIMIT_EVENTS = (EventNameSP.speedLimitActive, EventNameSP.speedLimitChanged)


class Drive:
  """Speed Limit Assist at 20 Hz on a stock-cruise car (this fork's Ioniq 5), offset +5 mph."""

  def __init__(self, v_ego=60, set_mph=60, limit=55):
    Params().put("SpeedLimitMode", 3)  # assist
    cp = car.CarParams.new_message(openpilotLongitudinalControl=False, pcmCruise=True)
    self.sla = SpeedLimitAssist(cp, custom.CarParamsSP.new_message())
    self.t = 1000.
    self.v_ego, self.set_mph, self.limit, self.final = v_ego, set_mph, limit, limit + 5
    self.ahead, self.raising, self.sign_expected, self.quiet = 0, False, False, False
    self.events = []
    self._patch = mock.patch.object(A.time, 'monotonic', side_effect=lambda: self.t)
    self._patch.start()

  def stop(self):
    self._patch.stop()

  def run(self, seconds, **changes):
    for k, v in changes.items():
      setattr(self, k, v)
    out = []
    for _ in range(int(round(seconds * 20))):
      ev = mock.MagicMock()
      self.sla.update(True, False, self.v_ego * M, 0., self.set_mph * M, self.limit * M, self.final * M, True, 0., ev,
                      quiet_change=self.quiet, map_agrees=True, raise_step=self.raising, sign_expected=self.sign_expected,
                      ahead_limit=self.ahead * M)
      for c in ev.add.call_args_list:
        if c.args[0] in SPEED_LIMIT_EVENTS:
          m = self.sla.message
          out.append((m.kind, round(m.limit / M), round(m.target / M), m.direction, m.silent))
      self.t += 0.05
      self.quiet = self.quiet  # easing steps stay quiet once started (set by the caller)
    self.events += out
    return out


class TestMessages(OpenpilotTestCase):
  def setup_method(self):
    self.d = Drive()
    self.d.run(2.)  # engaged at the limit: nothing to say

  def teardown_method(self):
    self.d.stop()

  def test_engaging_with_nothing_to_change_is_quiet(self):
    assert self.d.events == []

  def test_new_sign_slowing(self):
    assert self.d.run(1., limit=45, final=50) == [(Kind.sign, 45, 50, -1, False)]

  def test_new_sign_speeding_up(self):
    assert self.d.run(1., limit=65, final=70, v_ego=60) == [(Kind.sign, 65, 70, 1, False)]

  def test_new_sign_while_already_slower_only_sets(self):
    # 47 mph behind traffic under a new 50 (set 60 -> 55): the car's speed doesn't change
    self.d.run(1., v_ego=47)
    assert self.d.run(1., limit=50, final=55) == [(Kind.sign, 50, 55, 0, False)]

  def test_early_step_of_five_says_speeding_up(self):
    # 50 here, 55 ahead: the early step goes straight to 55 (set 60). Said 'Slowing for 55' before v34.
    self.d.run(1., limit=50, final=55, v_ego=55, set_mph=55)
    out = self.d.run(1., limit=55, final=60, ahead=55, raising=True)
    assert out == [(Kind.higherAhead, 55, 60, 1, True)]  # silent: 1 s after the 50 pop-up

  def test_early_step_part_way_then_sign(self):
    # 55 here, 70 ahead: step to 60 (set 65), then the sign takes it to 75 without a second chime
    self.d.run(4., limit=55, final=60, set_mph=60, v_ego=60)
    assert self.d.run(1., limit=60, final=65, ahead=70, raising=True) == [(Kind.higherAhead, 70, 65, 1, False)]
    self.d.run(4., set_mph=65, v_ego=65)
    out = self.d.run(1., limit=70, final=75, ahead=0, raising=False, sign_expected=True)
    assert out == [(Kind.sign, 70, 75, 1, True)]

  def test_easing_waits_until_the_car_slows(self):
    # Nov 3, 3:20: at 47 mph with the set speed at 60, 25 ahead. The easing steps the set speed down from 60; the
    # pop-up comes when the target reaches the car's speed, not 8 s earlier.
    self.d.run(4., v_ego=47)
    assert self.d.run(0.5, limit=54, final=59, ahead=25) == []          # first step of the easing
    for target in range(58, 48, -1):                                     # 1 mph steps, quiet
      assert self.d.run(0.5, limit=target - 5, final=target, quiet=True) == [], target
    # within ~1 mph of the car's speed: it starts to slow now
    assert self.d.run(0.5, limit=43, final=48, quiet=True) == [(Kind.lowerAhead, 25, 30, -1, False)]
    assert self.d.run(3., limit=40, final=45) == []                      # said once

  def test_easing_already_below_the_car_says_it_at_once(self):
    assert self.d.run(0.5, limit=50, final=55, ahead=35) == [(Kind.lowerAhead, 35, 40, -1, False)]

  def test_deferred_easing_dropped_when_the_sign_comes_first(self):
    self.d.run(4., v_ego=40)
    assert self.d.run(0.5, limit=54, final=59, ahead=35) == []
    assert self.d.run(2., limit=35, final=40, ahead=0, sign_expected=True, quiet=False) == []

  def test_limit_ahead_that_changes_nothing_is_quiet(self):
    # 30 mph with the set speed at 35 and 35 ahead: the early step's target is already set (Nov 4, '35 ahead' alone)
    self.d.run(4., limit=30, final=35, set_mph=35, v_ego=30)
    assert self.d.run(1., limit=30, final=35, ahead=35, raising=True) == []

  def test_second_popup_within_3s_is_silent(self):
    assert self.d.run(1., limit=45, final=50) == [(Kind.sign, 45, 50, -1, False)]
    assert self.d.run(1., limit=35, final=40) == [(Kind.sign, 35, 40, -1, True)]
    self.d.run(4.)
    assert self.d.run(1., limit=25, final=30) == [(Kind.sign, 25, 30, -1, False)]

  def test_resume_says_nothing_when_nothing_changes(self):
    # Nov 4, 16:14: on resuming it said 'set to 60' while heading to 50
    self.d.run(1., limit=45, final=50)
    self.d.run(5., set_mph=50, v_ego=50)
    self.d.sla.update(False, False, 50 * M, 0., 50 * M, 45 * M, 50 * M, True, 0., mock.MagicMock())  # cruise off
    self.d.t += 300.                                                     # much later
    assert self.d.run(2.) == []                                          # resumed at 50: already there

  def test_resume_below_the_target_says_speeding_up(self):
    self.d.sla.update(False, False, 25 * M, 0., 25 * M, 55 * M, 60 * M, True, 0., mock.MagicMock())
    self.d.t += 300.
    assert self.d.run(2., v_ego=25, set_mph=25) == [(Kind.sign, 55, 60, 1, False)]
