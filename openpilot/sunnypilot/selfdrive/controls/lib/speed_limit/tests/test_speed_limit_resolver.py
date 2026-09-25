"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import random
import time
from unittest import mock

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES, \
  ANTICIPATE_DECEL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.common.test import OpenpilotTestCase

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  gps_data = create_mock({
    'unixTimestampMillis': time.monotonic() * 1e3,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  now = time.monotonic() * 1e9
  sm_mock.logMonoTime = {'gpsLocation': now, 'gpsLocationExternal': now, 'liveMapDataSP': now}
  return sm_mock


def prime_sign(resolver, sm_mock):
  # Fork: these stock tests check how sources are resolved, starting from a camera reading that is already in use
  # (a new reading is only used once plausible and held; that filter is tested in TestSignFilter)
  resolver.sign_limit = sm_mock['carStateSP'].speedLimit


parametrized_policies = parameterized.expand(
  [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  names=["policy", "sm_key", "function_key"]
)


def resolver_class():
  return SpeedLimitResolver


class TestSpeedLimitResolverValidation(OpenpilotTestCase):

  @parameterized.expand(list(Policy), names=["policy"])
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    prime_sign(resolver, sm_mock)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    prime_sign(resolver, sm_mock)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    prime_sign(resolver, sm_mock)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @parameterized.expand(list(Policy), names=["policy"])
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    prime_sign(resolver, sm_mock)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @parameterized.expand(list(Policy), names=["policy"])
  def test_old_map_data_ignored(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = mocker.MagicMock()
    sm_mock['gpsLocation'].unixTimestampMillis = (time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE) * 1e3
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.


class _Clock:
  now = 1000.

  @classmethod
  def monotonic(cls):
    return cls.now


MPH = CV.MS_TO_MPH


class TestAnticipateLowerLimitAhead(OpenpilotTestCase):
  """Fork: ease off toward a lower limit the map says is ahead. Ages are taken from the messages' receive times
  (the receiver's wall-clock timestamp is what the device logs, and what stopped this from ever firing)."""

  def setup_method(self):
    import openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver as R
    self._patch = mock.patch.object(R, 'time', _Clock)
    self._patch.start()
    _Clock.now = 1000.

  def teardown_method(self):
    self._patch.stop()

  def _sm(self, car_limit, map_limit, ahead, dist, fix_age=0.05, map_age=0.0):
    sm = mock.MagicMock()
    msgs = {
      'carState': mock.MagicMock(gasPressed=False, brakePressed=False, standstill=False),
      'carStateSP': mock.MagicMock(speedLimit=car_limit),
      'liveMapDataSP': mock.MagicMock(speedLimit=map_limit, speedLimitValid=map_limit > 0, speedLimitAhead=ahead,
                                      speedLimitAheadValid=ahead > 0, speedLimitAheadDistance=dist),
      # what the device logs: wall-clock milliseconds
      'gpsLocation': mock.MagicMock(unixTimestampMillis=1.79e12),
      'gpsLocationExternal': mock.MagicMock(unixTimestampMillis=1.79e12),
    }
    sm.__getitem__.side_effect = lambda key: msgs[key]
    gps = (_Clock.now - fix_age) * 1e9
    sm.logMonoTime = {'gpsLocation': gps, 'gpsLocationExternal': gps, 'liveMapDataSP': (_Clock.now - map_age) * 1e9}
    return sm

  def _resolver(self, sign=None):
    resolver = SpeedLimitResolver()
    resolver.policy = Policy.car_state_only
    resolver.is_metric = False
    if sign is not None:
      resolver.sign_limit = sign  # the camera's current reading is already in use
    return resolver

  def _drive(self, resolver, v, seconds, sm_fn):
    """step 20 Hz for `seconds` at speed v; sm_fn(dist_travelled) builds the inputs"""
    out, travelled = [], 0.
    for _ in range(int(seconds * 20)):
      resolver.update(v, sm_fn(travelled))
      out.append((resolver.speed_limit, resolver.source, resolver.easing_step))
      _Clock.now += 0.05
      travelled += v * 0.05
    return out

  def test_ramps_in_whole_mph_toward_lower_limit(self):
    # 70 mph, map agrees, 55 mph in 200 m
    car, nxt = 70 / MPH, 55 / MPH
    resolver = self._resolver(car)
    resolver.update(car, self._sm(car, car, nxt, 200.))
    assert resolver.source == SpeedLimitSource.map
    ramp = (nxt ** 2 + 2 * ANTICIPATE_DECEL * 200.) ** 0.5
    assert resolver.speed_limit * MPH == math.ceil(ramp * MPH - 1e-6)
    assert type(resolver.speed_limit) is float

  def test_steps_about_twice_a_second_and_announces_once(self):
    car, nxt = 70 / MPH, 55 / MPH
    resolver = self._resolver(car)
    out = self._drive(resolver, 30., 8., lambda x: self._sm(car, car, nxt, max(0., 240. - x)))
    values = [round(v * MPH, 3) for v, _, _ in out]
    assert all(abs(v - round(v)) < 1e-6 for v in values)       # whole mph only
    changes = sum(1 for a, b in zip(values, values[1:], strict=False) if a != b)
    assert changes <= 16 and values[-1] <= 56                    # ~1 step per mph, down to the new limit
    eased = [e for _, src, e in out if src == SpeedLimitSource.map]
    assert eased and eased[0] is False and all(eased[1:])       # only the first eased step announces

  def test_no_ramp_when_map_disagrees_with_sign(self):
    resolver = self._resolver(24.6)
    resolver.update(24.6, self._sm(24.6, 20., 20.1, 167.))
    assert resolver.source == SpeedLimitSource.car and resolver.speed_limit == 24.6

  def test_no_ramp_for_higher_limit_far_ahead_or_stale(self):
    resolver = self._resolver()
    for sm in (self._sm(24.6, 24.6, 31.3, 167.), self._sm(24.6, 24.6, 0., 0.), self._sm(24.6, 24.6, 20.1, 900.),
               self._sm(24.6, 24.6, 20.1, 167., fix_age=2 * LIMIT_MAX_MAP_DATA_AGE), self._sm(24.6, 24.6, 20.1, 167., map_age=5.)):
      resolver = self._resolver(24.6)
      resolver.update(24.6, sm)
      assert resolver.source == SpeedLimitSource.car, sm

  def test_eases_toward_low_map_limits_but_not_implausible_ones(self):
    # 55 -> 25 entering a town: map-backed, eased; a 15 ahead is not a posted road limit: not eased
    resolver = self._resolver(55 / MPH)
    resolver.update(24.6, self._sm(55 / MPH, 55 / MPH, 25 / MPH, 100.))
    assert resolver.source == SpeedLimitSource.map and resolver.map_agrees
    resolver = self._resolver(55 / MPH)
    resolver.update(24.6, self._sm(55 / MPH, 55 / MPH, 15 / MPH, 50.))
    assert resolver.source == SpeedLimitSource.car

  def test_holds_through_map_dropout_until_camera_reads_sign(self):
    car, nxt = 70 / MPH, 55 / MPH
    resolver = self._resolver(car)
    self._drive(resolver, 30., 3., lambda x: self._sm(car, car, nxt, max(0., 240. - x)))
    before = resolver.speed_limit
    assert resolver.source == SpeedLimitSource.map
    # map loses the limit ahead, then switches to the new limit before the camera reads the sign
    self._drive(resolver, 30., 1., lambda x: self._sm(car, car, 0., 0.))
    assert resolver.source == SpeedLimitSource.map and resolver.speed_limit <= before
    self._drive(resolver, 30., 5., lambda x: self._sm(car, nxt, 0., 0.))  # passes the sign
    assert resolver.source == SpeedLimitSource.map and round(resolver.speed_limit * MPH) == 55
    # camera reads 55 (and holds it): camera wins
    self._drive(resolver, 30., 1.2, lambda x: self._sm(nxt, nxt, 0., 0.))
    assert resolver.source == SpeedLimitSource.car and round(resolver.speed_limit * MPH) == 55

  def test_releases_well_past_the_sign(self):
    car, nxt = 70 / MPH, 55 / MPH
    resolver = self._resolver(car)
    self._drive(resolver, 30., 1., lambda x: self._sm(car, car, nxt, max(0., 100. - x)))
    self._drive(resolver, 30., 16., lambda x: self._sm(car, car, 0., 0.))  # ~480 m more, camera never reads it
    assert resolver.source == SpeedLimitSource.car and resolver.speed_limit == car

  def test_keeps_lower_limit_when_camera_missed_the_sign(self):
    # the map now gives 55 here and the camera never read a 55 sign: keep 55 (the map said so ahead of time too)
    car, nxt = 70 / MPH, 55 / MPH
    resolver = self._resolver(car)
    self._drive(resolver, 30., 1., lambda x: self._sm(car, car, nxt, max(0., 100. - x)))
    self._drive(resolver, 30., 16., lambda x: self._sm(car, nxt, 0., 0.))
    assert resolver.source == SpeedLimitSource.map and round(resolver.speed_limit * MPH) == 55

  def test_numpy_speed_publishes(self):
    # plannerd passes v_ego from a FirstOrderFilter (numpy float64). Outputs must be plain Python types, or setting
    # speedLimitValid on the capnp message raises (crashed plannerd, Sep 24).
    import numpy as np
    resolver = self._resolver(55 / MPH)
    resolver.update(np.float64(24.6), self._sm(55 / MPH, 55 / MPH, 45 / MPH, 100.))
    assert resolver.source == SpeedLimitSource.map
    assert type(resolver.speed_limit_valid) is bool and type(resolver.speed_limit_last_valid) is bool
    msg = custom.LongitudinalPlanSP.new_message()
    r = msg.speedLimit.resolver
    r.speedLimit = float(resolver.speed_limit)
    r.speedLimitValid = resolver.speed_limit_valid
    r.speedLimitLastValid = resolver.speed_limit_last_valid
    r.distToSpeedLimit = float(resolver.distance)
    r.source = resolver.source
    assert r.speedLimitValid


class TestSignFilter(OpenpilotTestCase):
  """Fork: which camera readings are used, and when; and whether the map backs them up."""

  def setup_method(self):
    import openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver as R
    self._patch = mock.patch.object(R, 'time', _Clock)
    self._patch.start()
    _Clock.now = 1000.

  def teardown_method(self):
    self._patch.stop()

  def _run(self, readings_mph, seconds_each=2.0, map_mph=0, ahead=(0, 0.)):
    r = SpeedLimitResolver()
    r.policy = Policy.car_state_only
    r.is_metric = False
    out = []
    for mph in readings_mph:
      for _ in range(int(seconds_each * 20)):
        sm = TestAnticipateLowerLimitAhead._sm(None, mph / MPH, map_mph / MPH, ahead[0] / MPH, ahead[1])
        r.update(25., sm)
        _Clock.now += 0.05
      out.append(round(r.speed_limit * MPH))
    return r, out

  def test_new_reading_used_after_it_holds(self):
    _, out = self._run([55, 35], seconds_each=0.5)
    assert out == [0, 0]           # neither held for a second
    _, out = self._run([55, 35], seconds_each=1.2)
    assert out == [55, 35]

  def test_flicker_ignored(self):
    # 35 -> 15 -> 25 -> 35 through a town: the 15 is not a posted road limit (ignored); the 25 held, so it is used
    _, out = self._run([35, 15, 25, 35], seconds_each=1.5)
    assert out == [35, 35, 25, 35]
    r, out = self._run([35], seconds_each=1.5)
    assert out == [35]

  def test_implausible_readings_ignored(self):
    for bad in (5, 10, 15, 17, 90, 120):
      _, out = self._run([55, bad], seconds_each=1.5)
      assert out == [55, 55], bad

  def test_lost_reading_keeps_last(self):
    _, out = self._run([55, 0], seconds_each=1.5)
    assert out == [55, 55]

  def test_map_agreement(self):
    r, _ = self._run([35], seconds_each=1.5, map_mph=35)
    assert r.map_agrees and not r.map_conflict
    r, _ = self._run([35], seconds_each=1.5, map_mph=55)
    assert r.map_conflict and not r.map_agrees
    r, _ = self._run([35], seconds_each=1.5)
    assert not r.map_agrees and not r.map_conflict
    # the map's change point just ahead counts (sign read a little before the map switches)
    r, _ = self._run([25], seconds_each=1.5, map_mph=55, ahead=(25, 40.))
    assert r.map_agrees and not r.map_conflict
