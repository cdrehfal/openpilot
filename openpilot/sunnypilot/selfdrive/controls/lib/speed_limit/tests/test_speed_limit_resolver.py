"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
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
  return sm_mock


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
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
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


class TestAnticipateLowerLimitAhead(OpenpilotTestCase):
  """Fork: ease off toward a lower limit the map says is ahead, keyed on the message receive time (not the
  receiver's wall-clock timestamp, which is what the device logs and what stopped this from ever firing)."""

  def _sm(self, mocker, car_limit, map_limit, ahead, dist, fix_age=0.05):
    sm = mocker.MagicMock()
    sm.__getitem__.side_effect = lambda key: {
      'carState': create_mock({'gasPressed': False, 'brakePressed': False, 'standstill': False}, mocker),
      'carStateSP': create_mock({'speedLimit': car_limit}, mocker),
      'liveMapDataSP': create_mock({'speedLimit': map_limit, 'speedLimitValid': map_limit > 0,
                                    'speedLimitAhead': ahead, 'speedLimitAheadValid': ahead > 0,
                                    'speedLimitAheadDistance': dist}, mocker),
      # what the device logs: wall-clock milliseconds, ~1.7e12
      'gpsLocation': create_mock({'unixTimestampMillis': time.time() * 1e3}, mocker),
      'gpsLocationExternal': create_mock({'unixTimestampMillis': time.time() * 1e3}, mocker),
    }[key]
    mono = (time.monotonic() - fix_age) * 1e9
    sm.logMonoTime = {'gpsLocation': mono, 'gpsLocationExternal': mono}
    return sm

  def _resolver(self):
    resolver = SpeedLimitResolver()
    resolver.policy = Policy.car_state_only
    return resolver

  def test_ramps_toward_lower_limit_ahead(self, mocker):
    # 55 mph, map agrees, 40 mph in 167 m: aim for ~49 mph now, sourced to the map
    resolver = self._resolver()
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 17.9, 167.))
    assert resolver.source == SpeedLimitSource.map
    assert 21.5 < resolver.speed_limit < 22.5
    assert abs(resolver.distance - 167. + 24.6 * 0.05) < 2.

  def test_no_ramp_when_map_disagrees_with_sign(self, mocker):
    resolver = self._resolver()
    resolver.update(24.6, self._sm(mocker, 24.6, 20., 17.9, 167.))
    assert resolver.source == SpeedLimitSource.car
    assert resolver.speed_limit == 24.6

  def test_no_ramp_for_higher_limit_or_no_ahead(self, mocker):
    resolver = self._resolver()
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 31.3, 167.))
    assert resolver.source == SpeedLimitSource.car
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 0., 0.))
    assert resolver.source == SpeedLimitSource.car

  def test_far_ahead_or_stale_fix_ignored(self, mocker):
    resolver = self._resolver()
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 17.9, 900.))
    assert resolver.source == SpeedLimitSource.car
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 17.9, 167., fix_age=2 * LIMIT_MAX_MAP_DATA_AGE))
    assert resolver.source == SpeedLimitSource.car

  def test_never_above_the_sign(self, mocker):
    # right at the sign the ramp equals the lower limit; far from it, it is capped at the car's limit
    resolver = self._resolver()
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 17.9, 0.))
    assert abs(resolver.speed_limit - 17.9) < 0.01
    resolver.update(24.6, self._sm(mocker, 24.6, 24.6, 17.9, 599.))
    assert resolver.speed_limit == 24.6 and resolver.source == SpeedLimitSource.car
