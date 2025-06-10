import numpy as np
import pytest
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_I.AltitudeVelocity import AltitudeVelocity
from Class_I.PayloadRange import RangeCalculator
from Optimum_Performance.Range_speed import RangeAnalyzer
from utils import Data, MissionType

@pytest.fixture
def altitude_velocity():
    aircraft_data = Data('design3.json')
    mission_type = MissionType.DESIGN
    return AltitudeVelocity(aircraft_data, mission_type)

@pytest.fixture
def range_calculator_design():
    aircraft_data = Data('design3.json')
    mission_type = MissionType.DESIGN
    return RangeCalculator(data_object=aircraft_data, mission_type=mission_type)

@pytest.fixture
def range_calculator_altitude():
    aircraft_data = Data('design3.json')
    mission_type = MissionType.ALTITUDE
    return RangeCalculator(data_object=aircraft_data, mission_type=mission_type)

@pytest.fixture
def range_calculator_ferry():
    aircraft_data = Data('design3.json')
    mission_type = MissionType.FERRY
    return RangeCalculator(data_object=aircraft_data, mission_type=mission_type)

def test_max_roc_at_h0(altitude_velocity: AltitudeVelocity):
    max_roc, *_ = altitude_velocity.calculate_max_RoC(0)
    max_roc_ft_min = max_roc * 196.85
    assert max_roc_ft_min > 1000, f"Max RoC at h=0 is too low: {max_roc_ft_min} ft/min"

def test_max_aoc_at_h0(altitude_velocity: AltitudeVelocity):
    max_aoc, *_ = altitude_velocity.calculate_max_AoC(0)
    max_aoc_deg_min = max_aoc * 180 / np.pi
    assert max_aoc_deg_min > 3.6, f"Max AoC at h=0 is too low: {max_aoc_deg_min} deg/min"

def test_service_ceiling(altitude_velocity: AltitudeVelocity):
    service_ceiling, _ = altitude_velocity.calculate_h_max()
    service_ceiling_feet = service_ceiling / 0.3048  # Convert to feet
    assert service_ceiling_feet > 10000, f"Service ceiling is too low: {service_ceiling_feet} feet"

def test_design_range(range_calculator_design: RangeCalculator):
    ranges_nm, points = range_calculator_design.analyze_and_plot(show=False)
    assert ranges_nm['design'] >= 2000, f"Design range is too low: {ranges_nm['design']} nmi"

def test_altitude_range(range_calculator_altitude: RangeCalculator):
    ranges_nm, points = range_calculator_altitude.analyze_and_plot(show=False)
    assert ranges_nm['design'] >= 800, f"Altitude range is too low: {ranges_nm['design']} nmi"

def test_ferry_range(range_calculator_ferry: RangeCalculator):
    ranges_nm, points = range_calculator_ferry.analyze_and_plot(show=False)
    assert ranges_nm['ferry']*2 >= 6500, f"Ferry range is too low: {ranges_nm['ferry']*2} nmi"