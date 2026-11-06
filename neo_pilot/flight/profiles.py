"""
Flight profiles: every limit that differs between flying indoors and outdoors.

Chosen explicitly when a flight program starts (--env indoor|outdoor|route), never
guessed. SafeDrone (flight/safety.py) applies them to every command.
"""
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Profile:
    name: str
    max_stick: float        # cap on any raw stick (the speed tables go up to this)
    max_speed: float        # m/s, cap for the controllers (follow, approach, square)
    max_climb: float        # m/s
    max_yaw_rate: float     # deg/s
    h_min: float            # m above the surface below (ground sensor; else above takeoff): no descending below
    h_max: float            # m above takeoff: no climbing above
    fence: float            # m from where the test started: abort beyond (GPS outdoors, integrated velocity indoors)
    need_gps: bool          # tests start only with GPS in use
    square_side: float      # m
    square_speed: float     # m/s
    approach_speed: float   # m/s, top speed towards the target


INDOOR = Profile("indoor", max_stick=0.6, max_speed=0.4, max_climb=0.2, max_yaw_rate=30.0, h_min=0.5, h_max=2.0,
                 fence=2.5, need_gps=False, square_side=1.0, square_speed=0.3, approach_speed=0.3)

OUTDOOR = Profile("outdoor", max_stick=1.0, max_speed=3.0, max_climb=0.5, max_yaw_rate=60.0, h_min=1.5, h_max=15.0,
                  fence=40.0, need_gps=True, square_side=5.0, square_speed=1.0, approach_speed=1.0)

# routes (keys r, t; examples/example_route.csv reaches 25 m out): a wider fence and height band
ROUTE = replace(OUTDOOR, name="route", fence=50.0, h_max=22.0)

PROFILES = {p.name: p for p in (INDOOR, OUTDOOR, ROUTE)}
