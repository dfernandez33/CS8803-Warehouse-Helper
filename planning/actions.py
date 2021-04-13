from dataclasses import dataclass
from enum import Enum


class ActionType(Enum):
    NAVIGATE = 0,
    PICKUP = 1,
    DROP_OFF = 2


class Location(Enum):
    BALL_SHELF = 0,
    BEER_SHELF = 1,
    CUP_SHELF = 2,
    CARGO_BAY = 3,
    STARTING_LOCATION = 4


class Item(Enum):
    BEER = 0,
    CUP = 1,
    BALL = 2


@dataclass
class Action:
    command: ActionType


@dataclass
class Navigate(Action):
    place_from: Location
    place_to: Location


@dataclass
class PickUp(Action):
    item: Item
    location: Location


@dataclass
class DropOff(Action):
    item: Item
    location: Location
