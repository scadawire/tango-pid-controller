"""
Unit test for PidController -- exercises the regulation logic without a Tango bus.

The controller has no protocol of its own; it reads a sensor Tango attribute and
drives an actor Tango attribute through DeviceProxy on a background thread. The
tests replace those proxies with in-process mocks and call the real PidController
methods as unbound functions against a lightweight State stub, following the same
pattern as the other drivers' test suites.

A real advanced_pid.PID drives the control math -- only the Tango I/O is mocked.

Usage:
    python test_pid_controller.py
"""

import sys
import os
import functools
import tempfile
import json
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tango import CmdArgType
from advanced_pid import PID

from PidController import PidController


# ===========================================================================
#  Mock Tango DeviceProxy
# ===========================================================================

class MockAttr:
    def __init__(self, value, dtype):
        self.value = value
        self.type = dtype


class MockDevice:
    """Stand-in for a Tango DeviceProxy holding a single-value attribute store."""
    def __init__(self):
        self.store = {}
        self.writes = []

    def set_attr(self, name, value, dtype=CmdArgType.DevDouble):
        self.store[name] = MockAttr(value, dtype)

    def read_attribute(self, name):
        if name not in self.store:
            raise KeyError("no such attribute: %s" % name)
        return self.store[name]

    def write_attribute(self, name, value):
        self.writes.append((name, value))
        if name in self.store:
            self.store[name].value = value
        else:
            self.store[name] = MockAttr(value, CmdArgType.DevDouble)


# ===========================================================================
#  State carrier -- method lookups fall through to PidController
# ===========================================================================

class State:
    def __init__(self):
        # device properties consumed by the tested methods
        self.SensorAttribute = "value"
        self.ActorAttribute = "value"
        self.Hysteresis = 0.0
        self.ActorMinControlInterval = 0.0
        self.ActorMinValue = -10.0
        self.ActorMaxValue = 10.0
        self.sensorValueTargetInitial = -999999999
        self.enabledInitial = False

        self.deviceSensor = MockDevice()
        self.deviceActor = MockDevice()
        self.pid = PID(Kp=2.0, Ki=0.1, Kd=1.0, Tf=0.05)
        self.pid.set_output_limits(self.ActorMinValue, self.ActorMaxValue)
        self.pid.set_initial_value(0.0, None, None)

        # write to a throwaway state file so save_state() never touches cwd
        fd, path = tempfile.mkstemp(prefix="pid_state_", suffix=".json")
        os.close(fd)
        os.remove(path)
        self.STATE_FILE = path

        self.logs = []

    # mirror pytango stream signature: message rendered as `msg % args`
    def _log(self, level, msg, *args):
        self.logs.append((level, msg % args if args else msg))

    def debug_stream(self, msg, *a): self._log("DEBUG", msg, *a)
    def info_stream(self, msg, *a):  self._log("INFO", msg, *a)
    def warn_stream(self, msg, *a):  self._log("WARN", msg, *a)
    def error_stream(self, msg, *a): self._log("ERROR", msg, *a)

    def __getattr__(self, name):
        attr = getattr(PidController, name, None)
        if callable(attr):
            return functools.partial(attr, self)
        if attr is not None:
            return attr
        raise AttributeError("'State' has no attribute '%s'" % name)

    def cleanup(self):
        if os.path.exists(self.STATE_FILE):
            os.remove(self.STATE_FILE)


def make_state(sensor=0.0, target=None, enabled=True, actor=0.0,
               sensor_type=CmdArgType.DevDouble, actor_type=CmdArgType.DevDouble):
    s = State()
    s.deviceSensor.set_attr(s.SensorAttribute, sensor, sensor_type)
    s.deviceActor.set_attr(s.ActorAttribute, actor, actor_type)
    if target is not None:
        PidController.write_sensorValueTarget(s, target)
    PidController.write_enabled(s, enabled)
    # move the last-change timestamp into the past so the interval gate is open
    setattr(s, "_PidController__lastChanged", 0.0)
    return s


# ===========================================================================
#  Test harness
# ===========================================================================

passed = 0
failed = 0
errors = []


def assert_equal(name, actual, expected, tolerance=None):
    global passed, failed
    ok = abs(actual - expected) <= tolerance if tolerance is not None else actual == expected
    if ok:
        passed += 1
        print("  PASS  %s" % name)
    else:
        failed += 1
        msg = "  FAIL  %s: expected %r, got %r" % (name, expected, actual)
        print(msg)
        errors.append(msg)


def assert_true(name, value):  assert_equal(name, bool(value), True)
def assert_false(name, value): assert_equal(name, bool(value), False)


# ===========================================================================
#  Sensor / actor value reading
# ===========================================================================

def test_get_sensor_value_float():
    print("\n-- getSensorValueFloat --")
    s = make_state(sensor=42.5)
    assert_equal("reads numeric sensor", PidController.getSensorValueFloat(s), 42.5)
    s.cleanup()

    s = make_state(sensor="3.14", sensor_type=CmdArgType.DevString)
    assert_equal("casts string sensor to float",
                 PidController.getSensorValueFloat(s), 3.14, tolerance=1e-9)
    s.cleanup()


def test_get_actor_value_float():
    print("\n-- getActorValueFloat --")
    s = make_state(actor=7.0)
    assert_equal("reads numeric actor", PidController.getActorValueFloat(s), 7.0)
    s.cleanup()

    s = make_state(actor="1.5", actor_type=CmdArgType.DevString)
    assert_equal("casts string actor", PidController.getActorValueFloat(s), 1.5, tolerance=1e-9)
    s.cleanup()

    # unreadable actor attribute -> defaults to 0 rather than raising
    s = make_state()
    s.deviceActor.store.clear()
    assert_equal("missing actor defaults to 0", PidController.getActorValueFloat(s), 0)
    s.cleanup()


def test_get_difference():
    print("\n-- getDifference (reference - measurement) --")
    s = make_state(sensor=20.0, target=25.0)
    assert_equal("target above sensor -> positive", PidController.getDifference(s), 5.0)
    s.cleanup()

    s = make_state(sensor=30.0, target=25.0)
    assert_equal("target below sensor -> negative", PidController.getDifference(s), -5.0)
    s.cleanup()


# ===========================================================================
#  regulate() guard conditions
# ===========================================================================

def test_regulate_no_target():
    print("\n-- regulate: no target set --")
    s = make_state(sensor=10.0, target=None, enabled=True)  # target stays sentinel
    PidController.regulate(s)
    assert_equal("no actor write without target", len(s.deviceActor.writes), 0)
    s.cleanup()


def test_regulate_disabled():
    print("\n-- regulate: disabled --")
    s = make_state(sensor=10.0, target=50.0, enabled=False)
    PidController.regulate(s)
    assert_equal("no actor write when disabled", len(s.deviceActor.writes), 0)
    s.cleanup()


def test_regulate_min_interval():
    print("\n-- regulate: min control interval not reached --")
    s = make_state(sensor=10.0, target=50.0, enabled=True)
    s.ActorMinControlInterval = 3600.0
    import time
    setattr(s, "_PidController__lastChanged", time.time())  # just changed
    PidController.regulate(s)
    assert_equal("interval gate suppresses write", len(s.deviceActor.writes), 0)
    s.cleanup()


def test_regulate_hysteresis():
    print("\n-- regulate: hysteresis suppression --")
    s = make_state(sensor=49.5, target=50.0, enabled=True)  # |diff| = 0.5
    s.Hysteresis = 1.0
    PidController.regulate(s)
    assert_equal("difference inside hysteresis -> no write", len(s.deviceActor.writes), 0)
    s.cleanup()


# ===========================================================================
#  regulate() control action
# ===========================================================================

def test_regulate_writes_actor():
    print("\n-- regulate: drives actor toward target --")
    s = make_state(sensor=0.0, target=50.0, enabled=True)
    PidController.regulate(s)
    assert_equal("actor written once", len(s.deviceActor.writes), 1)
    name, value = s.deviceActor.writes[0]
    assert_equal("writes to actor attribute", name == s.ActorAttribute, True)
    # positive error (target above sensor) -> positive control signal
    assert_true("positive error -> positive control", value > 0)
    s.cleanup()


def test_regulate_respects_output_limits():
    print("\n-- regulate: output clamped to actor limits --")
    s = make_state(sensor=0.0, target=1e6, enabled=True)  # huge error -> saturates
    PidController.regulate(s)
    _, value = s.deviceActor.writes[0]
    assert_true("clamped at ActorMaxValue", value <= s.ActorMaxValue + 1e-9)
    s.cleanup()

    s = make_state(sensor=0.0, target=-1e6, enabled=True)
    PidController.regulate(s)
    _, value = s.deviceActor.writes[0]
    assert_true("clamped at ActorMinValue", value >= s.ActorMinValue - 1e-9)
    s.cleanup()


def test_regulate_string_actor():
    print("\n-- regulate: string-typed actor receives a string --")
    s = make_state(sensor=0.0, target=50.0, enabled=True,
                   actor="0", actor_type=CmdArgType.DevString)
    PidController.regulate(s)
    _, value = s.deviceActor.writes[0]
    assert_true("actor value coerced to str", isinstance(value, str))
    s.cleanup()


def test_regulate_updates_last_changed():
    print("\n-- regulate: last-changed timestamp advances after a write --")
    s = make_state(sensor=0.0, target=50.0, enabled=True)
    before = getattr(s, "_PidController__lastChanged")
    PidController.regulate(s)
    after = getattr(s, "_PidController__lastChanged")
    assert_true("lastChanged advanced", after > before)
    s.cleanup()


# ===========================================================================
#  State persistence
# ===========================================================================

def test_save_and_load_state():
    print("\n-- save_state / load_state round-trip --")
    s = make_state(sensor=0.0, target=42.0, enabled=True)
    PidController.save_state(s)
    assert_true("state file created", os.path.exists(s.STATE_FILE))

    with open(s.STATE_FILE) as f:
        raw = json.load(f)
    assert_equal("target persisted", raw["sensorValueTarget"], 42.0)
    assert_equal("enabled persisted", raw["enabled"], True)

    # a fresh controller loading that file adopts the persisted values
    s2 = State()
    s2.STATE_FILE = s.STATE_FILE
    PidController.load_state(s2)
    assert_equal("target restored", getattr(s2, "_PidController__sensorValueTarget"), 42.0)
    assert_equal("enabled restored", getattr(s2, "_PidController__enabled"), True)
    # the PID tuning survives the round-trip
    assert_true("pid state restored", s2.pid.__dict__.get("Kp") is not None)
    s.cleanup()


def test_load_state_missing_file():
    print("\n-- load_state: missing file is a no-op --")
    s = State()
    s.STATE_FILE = s.STATE_FILE + ".does-not-exist"
    PidController.load_state(s)  # must not raise
    assert_equal("target unchanged (sentinel)",
                 getattr(s, "_PidController__sensorValueTarget"), s.TARGET_NO_VALUE)


# ===========================================================================
#  read/write attribute accessors
# ===========================================================================

def test_target_and_enabled_accessors():
    print("\n-- sensorValueTarget / enabled accessors --")
    s = State()
    PidController.write_sensorValueTarget(s, 33.0)
    value, _, _ = PidController.read_sensorValueTarget(s)
    assert_equal("target write/read", value, 33.0)

    PidController.write_enabled(s, True)
    value, _, _ = PidController.read_enabled(s)
    assert_equal("enabled write/read", value, True)
    s.cleanup()


# ===========================================================================
#  Main
# ===========================================================================

def main():
    global failed
    print("=" * 60)
    print("  PidController Unit Test")
    print("=" * 60)
    try:
        test_get_sensor_value_float()
        test_get_actor_value_float()
        test_get_difference()
        test_regulate_no_target()
        test_regulate_disabled()
        test_regulate_min_interval()
        test_regulate_hysteresis()
        test_regulate_writes_actor()
        test_regulate_respects_output_limits()
        test_regulate_string_actor()
        test_regulate_updates_last_changed()
        test_save_and_load_state()
        test_load_state_missing_file()
        test_target_and_enabled_accessors()
    except Exception:
        traceback.print_exc()
        failed += 1

    total = passed + failed
    print("\n%s" % ("=" * 60))
    print("  Results: %d/%d passed, %d failed" % (passed, total, failed))
    if errors:
        print("\n  Failures:")
        for e in errors:
            print("    %s" % e)
    print("=" * 60)
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
