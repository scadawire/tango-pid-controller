# see also https://advanced-pid.readthedocs.io/en/latest/
# see also https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Derivative_term

import time
from tango import AttrQuality, AttrWriteType, DispLevel, DevState, Attr, CmdArgType, UserDefaultAttrProp, DeviceProxy
from tango.server import Device, attribute, command, DeviceMeta
from tango.server import class_property, device_property
from tango.server import run
import os
import json
from json import JSONDecodeError
from advanced_pid import PID
from threading import Thread

class PidController(Device, metaclass=DeviceMeta):
    pass

    STATE_FILE = "pid_state.json"
    TARGET_NO_VALUE = -999999999
    __sensorValueTarget = self.TARGET_NO_VALUE
    __enabled = False

    sensorValueCurrent = attribute(label="sensorValueCurrent", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ, polling_period=1000,
        unit="S", format="8.4f")

    actorValueCurrent = attribute(label="actorValueCurrent", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ, polling_period=1000,
        unit="A", format="8.4f")

    sensorValueTarget = attribute(label="sensorValueTarget", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ_WRITE, polling_period=1000,
        unit="T", format="8.4f")

    enabled = attribute(label="enabled", dtype=bool,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ_WRITE, polling_period=1000,
        unit="*")

    difference = attribute(label="difference", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ, polling_period=1000,
        unit="DELTA", format="8.4f")

    ActorDevice = device_property(dtype=str, default_value="")
    ActorAttribute = device_property(dtype=str, default_value="")
    SensorDevice = device_property(dtype=str, default_value="")
    SensorAttribute = device_property(dtype=str, default_value="")
    Hysteresis = device_property(dtype=float, default_value=0)
    ActorMinControlInterval = device_property(dtype=float, default_value=0)
    ActorMinValue = device_property(dtype=float, default_value=-10)
    ActorMaxValue = device_property(dtype=float, default_value=10)
    PID_kp = device_property(dtype=float, default_value=2.0)
    PID_ki = device_property(dtype=float, default_value=0.1)
    PID_kd = device_property(dtype=float, default_value=1.0)
    PID_tf = device_property(dtype=float, default_value=0.05)
    sensorValueTargetInitial = device_property(dtype=float, default_value=self.TARGET_NO_VALUE)
    enabledInitial = device_property(dtype=bool, default_value=False)
    regulateInterval = device_property(dtype=float, default_value=1)
    deviceActor = 0
    deviceSensor = 0
    pid = 0
    __lastChanged = time.time()
    
    def read_sensorValueCurrent(self):
        sensorValue = self.getSensorValueFloat()
        return sensorValue, time.time(), AttrQuality.ATTR_VALID
    
    def read_actorValueCurrent(self):
        actorValue = self.getActorValueFloat()
        return actorValue, time.time(), AttrQuality.ATTR_VALID

    def read_difference(self):
        difference = self.getDifference()
        return difference, time.time(), AttrQuality.ATTR_VALID

    def read_sensorValueTarget(self):
        return self.__sensorValueTarget, time.time(), AttrQuality.ATTR_VALID

    def write_sensorValueTarget(self, attr):
        self.__sensorValueTarget = attr.get_write_value()

    def read_enabled(self):
        return self.__enabled, time.time(), AttrQuality.ATTR_VALID

    def write_enabled(self, attr):
        self.__enabled = attr.get_write_value()

    @command()
    def regulateLoop(self):
        while(1):
            try:
                self.regulate()
            except Exception as e:
                print("regulate error: " + str(e))
            time.sleep(self.regulateInterval)

    def getSensorValueFloat(self):
        sensorAttribute = self.deviceSensor.read_attribute(self.SensorAttribute)
        sensorValue = sensorAttribute.value
        if(sensorAttribute.type == CmdArgType.DevString):
            sensorValue = float(sensorValue)
        return sensorValue

    def getActorValueFloat(self):
        actorValue = 0 # if controlled actor value is not derivable default to 0
        try:
            actorAttribute = self.deviceActor.read_attribute(self.ActorAttribute)
            actorValue = actorAttribute.value
            if(actorAttribute.type == CmdArgType.DevString):
                actorValue = float(actorValue)
        except Exception:
            pass
        return actorValue

    def getDifference(self):
        sensorValue = self.getSensorValueFloat()
        difference = float(self.__sensorValueTarget) - float(sensorValue) # reference - measurent
        return difference
        
    def regulate(self):
        if(self.__sensorValueTarget == self.TARGET_NO_VALUE):
            print("no sensorValueTarget given")
            return # not allowed to change again
        if(self.__enabled == False):
            print("Currently not enabled")
            return # not allowed to change again

        actorValue = self.getActorValueFloat()
        sensorValue = self.getSensorValueFloat()
        if((time.time() - self.__lastChanged ) < self.ActorMinControlInterval):
            print("no regulation: min control interval not reached")
            return # not allowed to change again
        difference = self.getDifference()
        if(abs(difference) < self.Hysteresis):
            print("no regulation: hysteresis suppression")
            return # difference is in bounds of hysterese
        
        print("current actorValue: " + str(actorValue))
        print("current sensorValue: " + str(sensorValue))
        print("current target value: " + str(self.__sensorValueTarget))
        print("difference: " + str(difference))
        
        # Calculate control signal by using PID controller
        newActorValue = self.pid(time.time(), difference)
        self.__lastChanged = time.time()
        print("changing actor to " + str(newActorValue))
        actorAttribute = self.deviceActor.read_attribute(self.ActorAttribute)
        if(actorAttribute.type == CmdArgType.DevString):
            newActorValue = str(newActorValue)
        self.deviceActor.write_attribute(self.ActorAttribute, newActorValue)
        self.save_state()

    def init_device(self):
        self.set_state(DevState.INIT)
        self.get_device_properties(self.get_device_class())
        self.deviceActor = DeviceProxy(self.ActorDevice)
        self.deviceSensor = DeviceProxy(self.SensorDevice)
        self.pid = PID(Kp=float(self.PID_kp), Ki=float(self.PID_ki), Kd=float(self.PID_kd), Tf=float(self.PID_tf))
        self.pid.set_output_limits(float(self.ActorMinValue), float(self.ActorMaxValue))
        self.pid.set_initial_value(time.time(), None, None)
        self.load_state()
        Thread(target=self.regulateLoop).start()
        self.set_state(DevState.ON)

    def save_state(self):
        state = {
            "pid": self.pid.__dict__,
            "sensorValueTarget": self.__sensorValueTarget,
            "enabled": self.__enabled,
        }
        with open(self.STATE_FILE, "w") as f:
            json.dump(state, f)

    def load_state(self):
        if not os.path.exists(self.STATE_FILE):
            return
        try:
            with open(self.STATE_FILE) as f:
                state = json.load(f)
                self.pid.__dict__.update(state.get("pid", {}))
                self.__sensorValueTarget = state.get("sensorValueTarget", self.sensorValueTargetInitial)
                self.__enabled = state.get("enabled", self.enabledInitial)
        except (OSError, JSONDecodeError):
            pass

if __name__ == "__main__":
    deviceServerName = os.getenv("DEVICE_SERVER_NAME")
    run({deviceServerName: PidController})
