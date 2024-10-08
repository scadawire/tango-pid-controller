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

    sensorValueCurrent = attribute(label="sensorValueCurrent", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ, polling_period=1000,
        unit="_", format="8.4f")

    actorValueCurrent = attribute(label="actorValueCurrent", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ, polling_period=1000,
        unit="_", format="8.4f")

    sensorValueTarget = attribute(label="sensorValueTarget", dtype=float,
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ_WRITE, polling_period=1000,
        unit="_", format="8.4f")
    
    __sensorValueTarget = 0

    ActorDevice = device_property(dtype=str, default_value="")
    ActorAttribute = device_property(dtype=str, default_value="")
    SensorDevice = device_property(dtype=str, default_value="")
    SensorAttribute = device_property(dtype=str, default_value="")
    Hysterese = device_property(dtype=float, default_value=0)
    ActorMinControlInterval = device_property(dtype=float, default_value=0)
    ActorMinValue = device_property(dtype=float, default_value=-10)
    ActorMaxValue = device_property(dtype=float, default_value=10)
    PID_kp = device_property(dtype=float, default_value=2.0)
    PID_ki = device_property(dtype=float, default_value=0.1)
    PID_kd = device_property(dtype=float, default_value=1.0)
    PID_tf = device_property(dtype=float, default_value=0.05)
    regulateInterval = device_property(dtype=float, default_value=1)
    sensorValueTargetInit = device_property(dtype=float, default_value=0)
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
        
    def read_sensorValueTarget(self):
        return self.__sensorValueTarget, time.time(), AttrQuality.ATTR_VALID

    def write_sensorValueTarget(self, _sensorValueTarget):
        print("sensorValueTarget set to %f" % _sensorValueTarget)
        self.__sensorValueTarget = _sensorValueTarget
        self.push_change_event("sensorValueTarget", self.__sensorValueTarget)

    @command()
    def regulateLoop(self):
        while(1):
            self.regulate()
            time.sleep(self.regulateInterval)

    def getSensorValueFloat(self):
        sensorAttribute = self.deviceSensor.read_attribute(self.SensorAttribute)
        sensorValue = sensorAttribute.value
        if(sensorAttribute.type == CmdArgType.DevString):
            sensorValue = float(sensorValue)
        return sensorValue

    def getActorValueFloat(self):
        actorAttribute = self.deviceActor.read_attribute(self.ActorAttribute)
        actorValue = actorAttribute.value
        if(actorAttribute.type == CmdArgType.DevString):
            actorValue = float(actorValue)
        return actorValue
        
    def regulate(self):
        actorValue = self.getActorValueFloat()
        sensorValue = self.getSensorValueFloat()
        if((time.time() - self.__lastChanged ) < self.ActorMinControlInterval):
            print("no regulation: min control interval not reached")
            return # not allowed to change again
        difference = float(sensorValue) - float(self.__sensorValueTarget)
        if(abs(difference) < self.Hysterese):
            print("no regulation: hysterese suppression")
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

    def init_device(self):
        self.set_state(DevState.INIT)
        self.get_device_properties(self.get_device_class())
        self.deviceActor = DeviceProxy(self.ActorDevice)
        self.deviceSensor = DeviceProxy(self.SensorDevice)
        self.__sensorValueTarget = self.sensorValueTargetInit
        self.pid = PID(Kp=float(self.PID_kp), Ki=float(self.PID_ki), Kd=float(self.PID_kd), Tf=float(self.PID_tf))
        self.pid.set_output_limits(float(self.ActorMinValue), float(self.ActorMaxValue))
        Thread(target=self.regulateLoop).start()
        self.set_state(DevState.ON)

if __name__ == "__main__":
    deviceServerName = os.getenv("DEVICE_SERVER_NAME")
    run({deviceServerName: PidController})
