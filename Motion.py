import ctypes
import ctypes.util
import os
import platform
from enum import Enum

class MotionCalibrationMode(Enum):
    Manual = 0
    Stillness = 1
    SensorFusion = 2

class Motion():
    def __init__(self):
        self._lib = None
        self._name = 'Motion'
        self._extension = ''
        self._system = platform.system()

        if self._system == 'Windows':
            self._extension = '.dll'
        elif self._system == 'Linux':
            self._extension = '.so'
        elif self._system == 'Darwin':
            self._extension = '.dylib'
        else:
            raise Exception('Your system is not known')

        # This will search JoyShockLibrary.dll in your PATH or
        # directory of your script.
        DLLPATH = ctypes.util.find_library(self._name + self._extension)
        if DLLPATH != None:
            self._lib = ctypes.cdll.LoadLibrary(DLLPATH)
        else:
            DLLPATH = './' + self._name + self._extension
            if os.path.exists(DLLPATH):
                try:
                    self._lib = ctypes.cdll.LoadLibrary(DLLPATH)
                except:
                    raise Exception(self._name + self._extension + ' Not Found')

        self._new = self._lib.newGamepadMotion
        self._new.argtypes = []
        self._new.restype = ctypes.c_void_p

        self._Reset = self._lib.GamepadMotion_Reset
        self._Reset.argtypes = [ctypes.c_void_p]
        self._Reset.restype = None

        self._ProcessMotion = self._lib.GamepadMotion_ProcessMotion
        self._ProcessMotion.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self._ProcessMotion.restype = None

        self._GetCalibratedGyro = self._lib.GamepadMotion_GetCalibratedGyro
        self._GetCalibratedGyro.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        self._GetCalibratedGyro.restype = None

        self._GetGravity = self._lib.GamepadMotion_GetGravity
        self._GetGravity.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        self._GetGravity.restype = None

        self._GetProcessedAcceleration = self._lib.GamepadMotion_GetProcessedAcceleration
        self._GetProcessedAcceleration.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        self._GetProcessedAcceleration.restype = None

        self._GetOrientation = self._lib.GamepadMotion_GetOrientation
        self._GetOrientation.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        self._GetOrientation.restype = None

        self._StartContinuousCalibration = self._lib.GamepadMotion_StartContinuousCalibration
        self._StartContinuousCalibration.argtypes = [ctypes.c_void_p]
        self._StartContinuousCalibration.restype = None

        self._PauseContinuousCalibration = self._lib.GamepadMotion_PauseContinuousCalibration
        self._PauseContinuousCalibration.argtypes = [ctypes.c_void_p]
        self._PauseContinuousCalibration.restype = None

        self._ResetContinuousCalibration = self._lib.GamepadMotion_ResetContinuousCalibration
        self._ResetContinuousCalibration.argtypes = [ctypes.c_void_p]
        self._ResetContinuousCalibration.restype = None

        self._GetCalibrationOffset = self._lib.GamepadMotion_GetCalibrationOffset
        self._GetCalibrationOffset.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        self._GetCalibrationOffset.restype = None

        self._SetCalibrationOffset = self._lib.GamepadMotion_SetCalibrationOffset
        self._SetCalibrationOffset.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self._SetCalibrationOffset.restype = None

        self._GetCalibrationMode = self._lib.GamepadMotion_GetCalibrationMode
        self._GetCalibrationMode.argtypes = [ctypes.c_void_p]
        self._GetCalibrationMode.restype = ctypes.c_int

        self._SetCalibrationMode = self._lib.GamepadMotion_SetCalibrationMode
        self._SetCalibrationMode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._SetCalibrationMode.restype = None

        self._ResetMotion = self._lib.GamepadMotion_ResetMotion
        self._ResetMotion.argtypes = [ctypes.c_void_p]
        self._ResetMotion.restype = None

        self._GetMinStillnessSamples = self._lib.GamepadMotionSettings_GetMinStillnessSamples
        self._GetMinStillnessSamples.argtypes = [ctypes.c_void_p]
        self._GetMinStillnessSamples.restype = ctypes.c_int
        self._GetMinStillnessCollectionTime = self._lib.GamepadMotionSettings_GetMinStillnessCollectionTime
        self._GetMinStillnessCollectionTime.argtypes = [ctypes.c_void_p]
        self._GetMinStillnessCollectionTime.restype = ctypes.c_float
        self._GetMinStillnessCorrectionTime = self._lib.GamepadMotionSettings_GetMinStillnessCorrectionTime
        self._GetMinStillnessCorrectionTime.argtypes = [ctypes.c_void_p]
        self._GetMinStillnessCorrectionTime.restype = ctypes.c_float
        self._GetMaxStillnessError = self._lib.GamepadMotionSettings_GetMaxStillnessError
        self._GetMaxStillnessError.argtypes = [ctypes.c_void_p]
        self._GetMaxStillnessError.restype = ctypes.c_float
        self._GetStillnessSampleDeteriorationRate = self._lib.GamepadMotionSettings_GetStillnessSampleDeteriorationRate
        self._GetStillnessSampleDeteriorationRate.argtypes = [ctypes.c_void_p]
        self._GetStillnessSampleDeteriorationRate.restype = ctypes.c_float
        self._GetStillnessErrorClimbRate = self._lib.GamepadMotionSettings_GetStillnessErrorClimbRate
        self._GetStillnessErrorClimbRate.argtypes = [ctypes.c_void_p]
        self._GetStillnessErrorClimbRate.restype = ctypes.c_float
        self._GetStillnessErrorDropOnRecalibrate = self._lib.GamepadMotionSettings_GetStillnessErrorDropOnRecalibrate
        self._GetStillnessErrorDropOnRecalibrate.argtypes = [ctypes.c_void_p]
        self._GetStillnessErrorDropOnRecalibrate.restype = ctypes.c_float
        self._GetStillnessCalibrationEaseInTime = self._lib.GamepadMotionSettings_GetStillnessCalibrationEaseInTime
        self._GetStillnessCalibrationEaseInTime.argtypes = [ctypes.c_void_p]
        self._GetStillnessCalibrationEaseInTime.restype = ctypes.c_float
        self._GetStillnessCalibrationHalfTime = self._lib.GamepadMotionSettings_GetStillnessCalibrationHalfTime
        self._GetStillnessCalibrationHalfTime.argtypes = [ctypes.c_void_p]
        self._GetStillnessCalibrationHalfTime.restype = ctypes.c_float
        self._GetStillnessGyroDelta = self._lib.GamepadMotionSettings_GetStillnessGyroDelta
        self._GetStillnessGyroDelta.argtypes = [ctypes.c_void_p]
        self._GetStillnessGyroDelta.restype = ctypes.c_float
        self._GetStillnessAccelDelta = self._lib.GamepadMotionSettings_GetStillnessAccelDelta
        self._GetStillnessAccelDelta.argtypes = [ctypes.c_void_p]
        self._GetStillnessAccelDelta.restype = ctypes.c_float
        self._GetSensorFusionCalibrationSmoothingStrength = self._lib.GamepadMotionSettings_GetSensorFusionCalibrationSmoothingStrength
        self._GetSensorFusionCalibrationSmoothingStrength.argtypes = [ctypes.c_void_p]
        self._GetSensorFusionCalibrationSmoothingStrength.restype = ctypes.c_float
        self._GetSensorFusionAngularAccelerationThreshold = self._lib.GamepadMotionSettings_GetSensorFusionAngularAccelerationThreshold
        self._GetSensorFusionAngularAccelerationThreshold.argtypes = [ctypes.c_void_p]
        self._GetSensorFusionAngularAccelerationThreshold.restype = ctypes.c_float
        self._GetSensorFusionCalibrationEaseInTime = self._lib.GamepadMotionSettings_GetSensorFusionCalibrationEaseInTime
        self._GetSensorFusionCalibrationEaseInTime.argtypes = [ctypes.c_void_p]
        self._GetSensorFusionCalibrationEaseInTime.restype = ctypes.c_float
        self._GetSensorFusionCalibrationHalfTime = self._lib.GamepadMotionSettings_GetSensorFusionCalibrationHalfTime
        self._GetSensorFusionCalibrationHalfTime.argtypes = [ctypes.c_void_p]
        self._GetSensorFusionCalibrationHalfTime.restype = ctypes.c_float
        self._GetGravityCorrectionShakinessMaxThreshold = self._lib.GamepadMotionSettings_GetGravityCorrectionShakinessMaxThreshold
        self._GetGravityCorrectionShakinessMaxThreshold.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionShakinessMaxThreshold.restype = ctypes.c_float
        self._GetGravityCorrectionShakinessMinThreshold = self._lib.GamepadMotionSettings_GetGravityCorrectionShakinessMinThreshold
        self._GetGravityCorrectionShakinessMinThreshold.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionShakinessMinThreshold.restype = ctypes.c_float
        self._GetGravityCorrectionStillSpeed = self._lib.GamepadMotionSettings_GetGravityCorrectionStillSpeed
        self._GetGravityCorrectionStillSpeed.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionStillSpeed.restype = ctypes.c_float
        self._GetGravityCorrectionShakySpeed = self._lib.GamepadMotionSettings_GetGravityCorrectionShakySpeed
        self._GetGravityCorrectionShakySpeed.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionShakySpeed.restype = ctypes.c_float
        self._GetGravityCorrectionGyroFactor = self._lib.GamepadMotionSettings_GetGravityCorrectionGyroFactor
        self._GetGravityCorrectionGyroFactor.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionGyroFactor.restype = ctypes.c_float
        self._GetGravityCorrectionGyroMinThreshold = self._lib.GamepadMotionSettings_GetGravityCorrectionGyroMinThreshold
        self._GetGravityCorrectionGyroMinThreshold.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionGyroMinThreshold.restype = ctypes.c_float
        self._GetGravityCorrectionGyroMaxThreshold = self._lib.GamepadMotionSettings_GetGravityCorrectionGyroMaxThreshold
        self._GetGravityCorrectionGyroMaxThreshold.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionGyroMaxThreshold.restype = ctypes.c_float
        self._GetGravityCorrectionMinimumSpeed = self._lib.GamepadMotionSettings_GetGravityCorrectionMinimumSpeed
        self._GetGravityCorrectionMinimumSpeed.argtypes = [ctypes.c_void_p]
        self._GetGravityCorrectionMinimumSpeed.restype = ctypes.c_float

        self._SetMinStillnessSamples = self._lib.GamepadMotionSettings_SetMinStillnessSamples
        self._SetMinStillnessSamples.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._SetMinStillnessSamples.restype = None
        self._SetMinStillnessCollectionTime = self._lib.GamepadMotionSettings_SetMinStillnessCollectionTime
        self._SetMinStillnessCollectionTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetMinStillnessCollectionTime.restype = None
        self._SetMinStillnessCorrectionTime = self._lib.GamepadMotionSettings_SetMinStillnessCorrectionTime
        self._SetMinStillnessCorrectionTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetMinStillnessCorrectionTime.restype = None
        self._SetMaxStillnessError = self._lib.GamepadMotionSettings_SetMaxStillnessError
        self._SetMaxStillnessError.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetMaxStillnessError.restype = None
        self._SetStillnessSampleDeteriorationRate = self._lib.GamepadMotionSettings_SetStillnessSampleDeteriorationRate
        self._SetStillnessSampleDeteriorationRate.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessSampleDeteriorationRate.restype = None
        self._SetStillnessErrorClimbRate = self._lib.GamepadMotionSettings_SetStillnessErrorClimbRate
        self._SetStillnessErrorClimbRate.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessErrorClimbRate.restype = None
        self._SetStillnessErrorDropOnRecalibrate = self._lib.GamepadMotionSettings_SetStillnessErrorDropOnRecalibrate
        self._SetStillnessErrorDropOnRecalibrate.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessErrorDropOnRecalibrate.restype = None
        self._SetStillnessCalibrationEaseInTime = self._lib.GamepadMotionSettings_SetStillnessCalibrationEaseInTime
        self._SetStillnessCalibrationEaseInTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessCalibrationEaseInTime.restype = None
        self._SetStillnessCalibrationHalfTime = self._lib.GamepadMotionSettings_SetStillnessCalibrationHalfTime
        self._SetStillnessCalibrationHalfTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessCalibrationHalfTime.restype = None
        self._SetStillnessGyroDelta = self._lib.GamepadMotionSettings_SetStillnessGyroDelta
        self._SetStillnessGyroDelta.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessGyroDelta.restype = None
        self._SetStillnessAccelDelta = self._lib.GamepadMotionSettings_SetStillnessAccelDelta
        self._SetStillnessAccelDelta.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetStillnessAccelDelta.restype = None
        self._SetSensorFusionCalibrationSmoothingStrength = self._lib.GamepadMotionSettings_SetSensorFusionCalibrationSmoothingStrength
        self._SetSensorFusionCalibrationSmoothingStrength.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetSensorFusionCalibrationSmoothingStrength.restype = None
        self._SetSensorFusionAngularAccelerationThreshold = self._lib.GamepadMotionSettings_SetSensorFusionAngularAccelerationThreshold
        self._SetSensorFusionAngularAccelerationThreshold.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetSensorFusionAngularAccelerationThreshold.restype = None
        self._SetSensorFusionCalibrationEaseInTime = self._lib.GamepadMotionSettings_SetSensorFusionCalibrationEaseInTime
        self._SetSensorFusionCalibrationEaseInTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetSensorFusionCalibrationEaseInTime.restype = None
        self._SetSensorFusionCalibrationHalfTime = self._lib.GamepadMotionSettings_SetSensorFusionCalibrationHalfTime
        self._SetSensorFusionCalibrationHalfTime.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetSensorFusionCalibrationHalfTime.restype = None
        self._SetGravityCorrectionShakinessMaxThreshold = self._lib.GamepadMotionSettings_SetGravityCorrectionShakinessMaxThreshold
        self._SetGravityCorrectionShakinessMaxThreshold.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionShakinessMaxThreshold.restype = None
        self._SetGravityCorrectionShakinessMinThreshold = self._lib.GamepadMotionSettings_SetGravityCorrectionShakinessMinThreshold
        self._SetGravityCorrectionShakinessMinThreshold.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionShakinessMinThreshold.restype = None
        self._SetGravityCorrectionStillSpeed = self._lib.GamepadMotionSettings_SetGravityCorrectionStillSpeed
        self._SetGravityCorrectionStillSpeed.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionStillSpeed.restype = None
        self._SetGravityCorrectionShakySpeed = self._lib.GamepadMotionSettings_SetGravityCorrectionShakySpeed
        self._SetGravityCorrectionShakySpeed.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionShakySpeed.restype = None
        self._SetGravityCorrectionGyroFactor = self._lib.GamepadMotionSettings_SetGravityCorrectionGyroFactor
        self._SetGravityCorrectionGyroFactor.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionGyroFactor.restype = None
        self._SetGravityCorrectionGyroMinThreshold = self._lib.GamepadMotionSettings_SetGravityCorrectionGyroMinThreshold
        self._SetGravityCorrectionGyroMinThreshold.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionGyroMinThreshold.restype = None
        self._SetGravityCorrectionGyroMaxThreshold = self._lib.GamepadMotionSettings_SetGravityCorrectionGyroMaxThreshold
        self._SetGravityCorrectionGyroMaxThreshold.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionGyroMaxThreshold.restype = None
        self._SetGravityCorrectionMinimumSpeed = self._lib.GamepadMotionSettings_SetGravityCorrectionMinimumSpeed
        self._SetGravityCorrectionMinimumSpeed.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self._SetGravityCorrectionMinimumSpeed.restype = None

        self._obj = self._new()
    
    def Reset(self):
        self._Reset(self._obj)
    
    def ProcessMotion(self, gx: float, gy: float, gz: float, ax: float, ay: float, az: float, deltaTime: float):
        self._ProcessMotion(self._obj, gx, gy, gz, ax, ay, az, deltaTime)
    
    def GetCalibratedGyro(self):
        x = ctypes.c_float()
        y = ctypes.c_float()
        z = ctypes.c_float()
        self._GetCalibratedGyro(self._obj, x, y, z)
        return x.value, y.value, z.value
    
    def GetGravity(self):
        x = ctypes.c_float()
        y = ctypes.c_float()
        z = ctypes.c_float()
        self._GetGravity(self._obj, x, y, z)
        return x.value, y.value, z.value
    
    def GetProcessedAcceleration(self):
        x = ctypes.c_float()
        y = ctypes.c_float()
        z = ctypes.c_float()
        self._GetProcessedAcceleration(self._obj, x, y, z)
        return x.value, y.value, z.value
    
    def GetOrientation(self):
        w = ctypes.c_float()
        x = ctypes.c_float()
        y = ctypes.c_float()
        z = ctypes.c_float()
        self._GetOrientation(self._obj, w, x, y, z)
        return w.value, x.value, y.value, z.value
    
    def StartContinuousCalibration(self):
        self._StartContinuousCalibration(self._obj)
    
    def PauseContinuousCalibration(self):
        self._PauseContinuousCalibration(self._obj)
    
    def ResetContinuousCalibration(self):
        self._ResetContinuousCalibration(self._obj)
    
    def GetCalibrationOffset(self):
        x = ctypes.c_float()
        y = ctypes.c_float()
        z = ctypes.c_float()
        self._GetCalibrationOffset(self._obj, x, y, z)
        return x.value, y.value, z.value
    
    def SetCalibrationOffset(self, xOffset: float, yOffset: float, zOffset: float):
        x = ctypes.c_float(xOffset)
        y = ctypes.c_float(yOffset)
        z = ctypes.c_float(zOffset)
        self._SetCalibrationOffset(self._obj, x, y, z)
    
    def GetCalibrationMode(self):
        return MotionCalibrationMode(self._GetCalibrationMode(self._obj))
    
    def SetCalibrationMode(self, calibrationMode: MotionCalibrationMode):
        self._SetCalibrationMode(self._obj, calibrationMode.value)

    def ResetMotion(self):
        self._ResetMotion(self._obj)
    
    def GetMinStillnessSamples(self):
        return self._GetMinStillnessSamples(self._obj)

    def GetMinStillnessCollectionTime(self):
        return self._GetMinStillnessCollectionTime(self._obj)

    def GetMinStillnessCorrectionTime(self):
        return self._GetMinStillnessCorrectionTime(self._obj)

    def GetMaxStillnessError(self):
        return self._GetMaxStillnessError(self._obj)

    def GetStillnessSampleDeteriorationRate(self):
        return self._GetStillnessSampleDeteriorationRate(self._obj)

    def GetStillnessErrorClimbRate(self):
        return self._GetStillnessErrorClimbRate(self._obj)

    def GetStillnessErrorDropOnRecalibrate(self):
        return self._GetStillnessErrorDropOnRecalibrate(self._obj)

    def GetStillnessCalibrationEaseInTime(self):
        return self._GetStillnessCalibrationEaseInTime(self._obj)

    def GetStillnessCalibrationHalfTime(self):
        return self._GetStillnessCalibrationHalfTime(self._obj)

    def GetStillnessGyroDelta(self):
        return self._GetStillnessGyroDelta(self._obj)

    def GetStillnessAccelDelta(self):
        return self._GetStillnessAccelDelta(self._obj)

    def GetSensorFusionCalibrationSmoothingStrength(self):
        return self._GetSensorFusionCalibrationSmoothingStrength(self._obj)

    def GetSensorFusionAngularAccelerationThreshold(self):
        return self._GetSensorFusionAngularAccelerationThreshold(self._obj)

    def GetSensorFusionCalibrationEaseInTime(self):
        return self._GetSensorFusionCalibrationEaseInTime(self._obj)

    def GetSensorFusionCalibrationHalfTime(self):
        return self._GetSensorFusionCalibrationHalfTime(self._obj)

    def GetGravityCorrectionShakinessMaxThreshold(self):
        return self._GetGravityCorrectionShakinessMaxThreshold(self._obj)

    def GetGravityCorrectionShakinessMinThreshold(self):
        return self._GetGravityCorrectionShakinessMinThreshold(self._obj)

    def GetGravityCorrectionStillSpeed(self):
        return self._GetGravityCorrectionStillSpeed(self._obj)

    def GetGravityCorrectionShakySpeed(self):
        return self._GetGravityCorrectionShakySpeed(self._obj)

    def GetGravityCorrectionGyroFactor(self):
        return self._GetGravityCorrectionGyroFactor(self._obj)

    def GetGravityCorrectionGyroMinThreshold(self):
        return self._GetGravityCorrectionGyroMinThreshold(self._obj)

    def GetGravityCorrectionGyroMaxThreshold(self):
        return self._GetGravityCorrectionGyroMaxThreshold(self._obj)

    def GetGravityCorrectionMinimumSpeed(self):
        return self._GetGravityCorrectionMinimumSpeed(self._obj)

    def SetMinStillnessSamples(self, value: int):
        self._SetMinStillnessSamples(self._obj, ctypes.c_int(value))

    def SetMinStillnessCollectionTime(self, value: float):
        self._SetMinStillnessCollectionTime(self._obj, ctypes.c_float(value))

    def SetMinStillnessCorrectionTime(self, value: float):
        self._SetMinStillnessCorrectionTime(self._obj, ctypes.c_float(value))

    def SetMaxStillnessError(self, value: float):
        self._SetMaxStillnessError(self._obj, ctypes.c_float(value))

    def SetStillnessSampleDeteriorationRate(self, value: float):
        self._SetStillnessSampleDeteriorationRate(self._obj, ctypes.c_float(value))

    def SetStillnessErrorClimbRate(self, value: float):
        self._SetStillnessErrorClimbRate(self._obj, ctypes.c_float(value))

    def SetStillnessErrorDropOnRecalibrate(self, value: float):
        self._SetStillnessErrorDropOnRecalibrate(self._obj, ctypes.c_float(value))

    def SetStillnessCalibrationEaseInTime(self, value: float):
        self._SetStillnessCalibrationEaseInTime(self._obj, ctypes.c_float(value))

    def SetStillnessCalibrationHalfTime(self, value: float):
        self._SetStillnessCalibrationHalfTime(self._obj, ctypes.c_float(value))

    def SetStillnessGyroDelta(self, value: float):
        self._SetStillnessGyroDelta(self._obj, ctypes.c_float(value))

    def SetStillnessAccelDelta(self, value: float):
        self._SetStillnessAccelDelta(self._obj, ctypes.c_float(value))

    def SetSensorFusionCalibrationSmoothingStrength(self, value: float):
        self._SetSensorFusionCalibrationSmoothingStrength(self._obj, ctypes.c_float(value))

    def SetSensorFusionAngularAccelerationThreshold(self, value: float):
        self._SetSensorFusionAngularAccelerationThreshold(self._obj, ctypes.c_float(value))

    def SetSensorFusionCalibrationEaseInTime(self, value: float):
        self._SetSensorFusionCalibrationEaseInTime(self._obj, ctypes.c_float(value))

    def SetSensorFusionCalibrationHalfTime(self, value: float):
        self._SetSensorFusionCalibrationHalfTime(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionShakinessMaxThreshold(self, value: float):
        self._SetGravityCorrectionShakinessMaxThreshold(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionShakinessMinThreshold(self, value: float):
        self._SetGravityCorrectionShakinessMinThreshold(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionStillSpeed(self, value: float):
        self._SetGravityCorrectionStillSpeed(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionShakySpeed(self, value: float):
        self._SetGravityCorrectionShakySpeed(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionGyroFactor(self, value: float):
        self._SetGravityCorrectionGyroFactor(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionGyroMinThreshold(self, value: float):
        self._SetGravityCorrectionGyroMinThreshold(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionGyroMaxThreshold(self, value: float):
        self._SetGravityCorrectionGyroMaxThreshold(self._obj, ctypes.c_float(value))

    def SetGravityCorrectionMinimumSpeed(self, value: float):
        self._SetGravityCorrectionMinimumSpeed(self._obj, ctypes.c_float(value))
