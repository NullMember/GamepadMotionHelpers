// Copyright (c) 2021 Malik Enes Safak
// See LICENSE.md for more information

#include "GamepadMotion.hpp"
#include "Motion.h"

extern "C" {

    GamepadMotion* newGamepadMotion(){
        return new GamepadMotion();
    }

    void GamepadMotion_Reset(GamepadMotion* v){
        v->Reset();
    }

    void GamepadMotion_ProcessMotion(GamepadMotion* v, 
            float gyroX, float gyroY, float gyroZ,
            float accelX, float accelY, float accelZ, float deltaTime){
                v->ProcessMotion(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, deltaTime);
            }

    void GamepadMotion_GetCalibratedGyro(GamepadMotion* v, float* x, float* y, float* z){
        v->GetCalibratedGyro(x, y, z);
    }
    void GamepadMotion_GetGravity(GamepadMotion* v, float* x, float* y, float* z){
        v->GetGravity(x, y, z);
    }
    void GamepadMotion_GetProcessedAcceleration(GamepadMotion* v, float* x, float* y, float* z){
        v->GetProcessedAcceleration(x, y, z);
    }
    void GamepadMotion_GetOrientation(GamepadMotion* v, float* w, float* x, float* y, float* z){
        v->GetOrientation(w, x, y, z);
    }

    void GamepadMotion_StartContinuousCalibration(GamepadMotion* v){
        v->StartContinuousCalibration();
    }
    void GamepadMotion_PauseContinuousCalibration(GamepadMotion* v){
        v->PauseContinuousCalibration();
    }
    void GamepadMotion_ResetContinuousCalibration(GamepadMotion* v){
        v->ResetContinuousCalibration();
    }
    void GamepadMotion_GetCalibrationOffset(GamepadMotion* v, float* xOffset, float* yOffset, float* zOffset){
        v->GetCalibrationOffset(xOffset, yOffset, zOffset);
    }
    void GamepadMotion_SetCalibrationOffset(GamepadMotion* v, float xOffset, float yOffset, float zOffset, int weight){
        v->SetCalibrationOffset(xOffset, yOffset, zOffset, weight);
    }

    int GamepadMotion_GetCalibrationMode(GamepadMotion* v){
        return (int)v->GetCalibrationMode();
    }

    void GamepadMotion_SetCalibrationMode(GamepadMotion* v, int calibrationMode){
        v->SetCalibrationMode((GamepadMotionHelpers::CalibrationMode)calibrationMode);
    }

    void GamepadMotion_ResetMotion(GamepadMotion* v){
        v->ResetMotion();
    }

    int GamepadMotionSettings_GetMinStillnessSamples(GamepadMotion* v){
        return v->Settings.MinStillnessSamples;
    }
    float GamepadMotionSettings_GetMinStillnessCollectionTime(GamepadMotion* v){
        return v->Settings.MinStillnessCollectionTime;
    }
    float GamepadMotionSettings_GetMinStillnessCorrectionTime(GamepadMotion* v){
        return v->Settings.MinStillnessCorrectionTime;
    }
    float GamepadMotionSettings_GetMaxStillnessError(GamepadMotion* v){
        return v->Settings.MaxStillnessError;
    }
    float GamepadMotionSettings_GetStillnessSampleDeteriorationRate(GamepadMotion* v){
        return v->Settings.StillnessSampleDeteriorationRate;
    }
    float GamepadMotionSettings_GetStillnessErrorClimbRate(GamepadMotion* v){
        return v->Settings.StillnessErrorClimbRate;
    }
    float GamepadMotionSettings_GetStillnessErrorDropOnRecalibrate(GamepadMotion* v){
        return v->Settings.StillnessErrorDropOnRecalibrate;
    }
    float GamepadMotionSettings_GetStillnessCalibrationEaseInTime(GamepadMotion* v){
        return v->Settings.StillnessCalibrationEaseInTime;
    }
    float GamepadMotionSettings_GetStillnessCalibrationHalfTime(GamepadMotion* v){
        return v->Settings.StillnessCalibrationHalfTime;
    }
    float GamepadMotionSettings_GetStillnessGyroDelta(GamepadMotion* v){
        return v->Settings.StillnessGyroDelta;
    }
    float GamepadMotionSettings_GetStillnessAccelDelta(GamepadMotion* v){
        return v->Settings.StillnessAccelDelta;
    }
    float GamepadMotionSettings_GetSensorFusionCalibrationSmoothingStrength(GamepadMotion* v){
        return v->Settings.SensorFusionCalibrationSmoothingStrength;
    }
    float GamepadMotionSettings_GetSensorFusionAngularAccelerationThreshold(GamepadMotion* v){
        return v->Settings.SensorFusionAngularAccelerationThreshold;
    }
    float GamepadMotionSettings_GetSensorFusionCalibrationEaseInTime(GamepadMotion* v){
        return v->Settings.SensorFusionCalibrationEaseInTime;
    }
    float GamepadMotionSettings_GetSensorFusionCalibrationHalfTime(GamepadMotion* v){
        return v->Settings.SensorFusionCalibrationHalfTime;
    }
    float GamepadMotionSettings_GetGravityCorrectionShakinessMaxThreshold(GamepadMotion* v){
        return v->Settings.GravityCorrectionShakinessMaxThreshold;
    }
    float GamepadMotionSettings_GetGravityCorrectionShakinessMinThreshold(GamepadMotion* v){
        return v->Settings.GravityCorrectionShakinessMinThreshold;
    }
    float GamepadMotionSettings_GetGravityCorrectionStillSpeed(GamepadMotion* v){
        return v->Settings.GravityCorrectionStillSpeed;
    }
    float GamepadMotionSettings_GetGravityCorrectionShakySpeed(GamepadMotion* v){
        return v->Settings.GravityCorrectionShakySpeed;
    }
    float GamepadMotionSettings_GetGravityCorrectionGyroFactor(GamepadMotion* v){
        return v->Settings.GravityCorrectionGyroFactor;
    }
    float GamepadMotionSettings_GetGravityCorrectionGyroMinThreshold(GamepadMotion* v){
        return v->Settings.GravityCorrectionGyroMinThreshold;
    }
    float GamepadMotionSettings_GetGravityCorrectionGyroMaxThreshold(GamepadMotion* v){
        return v->Settings.GravityCorrectionGyroMaxThreshold;
    }
    float GamepadMotionSettings_GetGravityCorrectionMinimumSpeed(GamepadMotion* v){
        return v->Settings.GravityCorrectionMinimumSpeed;
    }

    void GamepadMotionSettings_SetMinStillnessSamples(GamepadMotion* v, int value){
        v->Settings.MinStillnessSamples = value;
    }
    void GamepadMotionSettings_SetMinStillnessCollectionTime(GamepadMotion* v, float value){
        v->Settings.MinStillnessCollectionTime = value;
    }
    void GamepadMotionSettings_SetMinStillnessCorrectionTime(GamepadMotion* v, float value){
        v->Settings.MinStillnessCorrectionTime = value;
    }
    void GamepadMotionSettings_SetMaxStillnessError(GamepadMotion* v, float value){
        v->Settings.MaxStillnessError = value;
    }
    void GamepadMotionSettings_SetStillnessSampleDeteriorationRate(GamepadMotion* v, float value){
        v->Settings.StillnessSampleDeteriorationRate = value;
    }
    void GamepadMotionSettings_SetStillnessErrorClimbRate(GamepadMotion* v, float value){
        v->Settings.StillnessErrorClimbRate = value;
    }
    void GamepadMotionSettings_SetStillnessErrorDropOnRecalibrate(GamepadMotion* v, float value){
        v->Settings.StillnessErrorDropOnRecalibrate = value;
    }
    void GamepadMotionSettings_SetStillnessCalibrationEaseInTime(GamepadMotion* v, float value){
        v->Settings.StillnessCalibrationEaseInTime = value;
    }
    void GamepadMotionSettings_SetStillnessCalibrationHalfTime(GamepadMotion* v, float value){
        v->Settings.StillnessCalibrationHalfTime = value;
    }
    void GamepadMotionSettings_SetStillnessGyroDelta(GamepadMotion* v, float value){
        v->Settings.StillnessGyroDelta = value;
    }
    void GamepadMotionSettings_SetStillnessAccelDelta(GamepadMotion* v, float value){
        v->Settings.StillnessAccelDelta = value;
    }
    void GamepadMotionSettings_SetSensorFusionCalibrationSmoothingStrength(GamepadMotion* v, float value){
        v->Settings.SensorFusionCalibrationSmoothingStrength = value;
    }
    void GamepadMotionSettings_SetSensorFusionAngularAccelerationThreshold(GamepadMotion* v, float value){
        v->Settings.SensorFusionAngularAccelerationThreshold = value;
    }
    void GamepadMotionSettings_SetSensorFusionCalibrationEaseInTime(GamepadMotion* v, float value){
        v->Settings.SensorFusionCalibrationEaseInTime = value;
    }
    void GamepadMotionSettings_SetSensorFusionCalibrationHalfTime(GamepadMotion* v, float value){
        v->Settings.SensorFusionCalibrationHalfTime = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionShakinessMaxThreshold(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionShakinessMaxThreshold = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionShakinessMinThreshold(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionShakinessMinThreshold = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionStillSpeed(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionStillSpeed = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionShakySpeed(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionShakySpeed = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionGyroFactor(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionGyroFactor = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionGyroMinThreshold(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionGyroMinThreshold = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionGyroMaxThreshold(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionGyroMaxThreshold = value;
    }
    void GamepadMotionSettings_SetGravityCorrectionMinimumSpeed(GamepadMotion* v, float value){
        v->Settings.GravityCorrectionMinimumSpeed = value;
    }
}