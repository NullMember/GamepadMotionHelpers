// Copyright (c) 2021 Malik Enes Safak
// See LICENSE.md for more information

#ifndef __MOTION_H
#define __MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct GamepadMotion GamepadMotion;

GamepadMotion* newGamepadMotion();

void GamepadMotion_Reset(GamepadMotion* v);

void GamepadMotion_ProcessMotion(GamepadMotion* v, 
        float gyroX, float gyroY, float gyroZ,
		float accelX, float accelY, float accelZ, float deltaTime);

void GamepadMotion_GetCalibratedGyro(GamepadMotion* v, float* x, float* y, float* z);
void GamepadMotion_GetGravity(GamepadMotion* v, float* x, float* y, float* z);
void GamepadMotion_GetProcessedAcceleration(GamepadMotion* v, float* x, float* y, float* z);
void GamepadMotion_GetOrientation(GamepadMotion* v, float* w, float* x, float* y, float* z);

void GamepadMotion_StartContinuousCalibration(GamepadMotion* v);
void GamepadMotion_PauseContinuousCalibration(GamepadMotion* v);
void GamepadMotion_ResetContinuousCalibration(GamepadMotion* v);
void GamepadMotion_GetCalibrationOffset(GamepadMotion* v, float* xOffset, float* yOffset, float* zOffset);
void GamepadMotion_SetCalibrationOffset(GamepadMotion* v, float xOffset, float yOffset, float zOffset, int weight);

int GamepadMotion_GetCalibrationMode(GamepadMotion* v);
void GamepadMotion_SetCalibrationMode(GamepadMotion* v, int calibrationMode);

void GamepadMotion_ResetMotion(GamepadMotion* v);

int GamepadMotionSettings_GetMinStillnessSamples(GamepadMotion* v);
float GamepadMotionSettings_GetMinStillnessCollectionTime(GamepadMotion* v);
float GamepadMotionSettings_GetMinStillnessCorrectionTime(GamepadMotion* v);
float GamepadMotionSettings_GetMaxStillnessError(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessSampleDeteriorationRate(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessErrorClimbRate(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessErrorDropOnRecalibrate(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessCalibrationEaseInTime(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessCalibrationHalfTime(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessGyroDelta(GamepadMotion* v);
float GamepadMotionSettings_GetStillnessAccelDelta(GamepadMotion* v);
float GamepadMotionSettings_GetSensorFusionCalibrationSmoothingStrength(GamepadMotion* v);
float GamepadMotionSettings_GetSensorFusionAngularAccelerationThreshold(GamepadMotion* v);
float GamepadMotionSettings_GetSensorFusionCalibrationEaseInTime(GamepadMotion* v);
float GamepadMotionSettings_GetSensorFusionCalibrationHalfTime(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionShakinessMaxThreshold(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionShakinessMinThreshold(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionStillSpeed(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionShakySpeed(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionGyroFactor(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionGyroMinThreshold(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionGyroMaxThreshold(GamepadMotion* v);
float GamepadMotionSettings_GetGravityCorrectionMinimumSpeed(GamepadMotion* v);

void GamepadMotionSettings_SetMinStillnessSamples(GamepadMotion* v, int value);
void GamepadMotionSettings_SetMinStillnessCollectionTime(GamepadMotion* v, float value);
void GamepadMotionSettings_SetMinStillnessCorrectionTime(GamepadMotion* v, float value);
void GamepadMotionSettings_SetMaxStillnessError(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessSampleDeteriorationRate(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessErrorClimbRate(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessErrorDropOnRecalibrate(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessCalibrationEaseInTime(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessCalibrationHalfTime(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessGyroDelta(GamepadMotion* v, float value);
void GamepadMotionSettings_SetStillnessAccelDelta(GamepadMotion* v, float value);
void GamepadMotionSettings_SetSensorFusionCalibrationSmoothingStrength(GamepadMotion* v, float value);
void GamepadMotionSettings_SetSensorFusionAngularAccelerationThreshold(GamepadMotion* v, float value);
void GamepadMotionSettings_SetSensorFusionCalibrationEaseInTime(GamepadMotion* v, float value);
void GamepadMotionSettings_SetSensorFusionCalibrationHalfTime(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionShakinessMaxThreshold(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionShakinessMinThreshold(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionStillSpeed(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionShakySpeed(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionGyroFactor(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionGyroMinThreshold(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionGyroMaxThreshold(GamepadMotion* v, float value);
void GamepadMotionSettings_SetGravityCorrectionMinimumSpeed(GamepadMotion* v, float value);

#ifdef __cplusplus
}
#endif
#endif