//--------------------------------------------------------------
// Simple library use kalman filter
//--------------------------------------------------------------



//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include <kalman.h>



//--------------------------------------------------------------
// Function definitions
//--------------------------------------------------------------

//Initialize kalman structure
void SV_PopulateKalmanFilter(SV_KalmanFilter *kalmanFilter, const float measurementUncertainty, const float estimateError, const float processNoise)
    {
	kalmanFilter->measurementUncertainty = measurementUncertainty;
	kalmanFilter->estimateError = estimateError;
	kalmanFilter->processNoise = processNoise;
	kalmanFilter->lastEstimate = 0;
    }

//Update filtred value
float SV_UpdateEstimate(SV_KalmanFilter *kalmanFilter, const float measurement)
    {
	float kalmanGain = 0;
	float currentEstimate = 0;

	kalmanGain      = kalmanFilter->estimateError / (kalmanFilter->estimateError + kalmanFilter->measurementUncertainty);
	currentEstimate = kalmanFilter->lastEstimate + kalmanGain * (measurement - kalmanFilter->lastEstimate);

	if(kalmanFilter->lastEstimate > currentEstimate)
	{
	    kalmanFilter->estimateError = (1.0 - kalmanGain) * kalmanFilter->estimateError + (kalmanFilter->lastEstimate - currentEstimate) * kalmanFilter->processNoise;
	}
	else
	{
	    kalmanFilter->estimateError = (1.0 - kalmanGain) * kalmanFilter->estimateError + (currentEstimate - kalmanFilter->lastEstimate) * kalmanFilter->processNoise;
	}

	kalmanFilter->lastEstimate = currentEstimate;
	return currentEstimate;
    }
