#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_


typedef struct {
  float measurementUncertainty;
  float estimateError;
  float processNoise;
  float lastEstimate;
} SV_KalmanFilter;


void SV_PopulateKalmanFilter(SV_KalmanFilter *, const float, const float, const float);
float SV_UpdateEstimate(SV_KalmanFilter *, const float);


#endif /* INC_KALMAN_H_ */
