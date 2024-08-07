#ifndef CALIBRATION_DATA_H_
#define CALIBRATION_DATA_H_

//Setting Struct containing parameters we want to save to flash
struct CalibrationData {
	float timeCvOffset = 0.0;
	float skewCvOffset = 0.0;
	float feedbackCvOffset = 0.0;
	
	float vca1CvOffset = 0.0;
    float vca2CvOffset = 0.0;
    float vca3CvOffset = 0.0;
    float vca4CvOffset = 0.0;
	
	int calibrated = false;

	//Overloading the != operator
	//This is necessary as this operator is used in the PersistentStorage source code
	bool operator!=(const CalibrationData& a) const {
        return !(
				a.timeCvOffset==timeCvOffset && \
				a.skewCvOffset==skewCvOffset && \
				a.feedbackCvOffset==feedbackCvOffset && \
				a.vca1CvOffset==vca1CvOffset && \
				a.vca2CvOffset==vca2CvOffset && \
				a.vca3CvOffset==vca3CvOffset && \
				a.vca4CvOffset==vca4CvOffset && \
				a.calibrated==calibrated
			);
    }
};

#endif // CALIBRATION_DATA_H_