#include "daisy_patch_sm.h"
#include "daisysp.h"

#include "dsp/stereo_time_machine.h"
#include "dsp/slew.h"
#include "dsp/clock_rate_detector.h"
#include "dsp/continuous_schmidt.h"

#include "ui/leds.h"
#include "ui/calibrator.h"
#include "ui/ui.h"

#include "time_machine_hardware.h"

using namespace daisy;
using namespace oam;
using namespace time_machine;
using namespace std;

#define TIME_SECONDS 150
#define BUFFER_WIGGLE_ROOM_SAMPLES 1000
#define DEVELOPMENT_MODE true
#define LINEAR_TIME false


// init buffers - add an extra second just in case we somehow end up slightly beyond max time
// due to precision loss in floating point arithmetic (maybe use doubles for time values???)
float DSY_SDRAM_BSS bufferLeft[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];
float DSY_SDRAM_BSS bufferRight[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];

TimeMachineHardware hw;
Ui ui;
GateIn gate;
Leds leds;

// Keep track of the agreement between the random sequence sent to the 
// switch and the value read by the ADC.
uint32_t normalization_detection_count_ = 0;
uint32_t normalization_probe_state_ = 0;

const uint8_t kNumNormalizedChannels = 4;
const uint8_t kProbeSequenceDuration = 32;
uint8_t normalization_probe_mismatches_[kNumNormalizedChannels] = {0, 0, 0, 0}; 
bool is_patched_[kNumNormalizedChannels] = {false, false, false, false};
int normalized_channels_[kNumNormalizedChannels] = { 
	VCA_1_CV,
	VCA_2_CV,
	VCA_3_CV, 
	VCA_4_CV
};

float modulation_values_[kNumNormalizedChannels] = { 
	0.0, 0.0, 0.0, 0.0
};
  
StereoTimeMachine timeMachine;

PersistentStorage<CalibrationData> calibrationDataStorage(hw.qspi);
Calibrator calibrator;



// calibration offsets for CV
float timeCvOffset = 0.0;
float feedbackCvOffset = 0.0;
float skewCvOffset = 0.0;

float normalized_offsets_[kNumNormalizedChannels] = {0.0, 0.0, 0.0, 0.0};

float finalTimeValue = 0.0;
float finalDistributionValue = 0.0;
float finalFeedbackValue = 0.0;

// delay setting LEDs for startup sequences
bool setLeds = false;

CpuLoadMeter cpuMeter;

int droppedFrames = 0;

// if modulation is patched, then slider acts as attenuverter for modulation 
// if modulation is unpatched, slider is 
// @sliderIdx is between 1 and 8 (incl.); 
float readHeadAmp(int sliderIdx, Ui ui_local) {
	float sliderAmpValue = ui_local.sliderAmpValues_[sliderIdx - 1];
	// 4 modulation inputs
	int modulationIndex = (sliderIdx - 1) / 2;
	 
	if (!is_patched_[modulationIndex]) {
		return sliderAmpValue;
	} 
	// between -1 & 1
	// -1 is silent, 1 is max volume
	float modulationValue = modulation_values_[modulationIndex];
	
	return sliderAmpValue * modulationValue;
}

// called every N samples (search for SetAudioBlockSize)
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// cpu meter measurements start
	cpuMeter.OnBlockStart();
	droppedFrames++;

	ui.ProcessAllControls();

	for(int i=0; i<9; i++) {
		float loudnessLeft = timeMachine.timeMachineLeft.GetLoudness(i);
		float loudnessRight = timeMachine.timeMachineRight.GetLoudness(i);

		leds.Set(i, max(loudnessLeft, loudnessRight));
	}

	// set time machine dry slider value, feedback, "blur" which is semi-deprecated
	timeMachine.Set(ui.drySlider, ui.feedback, ui.feedback); // controlling "blur" with feedback now???

	for(int i=1; i<9; i++) {
		// let last 8 slider time/amp/blur values for left channel time machine instance
        timeMachine.timeMachineLeft.readHeads[i-1].Set(
            spread((i / 8.0), ui.distribution) * ui.time,
            readHeadAmp(i, ui),
			max(0., ui.feedback-1.0)
        );
		// let last 8 slider time/amp/blur values for right channel time machine instance
		timeMachine.timeMachineRight.readHeads[i-1].Set(
            spread((i / 8.0), ui.distribution) * ui.time,
            readHeadAmp(i, ui),
			max(0., ui.feedback-1.0)
        );
	}

	for (size_t i = 0; i < size; i++)
	{
		// process gate for clock rate detector at audio rate (per-sample) so it calculates clock correctly
		ui.ProcessClockRate(hw.gate_in_2.State());
		
		// process input into time machine
		float* output = timeMachine.Process(in[0][i], in[1][i]);
		// set hardware output to time machine output
		out[0][i] = output[0];
		out[1][i] = output[1];
	}

	//cpu meter measurement stop
	cpuMeter.OnBlockEnd();
	droppedFrames--;
}


void DetectNormalization() {
  bool expected_value = normalization_probe_state_ >> 31;
  for (int i = 0; i < kNumNormalizedChannels; ++i) {
    int channel = normalized_channels_[i];
	float value = hw.GetAdcValue(channel);
	bool read_value;
	if (value > 4.95) {
		read_value = true;
	} else if (value < 0.05) {
		read_value = false;
	}
	else {
		++normalization_probe_mismatches_[i];
		continue;
	}

    if (expected_value != read_value) {
      ++normalization_probe_mismatches_[i];
    }
  }
  
  ++normalization_detection_count_;
  if (normalization_detection_count_ == kProbeSequenceDuration) {
    normalization_detection_count_ = 0;
    for (int i = 0; i < kNumNormalizedChannels; ++i) {
      is_patched_[i] = normalization_probe_mismatches_[i] >= 2;
      normalization_probe_mismatches_[i] = 0;
    }
  }

  normalization_probe_state_ = 1103515245 * normalization_probe_state_ + 12345;
  hw.WriteNormalization(normalization_probe_state_ >> 31);
}

int main(void)
{
	
	// init time machine hardware
    hw.Init();
	hw.StartLog(true);

	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.PrintLine("AUDIO_INITIALIZED");
	ui.Init(hw);

	calibrator.Init(&calibrationDataStorage);

	dsy_gpio_pin gatePin = DaisyPatchSM::B9;
	gate.Init(&gatePin);

	// initialize LEDs
	leds.Init(
		DaisyPatchSM::D1, 
		DaisyPatchSM::D2,
		DaisyPatchSM::D3,
		DaisyPatchSM::D4,
		DaisyPatchSM::D5,
		DaisyPatchSM::A9,
		DaisyPatchSM::D10,
		DaisyPatchSM::D7,
		DaisyPatchSM::D6);

	
	// set sample rate
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// init time machine
    timeMachine.Init(
		hw.AudioSampleRate(), 
		TIME_SECONDS + (((float)BUFFER_WIGGLE_ROOM_SAMPLES) * 0.5 / hw.AudioSampleRate()), 
		bufferLeft, 
		bufferRight);
	

	// init cpu meter
	cpuMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

	// start time machine hardware audio and logging
    //hw.StartAudio(AudioCallback);
	hw.PrintLine("AUDIO_CALLBACK_STARTED");
	//leds.InitStartupSequence();

	// Calibrate if needed
	//calibrator.Calibrate(hw, leds);

	//TODO:  CALIBRATION OFFSETS NEED INJECTING TO UI
	// timeCvOffset = savedCalibrationData.timeCvOffset;
	// skewCvOffset = savedCalibrationData.skewCvOffset;
	// feedbackCvOffset = savedCalibrationData.feedbackCvOffset;
	// normalized_offsets_[0] = savedCalibrationData.vca1CvOffset;
	// normalized_offsets_[1] = savedCalibrationData.vca2CvOffset;
	// normalized_offsets_[2] = savedCalibrationData.vca3CvOffset;
	// normalized_offsets_[3] = savedCalibrationData.vca4CvOffset;


	leds.StartUi(); 
	hw.PrintLine("UI STARTED");
	while(1) {
		//DetectNormalization(); 

		if (DEVELOPMENT_MODE) {
			// print diagnostics
			hw.PrintLine("TIME_CV: " FLT_FMT(6), FLT_VAR(6, ui.timeCv));
			hw.PrintLine("FEEDBACK_CV: " FLT_FMT(6), FLT_VAR(6, ui.feedbackCv));
			hw.PrintLine("SKEW_CV: " FLT_FMT(6), FLT_VAR(6, ui.skewCv));
			
			if (is_patched_[0]) {
				hw.PrintLine("VCA_1_CV: " FLT_FMT(6), FLT_VAR(6, ui.vca1Cv));
			} else {
				hw.PrintLine("VCA_1_CV is unpatched!");
			}
			
			if (is_patched_[1]) {
				hw.PrintLine("VCA_2_CV: " FLT_FMT(6), FLT_VAR(6, ui.vca2Cv));
			} else {
				hw.PrintLine("VCA_2_CV is unpatched!");
			}

			if (is_patched_[2]) {
				hw.PrintLine("VCA_3_CV: " FLT_FMT(6), FLT_VAR(6, ui.vca3Cv));
			} else {
				hw.PrintLine("VCA_3_CV is unpatched!");
			}

			if (is_patched_[3]) {
				hw.PrintLine("VCA_4_CV: " FLT_FMT(6), FLT_VAR(6, ui.vca4Cv));
			} else {
				hw.PrintLine("VCA_4_CV is unpatched!");
			}
			
			hw.PrintLine("TIME_KNOB: " FLT_FMT(6), FLT_VAR(6, ui.timeKnob));
			hw.PrintLine("FEEDBACK_KNOB: " FLT_FMT(6), FLT_VAR(6, ui.feedbackKnob));
			hw.PrintLine("SKEW_KNOB: " FLT_FMT(6), FLT_VAR(6, ui.skewKnob));
			hw.PrintLine("GATE IN: %d", hw.gate_in_2.State());

			hw.PrintLine("GATE IN: %d", hw.gate_in_1.State());
			hw.PrintLine("CV IN 1: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_4)));
			hw.PrintLine("CV IN 2: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_5)));
			hw.PrintLine("CV IN 3: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_6)));
			hw.PrintLine("CV IN 4: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_7)));

			// hw.PrintLine("TIME_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.timeCvOffset));
			// hw.PrintLine("FEEDBACK_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.feedbackCvOffset));
			// hw.PrintLine("SKEW_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.skewCvOffset));
			// hw.PrintLine("CALIBRATED: %d", savedCalibrationData.calibrated);

			hw.PrintLine("FINAL TIME: " FLT_FMT(6), FLT_VAR(6, finalTimeValue));
			hw.PrintLine("FINAL DISTRIBUTION: " FLT_FMT(6), FLT_VAR(6, ui.distribution));
			hw.PrintLine("FINAL FEEDBACK: " FLT_FMT(6), FLT_VAR(6, finalFeedbackValue));

			hw.PrintLine("CPU AVG: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetAvgCpuLoad()));
			hw.PrintLine("CPU MIN: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMinCpuLoad()));
			hw.PrintLine("CPU MAX: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMaxCpuLoad()));

			hw.PrintLine("DROPPED FRAMES: %d", droppedFrames);

			for(int i=0; i<9; i++) {
				hw.PrintLine("%d: " FLT_FMT(6), i, FLT_VAR(6, minMaxSlider(1.0 - hw.GetSliderValue(i))));
			}

			hw.PrintLine("");
			System::Delay(250);
		}
		
	}
}
