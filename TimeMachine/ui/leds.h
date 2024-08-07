#include "daisy.h"

#ifndef LEDS_H_
#define LEDS_H_

namespace oam {
    namespace time_machine {
        
        class Leds {
            public:
                
                void Set(int idx, float intensity) {
                    if (initialized_ && ui_started_) {
                        leds_[idx].Set(intensity);
                        leds_[idx].Update();
                    }
                }

                void Init(
                    daisy::Pin dryPin,
                    daisy::Pin pin1, 
                    daisy::Pin pin2, 
                    daisy::Pin pin3, 
                    daisy::Pin pin4, 
                    daisy::Pin pin5, 
                    daisy::Pin pin6, 
                    daisy::Pin pin7, 
                    daisy::Pin pin8) {
                    
                    leds_[0].Init(dryPin, false);
                    leds_[1].Init(pin1, false);
                    leds_[2].Init(pin2, false);
                    leds_[3].Init(pin3, false);
                    leds_[4].Init(pin4, false);
                    leds_[5].Init(pin5, false);
                    leds_[6].Init(pin6, false);
                    leds_[7].Init(pin7, false);
                    leds_[8].Init(pin8, false);

                    initialized_ = true;

                }

                
                void InitStartupSequence() {
                    if(!initialized_) return;

                    // LED startup sequence
                    for(int i=0; i<9; i++) {
                        for(int j=0; j<9; j++) {
                            leds_[j].Set(j == i ? 1.0 : 0.0);
                            leds_[j].Update();
                        }
                        System::Delay(ledSeqDelayMs_);
                    };
                };

                void InitCalibrationSequence(int idx) {
                    if (!initialized_) return;
                    
                    for(int j=0; j<9; j++) {
                        leds_[j].Set(j == (8 - (idx % 9)) ? 1.0 : 0.0);
                        leds_[j].Update();
                    }
                }            

                void StartUi() {
                    ui_started_ = true;
                }
            private: 
                bool initialized_ = false;
                bool ui_started_ = false;
                int ledSeqDelayMs_ = 100;
                
                // LED at idx 0 is DRY
                // read heads are at 1-8
                daisy::Led leds_[9];
        };
    }
}

#endif // LEDS_H_
