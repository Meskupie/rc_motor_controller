// ATTINY85
// Clock speed: 1MHz // Prescale change from 1Mhz to 8Mhz
// In1 : PB2: INT0
// In2 : PB3: PCINT3
// Out1: PB0: OC0A
// Out2: PB1: OC0B
// BATT: PB5
// LED : PB4

#define MASTER_CLOCK_RATE 8000000.f // Prescale change from 1Mhz to 8Mhz
#define MASTER_CLOCK_SCALEING_FACTOR 0.983f
#define TIMER_PS 8.f
#define BOT_PULSEWIDTH 1.250f // (ms)
#define TOP_PULSEWIDTH 1.920f // (ms)
#define OUT_OF_BOUNDS 30.f // Expressed as a percentage of total travel
#define INPUT_TIMEOUT 0.05f // (s)
#define BATTERY_VOLTAGE_WARNING 3.3f
#define BATTERY_VOLTAGE_CRITICAL 2.7f
#define CHIP_VOLTAGE 4.2f

// Overflow count used for extended timing
volatile uint8_t timer_overflow_count;
volatile uint8_t timer_overflow_last_time;
// Pre-allocate variables to be used in the inturrupts
uint8_t cur_time_one;
uint8_t cur_overflow_one;
uint8_t cur_time_two;
uint8_t cur_overflow_two;
// Inturrupt one data
volatile uint16_t in_one_rise_time;
volatile uint16_t in_one_fall_time;
// Inturrupt two data
volatile uint16_t in_two_rise_time;
volatile uint16_t in_two_fall_time;
// State control for program execution
volatile uint8_t state_bus;
// when a bit is on, it represents the coresponding state
// bit 0: Input one PWM not initialized
// bit 1: Input two PWM not initialized
// bit 2: Input one PWM last pin state
// bit 3: Input two PWM last pin state
// bit 4: Input one PWM data ready flag
// bit 5: Input two PWM data ready flag
// bit 6: Inputs initially brought to 0 throttle together, armed
// bit 7: Battery voltage warning
volatile uint8_t error_bus;
// when a bit is on, it represents the coresponding error has occured
// bit 0: Input one PWM signal lost
// bit 1: Input two PWM signal lost
// bit 2: Input one PWM minimum pulse length exceeded
// bit 3: Input two PWM minimum pulse length exceeded
// bit 4: Input one PWM maximum pulse length exceeded
// bit 5: Input two PWM maximum pulse length exceeded
// bit 6: Input PWM phase error (missed or double inturrupt edge)
// bit 7: Battery voltage critical

// ==================================
// Timer overflow check function
// ---
// repetedly call this function with TCNT1 as the arguement to ensure rollovers are captured
// make sure to inturrupt proctect, padding this function call with: cli(); //function// sei();
// ==================================
inline void checkTimerOverflow(uint8_t polled_time){
  if(polled_time < timer_overflow_last_time){
    timer_overflow_count ++;
  }
  timer_overflow_last_time = polled_time;
}

// ==================================
// Register setup
// ---
// Setup the necessary chip registers for the deisred chip functions
// ==================================
void registerSetup() {
  // ----------------------------------
  // Setup main clock
  // ---------------------------------- 
  // bit(s) 0->3 clock prescaler option
  //             0000: No prescale
  // bit(s) 7    this bit must be set while setting all other bits to 0 to enable a prescale change
  CLKPR = B10000000;
  CLKPR = B00000000;

  // ----------------------------------
  // Setup the PWM output timer/counter
  // ---
  // Use timer0, waveform mode is set to "phase corrected PWM"
  // OCR0X is used to set the duty cycle from 0%->100% (value of 0->255)
  // Output PWM frequency is setup to be 8MHz/510
  // ----------------------------------
  // clear the ouput compare registers, no waveform output to pins when set to 0;
  OCR0A = 0;
  OCR0B = 0;
  // bit(s) 0->2 clock source selection bits
  //             001: No prescale
  // bit(s) 3    last of the 3 bits for selecting which timer/counter mode
  //             0|01:Phase Correct PWM (0xFF TOP)
  // bit(s) 6->7 force output compare (set to 0 when in PWM modes)
  // Desired: 00xx0001
  TCCR0B = (TCCR0B|B00000001)&B00110001;
  // bit(s) 0->1 first 2 of the 3 bits for selecting which timer/counter mode
  //             0|01:Phase Correct PWM (0xFF TOP)
  // bit(s) 4->7 setup OC0A/B for inverting phase corrected PWM on both coresponding pins
  // Desired: 1010xx01
  TCCR0A = (TCCR0A|B10100001)&B10101101;

  // ----------------------------------
  // Setup the timer to be used for input pulse width timing
  // ---
  // This is simply a rescaled free running counter that can be used for timing.
  // TCNT1 (0->255) returns the current timer count with an increment rate of clock_rate/prescale (8000000/16)
  // ----------------------------------
  // bit(s) 0->3 clock source selection bits
  //             0100: /8  prescale
  //             0101: /16 prescale
  // bit(s) 4->5 Comparator output mode A, setting to 00 disables outputing to a pin
  // bit(s) 6    Enable PWM A
  // bit(s) 7    Enable reset counter on compare match
  // Desired: 10000100
  TCCR1 = B10000100;
  // bit(s) 1->3 N/A
  // bit(s) 4->5 Comparator output mode B, setting to 00 disables outputing to a pin
  // bit(s) 6    Enable PWM B
  // Desired: x000xxxx
  GTCCR = GTCCR&B10001111;

  // ----------------------------------
  // Define IO Pins
  // ----------------------------------
  // bit(s) 0->5 data direction for all 6 pins (0 in, 1 out)
  // Desired: xx110011
  DDRB = (DDRB|B00110011)&B11110011;

  // ----------------------------------
  // Setup inturupts
  // ----------------------------------
  // Disable inturrupts
  cli();
  // bit(s) 0->1 detect edge change type on INT0
  // Desired: xxxxxx01
  MCUCR = (MCUCR|B00000001)&B11111101;
  // bit(s) 0->5 mask on pins to select the pins to inturrupt on change
  // Desired: xx001000
  PCMSK = (PCMSK|B00001000)&B11001000;
  // bit(s) 6 Enable INT0
  // bit(s) 5 Enable Pin change inturrupt
  // Desired: x11xxxxx
  GIMSK = GIMSK|B01100000;

  // ----------------------------------
  // Setup ADC
  // ----------------------------------
  // bit(s) 0->3 select the input to the ADC
  //             0000: ADC0(PB5)
  // bit(s) 4    bit 3 of 3 to select voltage reference
  //             0|00: Vcc reference
  // bit(s) 5    write one to left justify 10 bit data (downsample to 8 bit)
  // bit(s) 6->7 bits 1 and 2 of 3 to select voltage reference
  //             0|00: Vcc reference
  // Desired: 00100000
  ADMUX = B00100000;
  // bit(s) 0->2 ADC trigger prescaler selection
  // bit(s) 3    ADC enable inturrupt
  // bit(s) 4    ADC inturrupt flag
  // bit(s) 5    ADC auto trigger enable
  // bit(s) 6    ADC start conversion
  // bit(s) 7    ADC enable
  // Desired: 10000000
  ADCSRA = B10000000;
}

// ==================================
// Inturrut routines
// ---
// Both inturupt routines are almost identical and preform the same function for different channels.
// Inturrupts are triggered on both rising and falling edges of reciever channel outputs.
// The inturrupt routines time the gap from rise to fall and store the measurment in a global veriable.
// ==================================
ISR(INT0_vect){
  cli(); // just in case
  PORTB ^= B00010000;
  PORTB ^= B00010000;
  // grab the current time stamp
  cur_time_one = TCNT1;
  checkTimerOverflow(cur_time_one);
  cur_overflow_one = timer_overflow_count;
  // Enable inturrupts in case the other channel is out of sync. The worry here is that we get flooded by
  // nested inturrupts but given the ISR length, this should never recurse more than one level passed this.
  // This is enabled after recording the times to avoid an inturrupt occuring between those three lines
  sei();
  bool cur_pin_state = ((PINB&B00000100)==B00000100);
  if((state_bus&B00000001)==0){ // Make sure we are initialized
    if(((state_bus&B00000100)==B00000100)==cur_pin_state){ // Make sure that the new state is not the old state
      error_bus = error_bus|B01000000;
    }
    else if(cur_pin_state){ // Execute on rising edge
      // flip pin state high
      state_bus = state_bus|B00000100;
      in_one_rise_time = (cur_overflow_one<<8)+cur_time_one;
    }
    else{ // Execute on falling edge
      // flip pin state low + turn data ready flag on
      state_bus = (state_bus|B00010000)&B11111011;
      in_one_fall_time = (cur_overflow_one<<8)+cur_time_one;
    }
  }
  else if(cur_pin_state){ // Executed the first time a pin goes high
    // flip pin state high
    state_bus = state_bus|B00000100;
    in_one_rise_time = (cur_overflow_one<<8)+cur_time_one;
  }
  else if(!cur_pin_state && ((state_bus&B00000100)==B00000100)){ // Executed only on after one full pulse cycle
    // change state to pin low + channel initialized + flag off
    state_bus = state_bus&B11101010;
    in_one_fall_time = (cur_overflow_one<<8)+cur_time_one;
  }
}
ISR(PCINT0_vect){
  cli(); // just in case
  PORTB ^= B00010000;
  PORTB ^= B00010000;
  // grab the current time stamp
  cur_time_two = TCNT1;
  checkTimerOverflow(cur_time_two);
  cur_overflow_two = timer_overflow_count;
  // Enable inturrupts in case the other channel is out of sync. The worry here is that we get flooded by
  // nested inturrupts but given the ISR length, this should never recurse more than one level passed this.
  // This is enabled after recording the times to avoid an inturrupt occuring between those three lines
  sei();
  bool cur_pin_state = ((PINB&B00001000)==B00001000);
  if((state_bus&B00000010)==0){ // Make sure we are initialized
    if(((state_bus&B00001000)==B00001000)==cur_pin_state){ // Make sure that the new state is not the old state
      error_bus = error_bus|B01000000;
    }
    else if(cur_pin_state){ // Execute on rising edge
      // flip pin state high
      state_bus = state_bus|B00001000;
      in_two_rise_time = (cur_overflow_two<<8)+cur_time_two;
    }
    else{ // Execute on falling edge
      // flip pin state low + turn data ready flag on
      state_bus = (state_bus|B00100000)&B11110111;
      in_two_fall_time = (cur_overflow_two<<8)+cur_time_two;
    }
  }
  else if(cur_pin_state){ // Executed the first time a pin goes high
    // flip pin state high
    state_bus = state_bus|B00001000;
    in_two_rise_time = (cur_overflow_two<<8)+cur_time_two;
  }
  else if(!cur_pin_state && ((state_bus&B00001000)==B00001000)){ // Executed only on after one full pulse cycle
    // change state to pin low + channel initialized + flag off
    state_bus = state_bus&B11010101;
    in_two_fall_time = (cur_overflow_two<<8)+cur_time_two;
  }
}

// ====================
// Main loop
// ====================
int main() {
  // ----------------------------------
  // Initialize all registers
  // ----------------------------------
  registerSetup();

  // ----------------------------------
  // Initialize global variables
  // ----------------------------------
  in_one_rise_time = 0;
  in_one_fall_time = 0;
  in_two_rise_time = 0;
  in_two_fall_time = 0;
  state_bus = B00000011;
  error_bus = B00000000;

  // ----------------------------------
  // Pre-allocate main loop variables
  // ---------------------------------- 
  uint8_t cur_tick = 0;
  uint8_t cur_overflow = 0;
  uint16_t cur_time = 0;
  uint16_t last_time = 0;
  uint16_t bf_rise_time = 0;
  uint16_t bf_fall_time = 0;
  uint16_t delta_time = 0;
  uint8_t bf_state_bus = 0;
  int32_t bf_OCR0A = 0;
  int32_t bf_OCR0B = 0;
  uint8_t led_time_counter = 0;
  uint8_t batt_volt_index = 0;
  // initialize values at max voltage
  uint8_t batt_volt_window[8] = {255,255,255,255,255,255,255,255}; 
  uint32_t batt_volt_sum = 2040; // 255*8
  uint8_t batt_volt_average = 255;

  // ----------------------------------
  // Calculate constants
  // ---------------------------------- 
  uint16_t center_stick_int  = (uint16_t)round(((MASTER_CLOCK_RATE/1000.f)*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS)*
                               ((TOP_PULSEWIDTH+BOT_PULSEWIDTH)/2.f));
  uint16_t stick_radius_int  = (uint16_t)floor(((MASTER_CLOCK_RATE/1000.f)*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS)*
                               ((TOP_PULSEWIDTH-BOT_PULSEWIDTH)/2.f));
  uint16_t min_pulse_int     = (uint16_t)floor(((MASTER_CLOCK_RATE/1000.f)*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS)*
                               (BOT_PULSEWIDTH-(OUT_OF_BOUNDS*(TOP_PULSEWIDTH-BOT_PULSEWIDTH)/100.f)));
  uint16_t max_pulse_int     = (uint16_t)ceil(((MASTER_CLOCK_RATE/1000.f)*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS)*
                               (TOP_PULSEWIDTH+(OUT_OF_BOUNDS*(TOP_PULSEWIDTH-BOT_PULSEWIDTH)/100.f)));
  uint16_t input_timeout_int = (uint16_t)round((MASTER_CLOCK_RATE*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS)*
                               (INPUT_TIMEOUT));
  uint8_t led_s_time_threshold = round(0.125*(MASTER_CLOCK_RATE*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS/65536.f));
  uint8_t led_m_time_threshold = round(0.25*(MASTER_CLOCK_RATE*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS/65536.f));
  uint8_t led_l_time_threshold = round(0.5*(MASTER_CLOCK_RATE*MASTER_CLOCK_SCALEING_FACTOR/TIMER_PS/65536.f));
  uint8_t batt_volt_warn = (uint8_t)round(BATTERY_VOLTAGE_WARNING/CHIP_VOLTAGE*255.f);
  uint8_t batt_volt_crit = (uint8_t)round(BATTERY_VOLTAGE_CRITICAL/CHIP_VOLTAGE*255.f);

  // ----------------------------------
  // Initialize chain
  // ----------------------------------
  
  // Reset timer
  TCNT1 = 0;
  timer_overflow_last_time = 0;
  timer_overflow_count = 0;
  // Enable inturrupts
  sei();
  // Wait for inturrupt/input signal initialization
  while(!((state_bus&B00000011)==0)){
    cli();
    cur_tick = TCNT1;
    checkTimerOverflow(cur_tick);
    cur_overflow = timer_overflow_count;
    sei();
    cur_time = (cur_overflow<<8)+cur_tick;
    // Downsample main timer further for LED strobe
    if(cur_time < last_time){
      led_time_counter ++;
      if(led_time_counter >= led_s_time_threshold){
        led_time_counter = 0;
        PORTB ^= B00010000;
      }
    }
    last_time = cur_time;
  }
  PORTB = B00010000; // Turn indicator on until system is armed

  // ----------------------------------
  // Main loop
  // ----------------------------------
  for(;;){
    // ----------------------------------
    // Update channel 1
    // ----------------------------------
    // grab the current time stamp and pick an update path with inturrupts disabled
    cli(); // Disable inturrupts
    cur_tick = TCNT1;
    checkTimerOverflow(cur_tick);
    cur_overflow = timer_overflow_count;
    bf_rise_time = in_one_rise_time;
    bf_fall_time = in_one_fall_time;
    bf_state_bus = state_bus;
    sei(); // Re-enable inturrupts
    cur_time = (cur_overflow<<8)+cur_tick;
    if((bf_state_bus&B00010000)==B00010000){ // channel 1 is ready for a checkup
      state_bus &= B11101111; // reset flag
      delta_time = (bf_fall_time >= bf_rise_time)?(bf_fall_time-bf_rise_time):(bf_fall_time+1+(65535-bf_rise_time));
      if(delta_time <= min_pulse_int){
        error_bus |= B00000100;
      }
      else if(delta_time >= max_pulse_int){
        error_bus |= B00010000;
      }
      else{ // Passed bounds test
        bf_OCR0A = (int32_t)(((((int32_t)delta_time-(int32_t)center_stick_int)*32)/(int32_t)stick_radius_int)*4)+128;
      }
    }
    else if(!((bf_state_bus&B00000100)==0)){ // if we are waiting for a falling edge
      delta_time = (cur_time >= bf_rise_time)?(cur_time-bf_rise_time):(cur_time+1+(65535-bf_rise_time));
      if(delta_time >= max_pulse_int){
        error_bus |= B00010000;
      }
    }
    else{ // we are chilling waiting for a rising edge
      delta_time = (cur_time >= bf_rise_time)?(cur_time-bf_rise_time):(cur_time+1+(65535-bf_rise_time));
      if(delta_time >= input_timeout_int){
        error_bus |= B00000001;
      }
    }

    // ----------------------------------
    // Update channel 2
    // ----------------------------------
    // grab the current time stamp and pick an update path with inturrupts disabled
    cli(); // Disable inturrupts
    cur_tick = TCNT1;
    checkTimerOverflow(cur_tick);
    cur_overflow = timer_overflow_count;
    bf_rise_time = in_two_rise_time;
    bf_fall_time = in_two_fall_time;
    bf_state_bus = state_bus;
    sei(); // Re-enable inturrupts
    cur_time = (cur_overflow<<8)+cur_tick;
    if((bf_state_bus&B00100000)==B00100000){ // channel 1 is ready for a checkup
      state_bus &= B11011111; // reset flag
      delta_time = (bf_fall_time >= bf_rise_time)?(bf_fall_time-bf_rise_time):(bf_fall_time+1+(65535-bf_rise_time));
      if(delta_time <= min_pulse_int){
        error_bus |= B00001000;
      }
      else if(delta_time >= max_pulse_int){
        error_bus |= B00100000;
      }
      else{ // Passed bounds test
        bf_OCR0B = (int32_t)(((((int32_t)delta_time-(int32_t)center_stick_int)*32)/(int32_t)stick_radius_int)*4)+128;
      }
    }
    else if(!((bf_state_bus&B00001000)==0)){ // if we are waiting for a falling edge
      delta_time = (cur_time >= bf_rise_time)?(cur_time-bf_rise_time):(cur_time+1+(65535-bf_rise_time));
      if(delta_time >= max_pulse_int){
        error_bus |= B00100000;
      }
    }
    else{ // we are chilling waiting for a rising edge
      delta_time = (cur_time >= bf_rise_time)?(cur_time-bf_rise_time):(cur_time+1+(65535-bf_rise_time));
      if(delta_time >= input_timeout_int){
        error_bus |= B00000010;
      }
    }

    // ----------------------------------
    // Handle any errors
    // ---
    // If an error has occured, shut everything down and lock code in a loop.
    // Blick an error code out over the low voltage warning LED
    // ----------------------------------
    if(error_bus!=0){
      cli(); // Disable inturrupts 
      OCR0A = 0;
      OCR0B = 0;
      // Change timing clock prescaler to make timing for blick codes easier
      // bit(s) 0->3 clock source selection bits
      //             1100: /2048 prescale
      TCCR1 = (TCCR1|B00001100)&B11111100;
      // Define cycle shape, initial+9*digit+end < 255
      uint8_t interval_initial = 45;
      uint8_t interval_digit = 15;
      uint8_t interval_bin_low = 3;
      uint8_t interval_bin_high = 9;
      uint8_t interval_end_delay = 45;
      // Other
      uint8_t interval_bin = 0;
      uint8_t last_time = 0;
      // Reset timer
      TCNT1 = 0;
      timer_overflow_last_time = 0;
      timer_overflow_count = 0;
      // Lock in loop, blink code
      for(;;){
        PORTB = B00010000;
        while(timer_overflow_count < interval_initial){checkTimerOverflow(TCNT1);}
        PORTB = B00000000;
        for(int i = 0; i < 8; i++){
          // check the bits
          if(((error_bus<<i)&B10000000)==B10000000){interval_bin = interval_bin_high;}
          else{interval_bin = interval_bin_low;}
          // wait up and down
          while(timer_overflow_count < interval_initial+interval_digit+(i*interval_digit)+
          ((interval_digit-interval_bin)/2)){checkTimerOverflow(TCNT1);}
          PORTB = B00010000;
          while(timer_overflow_count < interval_initial+interval_digit+(i*interval_digit)+
          ((interval_digit-interval_bin)/2)+interval_bin){checkTimerOverflow(TCNT1);}
          PORTB = B00000000;
        }
        while(timer_overflow_count < interval_initial+(9*interval_digit)+interval_end_delay)
        {checkTimerOverflow(TCNT1);}
        timer_overflow_count = 0;
      }
    }

    // ----------------------------------
    // Output to motors if armed
    // ----------------------------------
    if((state_bus&B01000000)==B01000000){
      OCR0A = (uint8_t)((bf_OCR0A<0)?0:(bf_OCR0A>255)?255:bf_OCR0A);
      OCR0B = (uint8_t)((bf_OCR0B<0)?0:(bf_OCR0B>255)?255:bf_OCR0B);
    }
    else{ // Check if system can be armed
      if((bf_OCR0A<0)&&(bf_OCR0B<0)){ // must be less than 0 and therefore clearly inside deadband
        state_bus |= B01000000;
        PORTB &= B11101111; // Turn indicator off
      }
    }

    // ----------------------------------
    // Check battery voltage and warn if low
    // ---
    // A windowed average over the last 8 samples is used and compared to the two thresholds.
    // A warning will blick the status LED. When the voltage becomes critical, throttle is fethered off
    // To keep things simple: 
    // - cur_time is not re checked since the last update in the loop
    // - the ADC is only checked at the same rate that the warning LED would toggle 
    // ----------------------------------
    if(cur_time < last_time){
      led_time_counter ++;
      if(led_time_counter >= led_m_time_threshold){
        ADCSRA |= B01000000;
        led_time_counter = 0;
        batt_volt_sum -= batt_volt_window[batt_volt_index];
        while(!((ADCSRA&B01000000)==0)){
        }
        batt_volt_window[batt_volt_index] = ADCH;
        batt_volt_sum += batt_volt_window[batt_volt_index];
        batt_volt_average = (uint8_t)(batt_volt_sum/8); // should always be inside 0->255 after the /8
        batt_volt_index = (batt_volt_index<7)?(batt_volt_index+1):0;
        // Warn by flashing LED if votage is low
        if((state_bus&B10000000)==B10000000){PORTB ^= B00010000;}
        else if(batt_volt_average <= batt_volt_warn){state_bus|=B10000000;}
        // Throw an error if below cutoff
        if(batt_volt_average <= batt_volt_crit){error_bus|=B10000000;}
      }
    }
    last_time = cur_time;
  }
}



