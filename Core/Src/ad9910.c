/****************************************************************************
 *   OFICIAL WEB-SITE:  https://gra-afch.com/product-category/rf-units/     *
 ****************************************************************************                                      
 *
 * AD9910 - DDS Power Supply 3.3V - 1.8V
 * Internal PLL up to 1.52 GHz, REF CLK Oscillator 5 - 60Mhz (PLL Enabled)
 * For More pure spectrum Recomendeted exteral oscillator 1 - 1.5 GHz
 * For signal without harmonics and the spur use an RF transformer  
 * ADT2-1T + LPF 7rd order Range: 0.4-450MHZ
 * ADT2-1T-1P + LPF 7rd order Range: 8-600MHZ
 * WBC2-1TLB + LPF 7rd order Range: 0.1(-4dB)-600MHZ(-7dB)
 * CX2147 + LPF 7rd order Range: 0.02(-3dB)-365(-3dB)/ or 0.01(-8dB)-480(-8dB)
 * Fout up 600 Mhz 
 * 17.06.2020
 * Author Grisha Anofriev e-mail: grisha.anofriev@gmail.com
******************************************************************************/
#define DEFAULT_STEP_VALUE 1000

#include "ad9910.h"
//#include "menuclk.h"
#include "main.h"

#include <math.h>
#include <float.h>

uint32_t DDS_Core_Clock = 1000000000;
int DACCurrentIndex = 0;
int16_t ClockOffset = 0;

uint32_t DAC_Current;

uint8_t strBuffer[9];

uint32_t FTW;
uint32_t *jlob;

extern uint32_t Ref_Clk;

/*****************************************************************************************
   Update - data updates from memory
*****************************************************************************************/ 
void DDS_UPDATE(void)
{
	// Required - data updates from memory
	 HAL_Delay(10);
	 GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET); // IO_UPDATE = 0
	 HAL_Delay(10);
	 GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_SET); // IO_UPDATE = 1
	 HAL_Delay(10);
	 GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET); // IO_UPDATE = 0
	 HAL_Delay(10);
}
 

/*****************************************************************************************
   F_OUT - Set Frequency in HZ 
   Num_Prof - Single Tone Mode 0..7
   Amplitude_dB - amplitude in dB from 0 to -84 (only negative values)
*****************************************************************************************/
void DDS_Fout (uint32_t F_OUT, int16_t Amplitude_dB, uint8_t Num_Prof)
{
	FTW = FreqToFTW(F_OUT);
	jlob = & FTW;

	strBuffer[0] = Num_Prof; // Single_Tone_Profile_#;

	//ASF  - Amplitude 14bit 0...16127
	strBuffer[1] =  (uint16_t)powf(10,(Amplitude_dB+84.288)/20.0) >> 8;
	strBuffer[2] =  (uint16_t)powf(10,(Amplitude_dB+84.288)/20.0);
	strBuffer[3] = 0x00;
	strBuffer[4] = 0x00;

	strBuffer[5] = *(((uint8_t*)jlob)+ 3);
	strBuffer[6] = *(((uint8_t*)jlob)+ 2);
	strBuffer[7] = *(((uint8_t*)jlob)+ 1);
	strBuffer[8] = *(((uint8_t*)jlob));

	SPI_Transmit((uint8_t*)strBuffer, 9, 1000);

	DDS_UPDATE();
  
}	

/************************************************************************************************************
 * Функция находит и взвращает такое значение STEP и StepRate при котором значение StepRate было бы целым числом, но при этом значение STEP не превысило бы значение 1000
 * входные параметры это: указатетль на переменную STEP, по адресу этого указателя будет возвращено новое значение STEP, 
 * указатетль на переменную StepRate, по адресу этого указателя будет возвращено новое значение StepRate, 
 * F_mod - частота модуляции
 ************************************************************************************************************/
void calcBestStepRate(uint16_t *Step, uint64_t *Step_Rate, uint32_t F_mod)
{
  double T_Step;
  double fStep_Rate;

  uint32_t RealDDSCoreClock=CalcRealDDSCoreClockFromOffset();
  
  T_Step = 1.0/(F_mod * *Step); // necessary time step
  fStep_Rate = (T_Step * (float)RealDDSCoreClock)/4; // the value of M for the register Step_Rate, for the desired sampling rate from RAM

  *Step_Rate=ceil(fStep_Rate);
 
  *Step=(1.0/((*Step_Rate*4)/(float)RealDDSCoreClock))/F_mod;
}



/*****************************************************************************
  Initialization DDS
  * Config SPI, Reset DDS and ReConfig SPI
  * Enable/Disable internal PLL, mux, div, charge pump current, Set VCO
  
    Input Parametr:
  * PLL - 1 enable, if 0 disable
  * Divider - This input REF CLOCK divider by 2, if 1 - Divider by 2, if 0 - Divider OFF
  * 
  * Set Current Output - 0..255, 127 - Default equal to 0 dB 
  * (если выключено, то в функцию нужно передать (или записать в управляющий регистр) 127, а если включено, то 255, и если включено, то также нужно в главном меню к значению ДБ прибавлять +4 dBM)
*****************************************************************************/
void DDS_Init(bool PLL, bool Divider, uint32_t Ref_Clk)
 {
   // It is very important for DDS AD9910 to set the initial port states
   GPIO_WritePin(DDS_MASTER_RESET_GPIO_PORT, DDS_MASTER_RESET_PIN, GPIO_PIN_SET);   // RESET = 1
   HAL_Delay(10);
   GPIO_WritePin(DDS_MASTER_RESET_GPIO_PORT, DDS_MASTER_RESET_PIN, GPIO_PIN_RESET); // RESET = 0
   GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET);       // IO_UPDATE = 0
   GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);               // CS = 1
   GPIO_WritePin(DDS_OSK_GPIO_PORT, DDS_OSK_PIN, GPIO_PIN_SET);                     // OSK = 1
   GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_RESET);
   GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
   GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_RESET);
   

   strBuffer[0] = CFR1_addr;
   strBuffer[1] = 0;// RAM_enable;//RAM_Playback_Amplitude;// | RAM_enable;//0x00; 
   strBuffer[2] = 0;//Inverse_sinc_filter_enable;//0; //Continuous_Profile_0_1; //0;//0x80;//0x00;
   strBuffer[3] = 0; //OSK_enable | Select_auto_OSK;//0x00;
   strBuffer[4] = SDIO_input_only ;
   SPI_Transmit((uint8_t*)strBuffer, 5, 1000);

   strBuffer[0] = CFR2_addr;
   strBuffer[1] = Enable_amplitude_scale_from_single_tone_profiles;//1;//0x00;
   strBuffer[2] = 0;//SYNC_CLK_enable;// | Read_effective_FTW;
   strBuffer[3] = PDCLK_enable; // 0;//0x08;//PDCLK_enable;
   strBuffer[4] = Sync_timing_validation_disable | Parallel_data_port_enable | Data_assembler_hold_last_value;
   SPI_Transmit((uint8_t*)strBuffer, 5, 1000);
   
  switch (PLL)
  {
    case false:
      /******************* External Oscillator 60 - 1000Mhz (Overclock up to 1500Mhz) ***************/ 
      strBuffer[0] = CFR3_addr;
      strBuffer[1] = 0;//DRV0_REFCLK_OUT_High_output_current;//
      strBuffer[2] = 0;
      if (Divider) strBuffer[3] = REFCLK_input_divider_ResetB;
        else strBuffer[3] = REFCLK_input_divider_ResetB | REFCLK_input_divider_bypass;
      strBuffer[4] = 0; // SYSCLK= REF_CLK * N
      SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);
      //**************************
    break;
    case true:
      /******************* External Oscillator TCXO 3.2 - 60 MHz ***********************************************/ 
      strBuffer[0] = CFR3_addr;
      if (DDS_Core_Clock<=1000000000) strBuffer[1] = VCO3;
        else strBuffer[1] = VCO5;  // | DRV0_REFCLK_OUT_High_output_current;
      strBuffer[2] = Icp387uA;   // Icp212uA, Icp237uA, Icp262uA, Icp287uA, Icp312uA, Icp337uA, Icp363uA, Icp387uA 
      strBuffer[3] = REFCLK_input_divider_ResetB | PLL_enable; // REFCLK_input_divider_bypass; //
      //strBuffer[4]=((uint32_t)DDS_Core_Clock/Ref_Clk)*2; // multiplier for PLL
      strBuffer[4]=round((float)DDS_Core_Clock/(float)Ref_Clk)*2; // multiplier for PLL
      //strBuffer[4]=round(((float)DDS_Core_Clock-ClockOffset)/(float)Ref_Clk)*2; // multiplier for PLL
      SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);
    /**********************/
    break;
  }
   
   strBuffer[0] = FSC_addr;
   strBuffer[1] = 0;
   strBuffer[2] = 0;
   strBuffer[3] = 0;
   if (DACCurrentIndex==0) strBuffer[4] = 0x7F; //DAC current Normal
    else strBuffer[4] = 0xFF;// Max carrent 255 = 31mA //DAC current HI
   SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);
   
}                

/*************************************************************************
 * Freq Out, freq_output in Hz 0 HZ - 450MHZ, 
 * amplitude_dB_output - from 0 to -84 (negative)
 ************************************************************************/
void SingleProfileFreqOut(uint32_t freq_output, int16_t amplitude_dB_output, uint8_t profile)
{
  //*** RAM Disable ***
  strBuffer[0] = CFR1_addr;
  strBuffer[1] = 0; // RAM Disable
  strBuffer[2] = 0;//
  strBuffer[3] = 0;//OSK Disable
  strBuffer[4] = SDIO_input_only ;
  SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);
  strBuffer[0] = CFR2_addr;
  strBuffer[1] = Enable_amplitude_scale_from_single_tone_profiles;//1;//0x00;
  strBuffer[2] = 0;//SYNC_CLK_enable;// | Read_effective_FTW; // Digital Ramp disable
  strBuffer[3] = PDCLK_enable; // 0;//0x08;//PDCLK_enable;
  strBuffer[4] = Sync_timing_validation_disable | Parallel_data_port_enable | Data_assembler_hold_last_value;
  SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);
  
  DDS_Fout(freq_output, amplitude_dB_output, Single_Tone_Profile_0+profile);
}


/**************
 * Return Real Core Clock based on ClockOffset 
 */
uint32_t CalcRealDDSCoreClockFromOffset()
{
  int N=DDS_Core_Clock/INIT_REFCLK;
  uint32_t rRCC = (INIT_REFCLK+ClockOffset)*N;
  return rRCC;
}

/*************************************************************************************
 * Digital Ramp Generator Enable
 **********************************************************************************/
void DigitalRamp(uint32_t FTWStart, uint32_t FTWEnd, uint32_t FTWStepSize, uint16_t StepRate, uint8_t autoclear)
{

  //*** RAM Disable
  strBuffer[0] = CFR1_addr;
  strBuffer[1] = 0;// RAM_enable; 
  strBuffer[2] = 0; 
  strBuffer[3] = 0;//OSK Disable // Autoclear_digital_ramp_accumulator;
  strBuffer[4] = SDIO_input_only;
  SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);

//*** digital Ramp LIMITS enable
  strBuffer[0] = Digital_Ramp_Limit;
  
  strBuffer[1] = FTWEnd>>24; // upper limit
  strBuffer[2] = FTWEnd>>16; // upper limit
  strBuffer[3] = FTWEnd>>8;  // upper limit
  strBuffer[4] = FTWEnd;     // upper limit
  
  strBuffer[5] = FTWStart>>24; // lower limit
  strBuffer[6] = FTWStart>>16; // lower limit
  strBuffer[7] = FTWStart>>8;  // lower limit
  strBuffer[8] = FTWStart;     // lower limit
  
  SPI_Transmit( (uint8_t*)strBuffer, 9, 1000);

  //*** digital  Ramp Step Size enable
  strBuffer[0] = Digital_Ramp_Step_Size;
  
  //**** Dicrement ****
  strBuffer[1] = FTWStepSize>>24; 
  strBuffer[2] = FTWStepSize>>16;  
  strBuffer[3] = FTWStepSize>>8; 
  strBuffer[4] = FTWStepSize; 

   //**** Increment ****
  strBuffer[5] = FTWStepSize>>24; 
  strBuffer[6] = FTWStepSize>>16;
  strBuffer[7] = FTWStepSize>>8;
  strBuffer[8] = FTWStepSize; 
  
  SPI_Transmit( (uint8_t*)strBuffer, 9, 1000);

  //*** digital  Ramp Step Rate enable
  strBuffer[0] = Digital_Ramp_Rate;
  
  strBuffer[1] = StepRate>>8; // Negative Slope Rate
  strBuffer[2] = StepRate; // Negative Slope Rate
  strBuffer[3] = StepRate>>8; // Positive Slope Rate
  strBuffer[4] = StepRate; // Positive Slope Rate
  
  SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);

  //*** digital Ramp Generator enable
  strBuffer[0] = CFR2_addr;
  strBuffer[1] = 0;
  if(autoclear){
	  strBuffer[2] = Digital_Ramp_Destination_Frequency | Digital_ramp_enable | Digital_ramp_no_dwell_high | Digital_ramp_no_dwell_low;
  }else{
	  strBuffer[2] = Digital_Ramp_Destination_Frequency | Digital_ramp_enable;
  }
  strBuffer[3] = PDCLK_enable; // 0;//0x08;//PDCLK_enable;
  strBuffer[4] = Sync_timing_validation_disable | Parallel_data_port_enable | Data_assembler_hold_last_value;
  SPI_Transmit( (uint8_t*)strBuffer, 5, 1000);

  DDS_UPDATE();
}

//SweepTimeFormat: 0 - Seconds, 1 - Milliseconds (mS), 2 - MicroSeconds (uS), 3 - NanoSeconds
void Sweep(uint32_t StartSweepFreq, uint32_t StopSweepFreq, uint16_t SweepTime, uint8_t autoclear)
{
  //float64_t NanoSweepTime=f64((uint32_t)SweepTime); //F64_div, f64_mul
  uint64_t NanoSweepTime = SweepTime;
  uint64_t CaluclatedNanoSweepTime;
  uint32_t FTWStart=FreqToFTW(StartSweepFreq);
  uint32_t FTWEnd=FreqToFTW(StopSweepFreq);
  uint32_t DeltaFTW=FTWEnd-FTWStart;
  uint32_t FTWStepSize=1;
  uint16_t StepRate=1;

  NanoSweepTime=SweepTime*1E6;

  float GHZ_CoreClock=DDS_Core_Clock/1E9; //незабыть заменит на realDDSCoreClock
  CaluclatedNanoSweepTime=(4/GHZ_CoreClock*DeltaFTW*FTWStepSize);

  if (CaluclatedNanoSweepTime<NanoSweepTime)
  {
    uint32_t StepRateMultiplier=(uint32_t)round((float)NanoSweepTime/(float)CaluclatedNanoSweepTime);
    if (StepRateMultiplier<=0xFFFF) StepRate=StepRate*StepRateMultiplier;
      else StepRate=0xFFFF;
  }
  if (CaluclatedNanoSweepTime>NanoSweepTime)
  {
    uint32_t FTWMultiplier=(uint32_t)round((float)CaluclatedNanoSweepTime/(float)NanoSweepTime);
    FTWStepSize=FTWStepSize*FTWMultiplier;
  }
  DigitalRamp(FTWStart, FTWEnd, FTWStepSize, StepRate, autoclear);
}

uint32_t FreqToFTW(uint32_t freq){
	uint32_t RealDDSCoreClock=CalcRealDDSCoreClockFromOffset();
	return (uint32_t)((4294967296.0 *((float)freq / (float)RealDDSCoreClock)));
	   // FTW = round(4294967296.0 *((float)*F_OUT / ((float)RealDDSCoreClock-ClockOffset))); // закомментировано 27.05.2020

	   // значение FTW должно быть 4294967296
	//    float64_t f64F_OUT=f64(*F_OUT);
	//    float64_t f64CoreClock=f64(RealDDSCoreClock);
	//    float64_t TwoPower32=f64(4294967295UL);
	//    float64_t f64FTW;
	//    //float64_t Offset=f64(ClockOffset);
	//    //CoreClock=f64_sub(CoreClock, Offset);
	//    f64FTW=f64_div(TwoPower32, f64CoreClock);
	//    f64FTW=f64_mul(f64FTW, f64F_OUT);
	//   //FTW = 4294967296 * (*F_OUT / (float)tmpCoreClock);
	//   bool a;
	//   uint_fast8_t softfloat_roundingMode;
	//   softfloat_roundingMode=softfloat_round_near_maxMag;
	//   FTW=f64_to_ui32(f64FTW, softfloat_roundingMode, a);
	//
}
