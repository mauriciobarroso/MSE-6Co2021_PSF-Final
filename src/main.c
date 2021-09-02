/*
 * main.c
 *
 * Created on: Aug 17, 2021
 * Author: Mauricio Barroso Benavides
 */

/* inclusions ----------------------------------------------------------------*/

#include "sapi.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "fir.h"

/* macros --------------------------------------------------------------------*/
#define ADC_BITS		10		/**< Number of ADC bits */
#define SAMPLING_FREQ	10000	/**< Sampling frequency in Hz */
#define SAMPLES_NUM		H_PADD_LENGTH - h_LENGTH + 1	/**< Number of samples */
#define CUT_FREQ_1		1700	/**< Sampling frequency in Hz */
#define CUT_FREQ_2		2300	/**< Sampling frequency in Hz */

/* typedef -------------------------------------------------------------------*/

/* Packet to send structure
 * TODO: write descriptions of the members*/
typedef struct {
	char pre[8];	/**< */
	uint32_t id;	/**< */
	uint16_t n;		/**< */
	uint16_t fs;	/**< */
	uint16_t cutFreq1;
	uint16_t cutFreq2;
	uint16_t m;		/**< */
	uint16_t counter;
	char pos[4];	/**< */
} __attribute__ ((packed)) packet_t;

/* data declaration ----------------------------------------------------------*/

packet_t packet = {
		.pre = "*header*",
		.id = 0,
		.n = SAMPLES_NUM,
		.fs = SAMPLING_FREQ,
		.cutFreq1 = CUT_FREQ_1,
		.cutFreq2 = CUT_FREQ_2,
		.m = h_LENGTH,
		.counter = 0,
		.pos = "end*"
};

/* function declaration ------------------------------------------------------*/

/* main ----------------------------------------------------------------------*/

void main(void) {
	uint16_t sample = 0;
	int16_t adc[packet.n + packet.m - 1];
	q15_t fftInOut[(packet.n + packet.m - 1) * 2];
	q15_t fftAbs[packet.n + packet.m - 1];

	int cutBinInit = CUT_FREQ_1 / (packet.fs / (packet.n + packet.m - 1));
	int cutBinFinal= CUT_FREQ_2 / (packet.fs / (packet.n + packet.m - 1));
	uint16_t counter = cutBinFinal;
	uint16_t iter = 0;
	uint16_t acum = 0;

	for(; sample < packet.n + packet.m - 1; sample++) {
		adc[sample] = 0;
		fftInOut[sample * 2] = 0;
		fftInOut[(sample * 2) + 1 ] = 0;
	}

	/* Initialize board peripherals */
	boardConfig();
	uartConfig(UART_USB, 460800);
	adcConfig(ADC_ENABLE);
//	dacConfig(DAC_ENABLE);
	cyclesCounterInit(EDU_CIAA_NXP_CLOCK_SPEED);

	/* Infinite loop */
	for(;;) {
		/* Reset cycles counter */
		cyclesCounterReset();

		/* If sample is equal or greater than to number of samples, the toggle the red LED andclear sample */
		if(sample >= packet.n) {
			gpioToggle(LEDR);

			/*  */
			arm_cfft_q15(&arm_cfft_sR_q15_len512, fftInOut, 0, 1);
			arm_cmplx_mag_squared_q15(fftInOut, fftAbs, packet.n + packet.m - 1);
			arm_mult_q15(fftAbs, HAbs, fftAbs, packet.n + packet.m - 1);

//			for(uint16_t i = 0; i < (packet.n + packet.m - 1) / 2; i++) {
//				if(i >= cutBinInit && i <= cutBinFinal) {
//					if(fftAbs[i] > 1) {
//						gpioToggle(LED2);
//					}
//				}
//			}

//			while(counter >= cutBinInit) {
////				iter++;
//
//				if(fftAbs[counter] > 0) {
//					counter--;
//
//					packet.counter++;
//
//					if(packet.counter++ >= 5) {
//						gpioToggle(LED2);
//						counter = cutBinFinal;
//						break;
//					}
//				}
//				else {
//					break;
//				}
//
//			}


//			if(iter == 20) {
//				counter = cutBinFinal;
//				iter = 0;
//			}


			for(uint16_t i = cutBinFinal; i >= cutBinInit; i--) {
				if(fftAbs[i] > 1) {
					packet.counter++;
				}
			}

			if(packet.counter == 0) {
				iter++;

				if(iter == 2) {
					acum = 0;
					iter = 0;
				}
			}

			acum += packet.counter;

			if(acum  > 45) {
				gpioToggle(LED2);
				acum = 0;
			}

			/**/
			packet.id++;
			uartWriteByteArray(UART_USB, (uint8_t *)&packet, sizeof(packet_t));

			/* Send ADC and DFT data */
			for(sample = 0; sample < packet.n + packet.m - 1; sample++) {
				uartWriteByteArray(UART_USB, (uint8_t *)&adc[sample], sizeof(* adc));
				uartWriteByteArray(UART_USB, (uint8_t *)&fftAbs[sample], sizeof(* fftAbs));
			}

			/* Clear ... */
			packet.counter = 0;
			sample = 0;
			adcRead(CH1);
		}

		/* Get ADC sample */
		adc[sample] = (((int16_t)adcRead(CH1) - 512) >> (10 - ADC_BITS)) << (6 + 10 - ADC_BITS); /* PISA el sample que se acaba de mandar con una nueva muestra */
		adc[sample] *= 10;
		/* Write ADC samples in even positions and 0 in odd positions of the FFT array */
		fftInOut[sample * 2] = adc[sample];
		fftInOut[(sample * 2) + 1] = 0;

		sample++;



		/* Toggle LED1 and wait while the cycles counter is less than
		 * EDU_CIAA_NXP_CLOCK_SPEED/packet.fs * 1000. EDU_CIAA_NXP_CLOCK_SPEED is 204000000 */
		gpioToggle(LED1); /* este led blinkea a fs/2 */
		while(cyclesCounterRead() < EDU_CIAA_NXP_CLOCK_SPEED/packet.fs);
	}
}

/* function definition -------------------------------------------------------*/

/* end of file ---------------------------------------------------------------*/
