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
#define FFT_LENGTH			512		/**< Number of ADC bits */
#define ADC_BITS			10		/**< Number of ADC bits */
#define SAMPLING_FREQ		10000	/**< Sampling frequency in Hz */
#define SAMPLES_NUM			118		/**< Number of samples */
#define CUT_FREQ_1			1700	/**< Sampling frequency in Hz */
#define CUT_FREQ_2			2300	/**< Sampling frequency in Hz */

/* */
#define	CONCATENATE(X)		arm_cfft_sR_q15_len ## X
#define FFT_INIT(X)			CONCATENATE(X)

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

typedef struct {
	uint16_t cutBinFreq1;
	uint16_t cutBinFreq2;
	uint16_t counter;
	uint16_t acum;
	uint8_t zeros;
} app_t;

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

app_t app = {
		.cutBinFreq1 = CUT_FREQ_1 / (SAMPLING_FREQ / FFT_LENGTH),
		.cutBinFreq2 = CUT_FREQ_2 / (SAMPLING_FREQ / FFT_LENGTH),
		.counter = 0,
		.acum = 0,
		.zeros = 0
};

/* function declaration ------------------------------------------------------*/

void process(app_t * data, q15_t * array);

/* main ----------------------------------------------------------------------*/

void main(void) {
	/* Buffers */
	int16_t adc[FFT_LENGTH];
	q15_t fft[FFT_LENGTH * 2];
	q15_t fftAbs[FFT_LENGTH];

	/**/
	uint16_t sample = 0;
//	uint16_t zeros = 0;
//	uint16_t acum = 0;
//
	uint16_t cutBinInit = CUT_FREQ_1 / (SAMPLING_FREQ / FFT_LENGTH);
	uint16_t cutBinFinal= CUT_FREQ_2 / (SAMPLING_FREQ / FFT_LENGTH);

	for(; sample < FFT_LENGTH; sample++) {
		adc[sample] = 0;
		fft[sample * 2] = 0;
		fft[(sample * 2) + 1 ] = 0;
	}

	/* Initialize board peripherals */
	boardConfig();
	uartConfig(UART_USB, 460800);
	adcConfig(ADC_ENABLE);
	cyclesCounterInit(EDU_CIAA_NXP_CLOCK_SPEED);

	/* Infinite loop */
	for(;;) {
		/* Reset cycles counter */
		cyclesCounterReset();

		/* If sample is equal or greater than to number of samples, the toggle the red LED andclear sample */
		if(sample >= packet.n) {
			gpioToggle(LEDR);

			/* Process signal FFT with filter */
			arm_cfft_q15(&FFT_INIT(FFT_LENGTH), fft, 0, 1);
			arm_cmplx_mag_squared_q15(fft, fftAbs, FFT_LENGTH);
			arm_mult_q15(fftAbs, HAbs, fftAbs, FFT_LENGTH);

			/* Look for sounds into CUT_FREQ_1 and CUT_FREQ_2 frequencies, and toggle LED */
			process(&app, fftAbs);

			/* Add id and send packet */
			packet.id++;
			uartWriteByteArray(UART_USB, (uint8_t *)&packet, sizeof(packet_t));

			/* Send ADC and FFT data */
			for(sample = 0; sample < FFT_LENGTH; sample++) {
				uartWriteByteArray(UART_USB, (uint8_t *)&adc[sample], sizeof(* adc));
				uartWriteByteArray(UART_USB, (uint8_t *)&fftAbs[sample], sizeof(* fftAbs));
			}

			/* Clear samples and read ADC */
			sample = 0;
			adcRead(CH1);
		}

		/* Get ADC sample and rescale */
		adc[sample] = (((int16_t)adcRead(CH1) - 512) >> (10 - ADC_BITS)) << (6 + 10 - ADC_BITS); /* PISA el sample que se acaba de mandar con una nueva muestra */
		adc[sample] *= 10;

		/* Write ADC samples in even positions and 0 in odd positions of the FFT array */
		fft[sample * 2] = adc[sample];
		fft[(sample * 2) + 1] = 0;

		sample++;



		/* Toggle LED1 and wait while the cycles counter is less than
		 * EDU_CIAA_NXP_CLOCK_SPEED/packet.fs. EDU_CIAA_NXP_CLOCK_SPEED is 204000000 */
		gpioToggle(LED1); /* este led blinkea a fs/2 */
		while(cyclesCounterRead() < EDU_CIAA_NXP_CLOCK_SPEED/packet.fs);
	}
}

/* function definition -------------------------------------------------------*/

void process(app_t * data, q15_t * array) {
	/* Search in the bandwith the frequencies for non-zero values */
	for(uint16_t i = data->cutBinFreq2; i >= data->cutBinFreq1; i--) {
		if(array[i] > 1) {
			data->counter++;
		}
	}

	/* If two zeros are found, then reset the variables  */
	if(data->counter == 0) {
		data->zeros++;

		if(data->zeros == 2) {
			data->acum = 0;
			data->zeros = 0;
		}
	}
	else {
		data->zeros = 0;
	}

	/* Increment the acumulator and if the values is greather
	 * than 45, then toggle the LED */
	data->acum += data->counter;

	if(data->acum > 45) {
		gpioToggle(LED2);
		data->acum = 0;
	}

	/* Reset the variable */
	data->counter = 0;
}

/* end of file ---------------------------------------------------------------*/
