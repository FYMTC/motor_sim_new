# ADC DMA Troubleshooting Notes

## Symptom 1: HAL_ADC_ConvCpltCallback never fires
- **Observed behaviour**: `HAL_ADC_Start_DMA()` returned `HAL_OK`, but `HAL_ADC_ConvCpltCallback()` did not run and `g_adcDataReady` remained cleared.
- **Root cause**: the ADC DMA buffer was placed in the default `.bss` region (DTCM, `0x2000_0000`). The DMA1 controller in STM32H723 cannot access DTCM, so the transfer never completed.
- **Fix**: move DMA-visible buffers into a RAM area that DMA1 can reach (e.g. RAM_D2). In the linker script `STM32H723XG_FLASH.ld` add a `.dma_buffer` section mapped to `RAM_D2`, then place the buffer with `__attribute__((section(".dma_buffer"), aligned(4)))`.
- **Verification**: after relocation the DMA request completed, the conversion-complete interrupt fired, and the callback set `g_adcDataReady` as expected.

## Symptom 2: Main loop starving despite DMA callback toggling
- **Observed behaviour**: `HAL_ADC_ConvCpltCallback()` toggled GPIOs continuously, but the `while (1)` body in `main.c` never hit a breakpoint and `g_adcDataReady` stayed asserted.
- **Root cause**: the ADC sampling time was configured as `ADC_SAMPLETIME_1CYCLE_5`, producing conversions (and DMA interrupts) at a very high rate. The CPU spent almost all cycles servicing IRQs and never exited the ISR long enough for the main loop to clear the flag.
- **Fix**: increase the channel sampling time in `Core/Src/adc.c` (e.g. `sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;`). The longer sampling interval reduces the interrupt rate and allows the foreground loop to run normally.
- **Verification**: after lengthening the sampling time the `while (1)` loop regained execution time, breakpoints inside the loop triggered, and `g_adcDataReady` flickered between 0 and 1 as each buffer was processed.
