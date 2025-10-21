# ADC DMA 故障排查笔记

## 症状1: HAL_ADC_ConvCpltCallback 从未触发
- **观察到的行为**: `HAL_ADC_Start_DMA()` 返回 `HAL_OK`，但 `HAL_ADC_ConvCpltCallback()` 没有运行，并且 `g_adcDataReady` 保持清除状态。
- **根本原因**: ADC DMA 缓冲区被放置在默认的 `.bss` 区域（DTCM，`0x2000_0000`）。STM32H723 中的 DMA1 控制器无法访问 DTCM，因此传输从未完成。
- **修复方法**: 将 DMA 可见的缓冲区移动到 DMA1 可以访问的 RAM 区域（例如 RAM_D2）。在链接脚本 `STM32H723XG_FLASH.ld` 中添加映射到 `RAM_D2` 的 `.dma_buffer` 段，然后使用 `__attribute__((section(".dma_buffer"), aligned(4)))` 放置缓冲区。
- **验证**: 重新定位后，DMA 请求完成，转换完成中断触发，回调按预期设置了 `g_adcDataReady`。

## 症状2: 尽管 DMA 回调持续切换，但主循环饥饿
- **观察到的行为**: `HAL_ADC_ConvCpltCallback()` 持续切换 GPIO，但 `main.c` 中的 `while (1)` 主体从未命中断点，并且 `g_adcDataReady` 保持置位状态。
- **根本原因**: ADC 采样时间被配置为 `ADC_SAMPLETIME_1CYCLE_5`，以非常高的速率产生转换（和 DMA 中断）。CPU 几乎将所有周期都用于服务 IRQ，从未退出 ISR 足够长的时间以让主循环清除标志。
- **修复方法**: 增加 `Core/Src/adc.c` 中的通道采样时间（例如 `sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;`）。更长的采样间隔降低了中断速率，并允许前台循环正常运行。
- **验证**: 延长采样时间后，`while (1)` 循环重新获得了执行时间，循环内的断点被触发，并且 `g_adcDataReady` 在每个缓冲区被处理时在 0 和 1 之间闪烁。

