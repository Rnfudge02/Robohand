/*!
 * \file Robohand_dma.h
 * \brief Robohand DMA backend, complicated, extremely performant.
 * \details Work in progress featureset.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_DMA_H
#define ROBOHAND_DMA_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#define USE_DMA false                                           ///< Use DMA for system communication

/** @defgroup dma_config DMA Configuration
 *  @brief Constants and definitions for DMA operations.
 *  @{
 */
 
#define DMA_CHANNEL_I2C 0                                       ///< DMA channel for I2C transfers
#define DMA_CHANNEL_ADC 1                                       ///< DMA channel for ADC transfers
#define DMA_ADC_SAMPLES 256                                     ///< Number of ADC samples per DMA transfer
#define DMA_IN_USE 4                                            ///< Number of channels to use for DMA
#define I2C_DMA_TX DREQ_I2C0_TX                                 ///< I2C DMA TX channel
#define I2C_DMA_RX DREQ_I2C0_RX                                 ///< I2C DMA RX channel
  
/** @} */ // end of dma_config

/**
 * @defgroup dma DMA Handler Functions
 * @brief Functions to initialize and manage DMA operations
 * @{
 */
void init_dma(void);
void read_adc_data(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_DMA_H