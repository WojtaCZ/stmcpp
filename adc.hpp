/* 
 * This file is part of the stmcpp distribution (https://github.com/WojtaCZ/stmcpp).
 * Copyright (c) 2024 Vojtech Vosahlo.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STMCPP_ADC_H
#define STMCPP_ADC_H

#include "stm32h753xx.h"
#include <cstdint>

#include <stmcpp/register.hpp>
#include <stmcpp/units.hpp>
#include <stmcpp/clock.hpp>
#include <stmcpp/error.hpp>


namespace stmcpp::adc {
    using namespace stmcpp;
    using namespace stmcpp::units;

    enum class error {
        ldo_timeout,
        calibration_timeout,
        timeout
    };

    static stmcpp::error::handler<error, "stmcpp::adc"> errorHandler;
    
    enum class peripheral : std::uint32_t {
        adc1 = ADC1_BASE,
        adc2 = ADC2_BASE,
        adc3 = ADC3_BASE
    };

    enum class interrupt : std::uint32_t {
        adcReady = ADC_IER_ADRDYIE_Msk,
        endOfSampling = ADC_IER_EOSMPIE_Msk,
        endOfConversion = ADC_IER_EOCIE_Msk,
        endOfSequence = ADC_IER_EOSIE_Msk,
        overrun = ADC_IER_OVRIE_Msk,
        endOfInjectedConversion = ADC_IER_JEOCIE_Msk,
        endOfInjectedSequence = ADC_IER_JEOSIE_Msk,
        analogWatchdog1 = ADC_IER_AWD1IE_Msk,
        analogWatchdog2 = ADC_IER_AWD2IE_Msk,
        analogWatchdog3 = ADC_IER_AWD3IE_Msk,
        injectedContextQueueOverflow = ADC_IER_JQOVFIE_Msk
    };

    enum class status : std::uint32_t {
        ldoReady = ADC_ISR_LDORDY_Msk
    };

    enum class calibration {
        singleEnded,
        differential
    };

    template<peripheral Peripheral>
        class adc {
            private:
                ADC_TypeDef * const adcHandle_ = reinterpret_cast<ADC_TypeDef *>(static_cast<std::uint32_t>(Peripheral));

            public:
                adc() {
                    

                    /*reg::write(std::ref(adcHandle_->CR1),
                        ((static_cast<uint8_t>(enableSS_) & 0b1) << SPI_CR1_SSI_Pos) |
                        ((static_cast<uint8_t>(enablesuspend) & 0b1) << SPI_CR1_MASRX_Pos) |
                        ((static_cast<uint8_t>(crcfullsize) & 0b1) << SPI_CR1_CRC33_17_Pos) |
                        ((static_cast<uint8_t>(crcrx) & 0b1) << SPI_CR1_RCRCINI_Pos) |
                        ((static_cast<uint8_t>(crctx) & 0b1) << SPI_CR1_TCRCINI_Pos)
                    );*/

                   
                }

                void enable() const {
                    reg::set(std::ref(adcHandle_->CR), ADC_CR_ADEN);
                }

                void disable() const {
                    reg::clear(std::ref(adcHandle_->CR), ADC_CR_ADEN);
                }

                void calibrate(calibration calib, bool enableLinearity = false) const {
                    // Disable the ADC
                    disable();
                    // Make sure to disable deep powerdown
                    reg::clear(std::ref(adcHandle_->CR), ADC_CR_DEEPPWD);
                    // Enable the ADC voltage regulator
                    reg::set(std::ref(adcHandle_->CR), ADC_CR_ADVREGEN);
                    // Wait for it to be ready
                    reg::waitForBitSet(std::ref(adcHandle_->ISR), status::ldoReady, []() { errorHandler.hardThrow(error::ldo_timeout); });

                    // Set up DIFF/Single Ended calibration
                    if (calib == calibration::singleEnded){
                        reg::clear(std::ref(adcHandle_->CR), ADC_CR_ADCALDIF);
                    } else reg::set(std::ref(adcHandle_->CR), ADC_CR_ADCALDIF);
                    
                    // Enable or disable linearity calibration
                    if (enableLinearity) {
                        reg::set(std::ref(adcHandle_->CR), ADC_CR_ADCALLIN);
                    } else reg::clear(std::ref(adcHandle_->CR), ADC_CR_ADCALLIN);

                    // Start the calibration and wait for it to finish
                    reg::set(std::ref(adcHandle_->CR), ADC_CR_ADCAL);
                    reg::waitForBitClear(std::ref(adcHandle_->CR), ADC_CR_ADCAL, []() { errorHandler.hardThrow(error::calibration_timeout); });
                }

                void enableInterrupt(interrupt interrupt) const {
                    reg::set(std::ref(adcHandle_->IER), static_cast<std::uint32_t>(interrupt));              
                }

                void disableInterrupt(interrupt interrupt) const {
                    reg::clear(std::ref(adcHandle_->IER), static_cast<std::uint32_t>(interrupt));
                }

                bool getInterruptFlag(interrupt interrupt) const {
                    return static_cast<bool>(reg::read(std::ref(adcHandle_->ISR), static_cast<std::uint32_t>(interrupt)));
                }

                bool getStatusFlag(status interrupt) const {
                    return static_cast<bool>(reg::read(std::ref(adcHandle_->ISR), static_cast<std::uint32_t>(interrupt)));
                }

                void clearInterruptFlag(interrupt interrupt) const {
                    reg::set(std::ref(adcHandle_->ISR), static_cast<std::uint32_t>(interrupt) & 0x07FD);
                }

    };
}

#endif