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
        enable_timeout,
        disable_timeout,
        regular_stop_timeout,
        injected_stop_timeout,
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
                // Make sure to disable deep powerdown
                reg::clear(std::ref(adcHandle_->CR), ADC_CR_DEEPPWD);
                // Enable the ADC voltage regulator
                reg::set(std::ref(adcHandle_->CR), ADC_CR_ADVREGEN);
                // Wait for it to be ready
                reg::waitForBitSet(std::ref(adcHandle_->ISR), status::ldoReady, []() { errorHandler.hardThrow(error::ldo_timeout); });

                
            }

            void enable() const {
                // Enable the ADC and wait for it to be ready
                reg::set(std::ref(adcHandle_->ISR), ADC_ISR_ADRDY);
                reg::set(std::ref(adcHandle_->CR), ADC_CR_ADEN);
                reg::waitForBitSet(std::ref(adcHandle_->ISR), interrupt::adcReady, []() { errorHandler.hardThrow(error::enable_timeout); });
            }

            void disable() const {
                // If there is a conversion ongoing, stop it
                if (reg::read(std::ref(adcHandle_->CR), ADC_CR_ADSTART | ADC_CR_JADSTART)) {
                    reg::set(std::ref(adcHandle_->CR), ADC_CR_ADSTP | ADC_CR_JADSTP);
                    reg::waitForBitClear(std::ref(adcHandle_->CR), ADC_CR_ADSTP | ADC_CR_JADSTP, []() { errorHandler.hardThrow(error::disable_timeout); });
                }

                reg::set(std::ref(adcHandle_->CR), ADC_CR_ADDIS);
                reg::waitForBitClear(std::ref(adcHandle_->CR), ADC_CR_ADEN, []() { errorHandler.hardThrow(error::disable_timeout); });
            }

            void calibrate(calibration calib, bool enableLinearity = false) const {
                // Disable the ADC
                disable();
                // Make sure to disable deep powerdown
                reg::clear(std::ref(adcHandle_->CR), ADC_CR_DEEPPWD);
                // And check if the LDO is ready
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

            void startRegular() const {
                reg::set(std::ref(adcHandle_->CR), ADC_CR_ADSTART);
            }

            void stopRegular() const {
                reg::set(std::ref(adcHandle_->CR), ADC_CR_ADSTP);
                reg::waitForBitClear(std::ref(adcHandle_->CR), ADC_CR_ADSTP, []() { errorHandler.hardThrow(error::regular_stop_timeout); });
            }

            bool isConvertingRegular() const {
                return static_cast<bool>(reg::read(std::ref(adcHandle_->CR), ADC_CR_ADSTART)); 
            }

            void startInjected() const {
                reg::set(std::ref(adcHandle_->CR), ADC_CR_JADSTART);
            }

            void stopInjected() const {
                reg::set(std::ref(adcHandle_->CR), ADC_CR_JADSTP);
                reg::waitForBitClear(std::ref(adcHandle_->CR), ADC_CR_ADSTP, []() { errorHandler.hardThrow(error::injected_stop_timeout); });
            }

            bool isConvertingInjected() const {
                return static_cast<bool>(reg::read(std::ref(adcHandle_->CR), ADC_CR_JADSTART)); 
            }



    };

    namespace channel {
        enum class channel {
            ch0,
            ch1,
            ch2,
            ch3,
            ch4,
            ch5,
            ch6,
            ch7,
            ch8,
            ch9,
            ch10,
            ch11,
            ch12,
            ch13,
            ch14,
            ch15
        };

        enum class oversampling {
            oneAndHalfClocks = 0b000,
            twoAndHalfClocks = 0b001,
            eightAndHalfClocks = 0b010,
            sixteenAndHalfClocks = 0b011,
            thirtyTwoAndHalfClocks = 0b100,
            sixtyFourAndHalfClocks = 0b101,
            threeHundretEightySevenAndHalfClocks = 0b110,
            eightHundretTenAndHalfClocks = 0b111

        };

        template<channel Channel>
        class channel {
            channel() {

            }
        };

    }
}

#endif