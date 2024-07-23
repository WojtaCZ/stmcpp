#ifndef STMCPP_I2C_H
#define STMCPP_I2C_H

#include <cstdint>
#include <cstddef>
#include <cmath>

#include <stmcpp/register.hpp>
#include <stmcpp/units.hpp>

#include "stm32h753xx.h"

namespace i2c{
    enum class peripheral : uint32_t {
        i2c1 = I2C1_BASE, ///< I2C1 peripheral selected
        i2c2 = I2C2_BASE, ///< I2C2 peripheral selected
        i2c3 = I2C3_BASE, ///< I2C3 peripheral selected
        i2c4 = I2C4_BASE  ///< I2C4 peripheral selected
    };

    enum class interrupt : uint32_t {
        txInterrupt = (0b1 << I2C_ISR_TXIS_Pos),
        rxInterrupt = (0b1 << I2C_ISR_RXNE_Pos),
        addrMatched = (0b1 << I2C_ISR_ADDR_Pos),  ///< Address matched
        nack        = (0b1 << I2C_ISR_NACKF_Pos), ///< Not acknowledge
        stop        = (0b1 << I2C_ISR_STOPF_Pos), ///< STOP detection
        // These interrupts are enabled by the TCIE bit
        txComplete  = (0b1 << I2C_ISR_TC_Pos),    ///< Transfer complete
        txReload    = (0b1 << I2C_ISR_TCR_Pos),   ///< Transfer complete reload
        // These interrupts are enabled by the ERRIE bit
        busError    = (0b1 << I2C_ISR_BERR_Pos),  ///< Bus error
        arbitrationError = (0b1 << I2C_ISR_ARLO_Pos),  ///< Arbitration lost
        overrunError     = (0b1 << I2C_ISR_OVR_Pos),   ///< Overrun/Underrun 
    };

    enum class status : uint32_t {
        txEmpty     = (0b1 << I2C_ISR_TXE_Pos),  ///< TX buffer empty
        busy        = (0b1 << I2C_ISR_BUSY_Pos)  ///< Communication on the bus is in progress
    };

    enum addressing : std::uint8_t {
        sevenBit = 0b0,
        tenBit   = 0b1
    };

    template<peripheral Peripheral>
    class i2c{
        private:
            I2C_TypeDef * const i2cHandle_ = reinterpret_cast<I2C_TypeDef *>(static_cast<std::uint32_t>(Peripheral));

        public:
            constexpr i2c(uint8_t prescaler, uint8_t setupTime, uint8_t holdTime, uint8_t highPeriod, uint8_t lowPeriod, addressing addressing = addressing::sevenBit,
                          bool autoEnd = false, bool clockStretching = true, bool analogFilter = true, std::uint8_t digitalFilter = 0
                        ){

                reg::write(std::ref(i2cHandle_->CR1),
                    ((static_cast<uint8_t>(clockStretching) & 0b1) << I2C_CR1_NOSTRETCH_Pos) |
                    ((static_cast<uint8_t>(analogFilter) & 0b1) << I2C_CR1_ANFOFF_Pos) |
                    ((static_cast<uint8_t>(digitalFilter) & 0b1111) << I2C_CR1_DNF_Pos) 
                );

                reg::write(std::ref(i2cHandle_->CR2),
                    ((static_cast<uint8_t>(autoEnd) & 0b1) << I2C_CR2_AUTOEND_Pos) |
                    ((static_cast<uint8_t>(addressing) & 0b1) << I2C_CR2_ADD10_Pos) 
                );

                reg::write(std::ref(i2cHandle_->TIMINGR),
                    ((static_cast<uint8_t>(prescaler) & 0b1111) << I2C_TIMINGR_PRESC_Pos) |
                    ((static_cast<uint8_t>(setupTime) & 0b1111) << I2C_TIMINGR_SCLDEL_Pos) |
                    ((static_cast<uint8_t>(holdTime) & 0b1111) << I2C_TIMINGR_SDADEL_Pos) |
                    ((static_cast<uint8_t>(highPeriod) & 0b11111111) << I2C_TIMINGR_SCLH_Pos) |
                    ((static_cast<uint8_t>(lowPeriod) & 0b11111111) << I2C_TIMINGR_SCLL_Pos) |
                );
            }

            void enable() const {
                reg::set(std::ref(i2cHandle_->CR1), I2C_CR1_PE);
            }

            void disable() const {
                reg::clear(std::ref(i2cHandle_->CR1), I2C_CR1_PE);
            }

            void enableRxDma() const {
                reg::set(std::ref(i2cHandle_->CR1), I2C_CR1_RXDMAEN);
            }

            void disableRxDma() const {
                reg::clear(std::ref(i2cHandle_->CR1), I2C_CR1_RXDMAEN);
            }

            void enableTxDma() const {
                reg::set(std::ref(i2cHandle_->CR1), I2C_CR1_TXDMAEN);
            }

            void disableTxDma() const {
                reg::clear(std::ref(i2cHandle_->CR1), I2C_CR1_TXDMAEN);
            }

            void enableInterrupt(interrupt interrupt) const {
                //All these three interrupts are affected by this bit
                if (interrupt == interrupt::txComplete || interrupt == interrupt::txReload) {
                    reg::set(std::ref(i2cHandle_->CR1), I2C_CR1_TCIE);
                } else if (interrupt == interrupt::busError || interrupt == interrupt::arbitrationError || interrupt == interrupt::overrunError) {
                    reg::set(std::ref(i2cHandle_->CR1), I2C_CR1_ERRIE);
                } else reg::set(std::ref(i2cHandle_->CR1), static_cast<std::uint32_t>(interrupt));              
            }

            void disableInterrupt(interrupt interrupt) const {
                //All these three interrupts are affected by this bit
                if (interrupt == interrupt::txComplete || interrupt == interrupt::txReload) {
                    reg::clear(std::ref(i2cHandle_->CR1), I2C_CR1_TCIE);
                } else if (interrupt == interrupt::busError || interrupt == interrupt::arbitrationError || interrupt == interrupt::overrunError) {
                    reg::clear(std::ref(i2cHandle_->CR1), I2C_CR1_ERRIE);
                } else reg::clear(std::ref(i2cHandle_->CR1), static_cast<std::uint32_t>(interrupt));
            }

            bool getInterruptFlag(interrupt interrupt) const {
                return static_cast<bool>(reg::read(std::ref(i2cHandle_->ISR), static_cast<std::uint32_t>(interrupt)));
            }

            bool getStatusFlag(status interrupt) const {
                return static_cast<bool>(reg::read(std::ref(i2cHandle_->ISR), static_cast<std::uint32_t>(interrupt)));
            }

            void clearInterruptFlag(interrupt interrupt) const {
                reg::read(std::ref(i2cHandle_->ICR), static_cast<std::uint32_t>(interrupt) & 0x3F38);
            }

            

    };
}

#endif


