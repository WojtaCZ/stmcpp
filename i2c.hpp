#ifndef STMCPP_I2C_H
#define STMCPP_I2C_H

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <array>

#include <stmcpp/register.hpp>
#include <stmcpp/units.hpp>
#include <stmcpp/clock.hpp>

#include <stmcpp/error.hpp>

#include "stm32h753xx.h"

namespace stmcpp::i2c {
    using namespace stmcpp;
    using namespace stmcpp::units;

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
   
    class address {
        public:
            enum type : std::uint8_t {
                sevenBit = 0b0,
                tenBit   = 0b1
            };

        protected:
            type addressType_;
            std::uint16_t addressRaw_;

        public:
            address (std::uint16_t address, type type = type::sevenBit) : addressType_(type), addressRaw_(address) {
                if (type == type::sevenBit) {
                    addressRaw_ <<= 1;
                }
            }

        template<peripheral Peripheral> friend class i2c;
    };

    template<peripheral Peripheral>
    class i2c {
        private:
            I2C_TypeDef * const i2cHandle_ = reinterpret_cast<I2C_TypeDef *>(static_cast<std::uint32_t>(Peripheral));

            void waitForInterrupt(interrupt flag, duration timeout = 5000_ms) const {
                if(!stmcpp::clock::systick::initialized()) error::handler::hardThrow(error::code::systick_used_uninitialized);
                duration timestamp_ = stmcpp::clock::systick::getDuration();

                while (stmcpp::clock::systick::getDuration() < (timestamp_ + timeout)) {
                    if(getInterruptFlag(flag)) { 
                        return;
                    } else if (getInterruptFlag(interrupt::nack)) {
                        error::handler::hardThrow(error::code::i2c_nack);
                    } else if (getInterruptFlag(interrupt::busError)) {
                        error::handler::hardThrow(error::code::i2c_bus);
                    } else if (getInterruptFlag(interrupt::arbitrationError)) {
                        error::handler::hardThrow(error::code::i2c_arbitration);
                    } else if (getInterruptFlag(interrupt::overrunError)) {
                       //error::handler::hardThrow(error::code::i2c_overrun);
                       return;
                    } 
                }

                error::handler::hardThrow(error::code::i2c_timeout);
            }

            void waitForNotBusy(duration timeout = 100_ms) const {
                duration timestamp_ = stmcpp::clock::systick::getDuration();

                while (stmcpp::clock::systick::getDuration() < (timestamp_ + timeout)) {
                    if(!getStatusFlag(status::busy)) return;
                }
                error::handler::hardThrow(error::code::i2c_timeout);
            }

           

        public:
            constexpr i2c(uint8_t prescaler, uint8_t setupTime, uint8_t holdTime, uint8_t highPeriod, uint8_t lowPeriod,
                          bool clockStretching = true, bool analogFilter = true, std::uint8_t digitalFilter = 0
                        ){

                reg::write(std::ref(i2cHandle_->CR1),
                    ((static_cast<uint8_t>(clockStretching) & 0b1) << I2C_CR1_NOSTRETCH_Pos) |
                    ((static_cast<uint8_t>(analogFilter) & 0b1) << I2C_CR1_ANFOFF_Pos) |
                    ((static_cast<uint8_t>(digitalFilter) & 0b1111) << I2C_CR1_DNF_Pos) 
                );

                reg::write(std::ref(i2cHandle_->TIMINGR),
                    ((static_cast<uint8_t>(prescaler) & 0b1111) << I2C_TIMINGR_PRESC_Pos) |
                    ((static_cast<uint8_t>(setupTime) & 0b1111) << I2C_TIMINGR_SCLDEL_Pos) |
                    ((static_cast<uint8_t>(holdTime) & 0b1111) << I2C_TIMINGR_SDADEL_Pos) |
                    ((static_cast<uint8_t>(highPeriod) & 0b11111111) << I2C_TIMINGR_SCLH_Pos) |
                    ((static_cast<uint8_t>(lowPeriod) & 0b11111111) << I2C_TIMINGR_SCLL_Pos) 
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
                reg::set(std::ref(i2cHandle_->ICR), static_cast<std::uint32_t>(interrupt) & 0x3F38);
            }

            void start() const {
                reg::set(std::ref(i2cHandle_->CR2), I2C_CR2_START);
            }

            void stop() const {
                reg::set(std::ref(i2cHandle_->CR2), I2C_CR2_STOP);
            }

            void write(std::vector<std::uint8_t> & data, address & slaveAddress) const {
                int nbytes_ = data.size();
                if (nbytes_ > 255) error::handler::hardThrow(error::code::i2c_too_large_payload);
                
                std::uint32_t config_ = (nbytes_ << I2C_CR2_NBYTES_Pos) | 
                                        (slaveAddress.addressRaw_ << I2C_CR2_SADD_Pos) |
                                        (static_cast<std::uint8_t>(slaveAddress.addressType_) << I2C_CR2_ADD10_Pos) |
                                        (0b1 << I2C_CR2_AUTOEND_Pos) | 
                                        (0b0 << I2C_CR2_RD_WRN_Pos);
                                                  
                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_SADD_Msk | I2C_CR2_ADD10_Msk | I2C_CR2_AUTOEND_Msk | I2C_CR2_RD_WRN_Msk, config_);
                
                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);
                
                // Write each of the data bytes and wait for completion
                start();
                waitForInterrupt(interrupt::txInterrupt);
                for (std::uint8_t byte : data) {
                    reg::write(std::ref(i2cHandle_->TXDR), byte);
                    waitForInterrupt(interrupt::txInterrupt);
                }
                waitForNotBusy();
                //stop();

            }

            void write(std::uint8_t data, address & slaveAddress) const {
                               
                std::uint32_t config_ = (1 << I2C_CR2_NBYTES_Pos) | 
                                        (slaveAddress.addressRaw_ << I2C_CR2_SADD_Pos) |
                                        (static_cast<std::uint8_t>(slaveAddress.addressType_) << I2C_CR2_ADD10_Pos) |
                                        (0b1 << I2C_CR2_AUTOEND_Pos) | 
                                        (0b0 << I2C_CR2_RD_WRN_Pos);
                                                  
                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_SADD_Msk | I2C_CR2_ADD10_Msk | I2C_CR2_AUTOEND_Msk | I2C_CR2_RD_WRN_Msk, config_);
                
                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);

                // Write each of the data bytes and wait for completion
                start();
                waitForInterrupt(interrupt::txInterrupt);
                reg::write(std::ref(i2cHandle_->TXDR), data);
                waitForNotBusy();
               // waitForInterrupt(interrupt::txComplete);
                //stop();

            }

            
            void writeRegister(std::uint8_t regAddress, std::uint8_t data, address & slaveAddress) const {
                std::uint32_t config_ = (2 << I2C_CR2_NBYTES_Pos) | 
                                        (slaveAddress.addressRaw_ << I2C_CR2_SADD_Pos) |
                                        (static_cast<std::uint8_t>(slaveAddress.addressType_) << I2C_CR2_ADD10_Pos) |
                                        (0b1 << I2C_CR2_AUTOEND_Pos) | 
                                        (0b0 << I2C_CR2_RD_WRN_Pos);
                                                  
                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_SADD_Msk | I2C_CR2_ADD10_Msk | I2C_CR2_AUTOEND_Msk | I2C_CR2_RD_WRN_Msk, config_);
                
                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);

                // Write each of the data bytes and wait for completion
                start();
                waitForInterrupt(interrupt::txInterrupt);
                reg::write(std::ref(i2cHandle_->TXDR), regAddress);
                waitForInterrupt(interrupt::txInterrupt);
                reg::write(std::ref(i2cHandle_->TXDR), data);
                waitForNotBusy();
                
                //stop();
            }

            void read(std::vector<std::uint8_t> & data, int size, address & slaveAddress) const {
                if (size > 255) error::handler::hardThrow(error::code::i2c_too_large_payload);

                std::uint32_t config_ = (size << I2C_CR2_NBYTES_Pos) | 
                                        (slaveAddress.addressRaw_ << I2C_CR2_SADD_Pos) |
                                        (static_cast<std::uint8_t>(slaveAddress.addressType_) << I2C_CR2_ADD10_Pos) |
                                        (0b0<< I2C_CR2_AUTOEND_Pos) |
                                        (0b1 << I2C_CR2_RD_WRN_Pos);
                                                  
                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_SADD_Msk | I2C_CR2_ADD10_Msk | I2C_CR2_AUTOEND_Msk | I2C_CR2_RD_WRN_Msk, config_);

                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);

                // Read the requested bytes
                std::uint8_t val_;
                start();
                while (size > 0) {
                    waitForInterrupt(interrupt::rxInterrupt);
                    val_ = reg::read(std::ref(i2cHandle_->RXDR));
                    data.push_back(val_);
                    size--;
                }
                stop();
                waitForNotBusy();
            }

            std::uint8_t read(address & slaveAddress) const {

                std::uint32_t config_ = (1 << I2C_CR2_NBYTES_Pos) | 
                                        (slaveAddress.addressRaw_ << I2C_CR2_SADD_Pos) |
                                        (static_cast<std::uint8_t>(slaveAddress.addressType_) << I2C_CR2_ADD10_Pos) |
                                        (0b0<< I2C_CR2_AUTOEND_Pos) |
                                        (0b1 << I2C_CR2_RD_WRN_Pos);

                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_SADD_Msk | I2C_CR2_ADD10_Msk | I2C_CR2_AUTOEND_Msk | I2C_CR2_RD_WRN_Msk, config_);
                
                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);

                // Read the requested bytes
                start();
                waitForInterrupt(interrupt::rxInterrupt);
                std::uint8_t val_ = reg::read(std::ref(i2cHandle_->RXDR));
                stop();
                waitForNotBusy();

                return val_;
            }


            std::uint8_t readRegister(std::uint8_t regAddress, address & slaveAddress) const {
                std::uint32_t config_ = (1 << I2C_CR2_NBYTES_Pos) | 
                                        (slaveAddress.addressRaw_ << I2C_CR2_SADD_Pos) |
                                        (static_cast<std::uint8_t>(slaveAddress.addressType_) << I2C_CR2_ADD10_Pos) |
                                        (0b0<< I2C_CR2_AUTOEND_Pos) |
                                        (0b0 << I2C_CR2_RD_WRN_Pos);
                                                  
                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_SADD_Msk | I2C_CR2_ADD10_Msk | I2C_CR2_AUTOEND_Msk | I2C_CR2_RD_WRN_Msk, config_);
                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);
                // Write the register address
                start();
                waitForInterrupt(interrupt::txInterrupt);
                reg::write(std::ref(i2cHandle_->TXDR), regAddress);
                waitForInterrupt(interrupt::txComplete);
                stop();

                waitForNotBusy();
                // Clear the flags
                reg::write(std::ref(i2cHandle_->ICR), 0xFF);

                // Change the config to receive instead of transmitting
                reg::change(std::ref(i2cHandle_->CR2), I2C_CR2_NBYTES_Msk | I2C_CR2_RD_WRN_Msk, (1 << I2C_CR2_NBYTES_Pos) | (1 << I2C_CR2_RD_WRN_Pos));

                // Read the requested byte
                start();
                waitForInterrupt(interrupt::rxInterrupt);
                std::uint8_t val_ = reg::read(std::ref(i2cHandle_->RXDR));
                stop();
                waitForNotBusy();

                return val_;
            }

    };
}

#endif


