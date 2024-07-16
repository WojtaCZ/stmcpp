/* 
 * This file is part of the stmcpp distribution (https://github.com/WojtaCZ/stm-cpp).
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

#ifndef STMCPP_USART_H
#define STMCPP_USART_H

#include <cstdint>
#include <cstddef>
#include <cmath>
#include "register.hpp"
#include "stm32h753xx.h"

namespace usart{

    enum class peripheral : std::uint32_t {
        usart1 = USART1_BASE,
        usart2 = USART2_BASE,
        usart3 = USART3_BASE,
        uart4 = UART4_BASE,
        uart5 = UART5_BASE,
        usart6 = USART6_BASE,
        uart7 = UART7_BASE,
        uart8 = UART8_BASE
    };

    enum class oversampling : std::uint8_t {
        times16 = 0b0,
        times8  = 0b1
    };

    enum class stopbits : std::uint8_t{
        one         = 0b00,
        half        = 0b01,
        two         = 0b10,
        oneandhalf  = 0b11
    };

    enum class linbreaklength : std::uint8_t{
        tenbit    = 0b0,
        elevenbit = 0b1
    };

    enum class addresslength : std::uint8_t{
        fourbit  = 0b0,
        sevenbit = 0b1
    };

    enum class rxfifotreshold : std::uint8_t{
        eight           = 0b000,
        quarter         = 0b001,
        half            = 0b010,
        threequarter    = 0b011,
        seveneights     = 0b100,
        full            = 0b101
    };

    enum class txfifotreshold : std::uint8_t{
        eight           = 0b000,
        quarter         = 0b001,
        half            = 0b010,
        threequarter    = 0b011,
        seveneights     = 0b100,
        empty           = 0b101
    };
    
    enum class wakeupevent : std::uint8_t{
        addressmatch  = 0b00,
        startbit      = 0b10,
        rxnotempty    = 0b11
    };

    enum class parity : std::uint8_t{
        none  = 0b00,
        even  = 0b10,
        odd   = 0b11
    };
    
    enum class wordlength : std::uint8_t{
        eightbit  = 0b00,
        ninebit   = 0b01,
        sevenbit  = 0b10
    };

    enum class bitorder : std::uint8_t {
        msbfirst    = 0b1, ///< MSB is transmitted first
        lsbfirst    = 0b0  ///< LSB is transmitted first
    };

    enum class clockpol : std::uint8_t {
        idlelow     = 0b0, ///< The USART clock is low in idle
        idlehigh    = 0b1  ///< The USART clock is high in idle
    };

    enum class clockphase : std::uint8_t {
        firsttransition     = 0b0, ///< The first clock transition is the first data capture edge
        secondtransition    = 0b1  ///< The second clock transition is the first data capture edge
    };

    enum class driverenablepol : std::uint8_t {
        activehigh     = 0b0, 
        activelow    = 0b1 
    };

    enum class divider : std::uint8_t {
        nodivide    = 0b0000, 
        div2        = 0b0001,
        div4        = 0b0010,
        div6        = 0b0011,
        div8        = 0b0100,
        div10       = 0b0101,
        div12       = 0b0110,
        div16       = 0b0111,
        div32       = 0b1000,
        div64       = 0b1001,
        div128      = 0b1010,
        div256      = 0b1011
    };

    enum class interrupt : std::uint32_t{
        txempty         = USART_ICR_TXFECF,
        endofblock      = USART_ICR_EOBCF,
        rxtimeout       = USART_ICR_RTOCF,
        charactermatch  = USART_ICR_CMCF,
        parityerror     = USART_ICR_PECF,
        framingerror    = USART_ICR_FECF,
        noisedetect     = USART_ICR_NECF,
        overerun        = USART_ICR_ORECF,
        txcomplete      = USART_ICR_TCCF,
        idle            = USART_ICR_IDLECF,
        linbreak        = USART_ICR_LBDCF,
        underrun        = USART_ICR_UDRCF,
        lpwakeup        = USART_ICR_WUCF,
        cts             = USART_ICR_CTSCF,
        txbeforeguard   = USART_ICR_TCBGTCF
    };

    enum class flag : std::uint32_t{
        rxfull          = USART_ISR_RXFF,
        rxtreshold      = USART_ISR_RXFT,
        txtreshold      = USART_ISR_TXFT,
        cts             = USART_ISR_CTS,
        autobauderror   = USART_ISR_ABRE,
        autobaudflag    = USART_ISR_ABRF,
        busy            = USART_ISR_BUSY,
        sendbreak       = USART_ISR_SBKF,
        mutewakeup      = USART_ISR_RWU,
        txenable        = USART_ISR_TEACK,
        rxenable        = USART_ISR_REACK,
        rxnotempty      = USART_ISR_RXNE_RXFNE,
        txfree          = USART_ISR_TXE_TXFNF
    };

    //template<unsigned int periphclock, unsigned int baudrate, usart::oversampling oversamp = usart::oversampling::times16>
    constexpr std::uint32_t baudToRaw(const unsigned int periphclock, const unsigned int baudrate, const usart::oversampling oversamp = usart::oversampling::times16){
        auto multiplier_ = (oversamp == usart::oversampling::times16) ? 1.0 : 2.0;
        auto baud_ = (multiplier_ * static_cast<double>(periphclock)) / static_cast<double>(baudrate);

        if(baud_ - static_cast<double>(static_cast<uint32_t>(baud_)) > 0.5){
            return static_cast<uint32_t>(baud_) + 1;
        }
        
        return static_cast<uint32_t>(baud_);
    }

    template<usart::peripheral Peripheral>
    class uart{
        private:
            USART_TypeDef * const usartHandle_ = reinterpret_cast<USART_TypeDef *>(static_cast<std::uint32_t>(Peripheral));
        public:
            constexpr uart(std::uint32_t rawbaud, usart::divider divider = usart::divider::nodivide, bool fifomode = false, usart::wordlength wordlength = usart::wordlength::eightbit, usart::oversampling oversampling = usart::oversampling::times16,
                            usart::parity parity = usart::parity::none, usart::stopbits stopbits = usart::stopbits::one, usart::bitorder bitorder = usart::bitorder::lsbfirst
                            ){
                                
                reg::write(std::ref(usartHandle_->CR1),
                    ((static_cast<uint8_t>(fifomode) & 0b1) << USART_CR1_FIFOEN_Pos) |
                    ((static_cast<uint8_t>(wordlength) & 0b10) << (USART_CR1_M1_Pos - 1)) |
                    ((static_cast<uint8_t>(wordlength) & 0b01) << USART_CR1_M0_Pos) |
                    ((static_cast<uint8_t>(oversampling) & 0b1) << USART_CR1_OVER8_Pos) |
                    ((static_cast<uint8_t>(parity) & 0b11) << USART_CR1_PS_Pos)
                );

                reg::write(std::ref(usartHandle_->CR2),
                    ((static_cast<uint8_t>(bitorder) & 0b1) << USART_CR2_MSBFIRST_Pos) |
                    ((static_cast<uint8_t>(stopbits) & 0b11) << USART_CR2_STOP_Pos) 

                );

                if(oversampling == usart::oversampling::times16){
                    reg::write(std::ref(usartHandle_->BRR), rawbaud);
                }else reg::write(std::ref(usartHandle_->BRR), (rawbaud & 0xFFFFFFF8) | ((rawbaud >> 1) & 0x3));
                
            }

            void enable() const {
                reg::set(std::ref(usartHandle_->CR1), USART_CR1_UE);
            }

            void disable() const {
                reg::clear(std::ref(usartHandle_->CR1), USART_CR1_UE);
            }

            void enableTx() const {
                reg::set(std::ref(usartHandle_->CR1), USART_CR1_TE);
            }

            void disableTx() const {
                reg::clear(std::ref(usartHandle_->CR1), USART_CR1_TE);
            }

            void enableTxDma() const {
                reg::set(std::ref(USART2->CR3), USART_CR3_DMAT);
            }

            void disableTxDma() const {
                reg::clear(std::ref(USART2->CR3), USART_CR3_DMAT);
            }

            void enableRx() const {
                reg::set(std::ref(usartHandle_->CR1), USART_CR1_RE);
            }

            void disableRx() const {
                reg::clear(std::ref(usartHandle_->CR1), USART_CR1_RE);
            }

            void enableRxDma() const {
                reg::set(std::ref(USART2->CR3), USART_CR3_DMAR);
            }

            void disableRxDma() const {
                reg::clear(std::ref(USART2->CR3), USART_CR3_DMAR);
            }

            void transmit(std::uint8_t data) const {
                reg::write(std::ref(usartHandle_->TDR), data);
            }

            void txFifoTreshold(usart::txfifotreshold treshold) const {
                reg::change(std::ref(usartHandle_->CR3), 0x07, static_cast<uint8_t>(treshold), USART_CR3_TXFTCFG_Pos);
            }

            void rxFifoTreshold(usart::rxfifotreshold treshold) const {
                reg::change(std::ref(usartHandle_->CR3), 0x07, static_cast<uint8_t>(treshold), USART_CR3_RXFTCFG_Pos);
            }

            void wakeupEvent(usart::wakeupevent wakeupevent) const {
                reg::change(std::ref(usartHandle_->CR3), 0x03, static_cast<uint8_t>(wakeupevent), USART_CR3_WUS_Pos);
            }

            /// @brief Set the USART receiver timeout
            /// @param timeout Timeout in terms of number of bits
            void rxTimeout(std::uint32_t timeout) const { 
                reg::change(std::ref(usartHandle_->RTOR), 0x00FFFFFF, timeout);
            }

            void enableDe(usart::driverenablepol depolarity = usart::driverenablepol::activehigh, std::uint8_t assertion = 0, std::uint8_t deassertion = 0) const {
                reg::set(std::ref(usartHandle_->CR1),
                    ((assertion & 0b11111) << USART_CR1_DEAT_Pos) |
                    ((deassertion & 0b11111) << USART_CR1_DEDT_Pos) 
                );

                reg::set(std::ref(usartHandle_->CR3),
                    (                                  0b1 << USART_CR3_DEM_Pos) |
                    ((static_cast<uint8_t>(depolarity) & 0b1) << USART_CR3_DEP_Pos) 
                );
            }

            void disableDe() const {
                reg::clear(std::ref(usartHandle_->CR1),
                    (0b11111 << USART_CR1_DEAT_Pos) |
                    (0b11111 << USART_CR1_DEDT_Pos) 
                );

                reg::clear(std::ref(usartHandle_->CR3),
                    (0b1 << USART_CR3_DEM_Pos) |
                    (0b1 << USART_CR3_DEP_Pos) 
                );
            }

            void enableCk(usart::clockpol clockpol = usart::clockpol::idlelow, usart::clockphase clockphase = usart::clockphase::firsttransition, bool lastbitclock = false) const {
                reg::set(std::ref(usartHandle_->CR2),
                    (                                  0b1 << USART_CR2_CLKEN_Pos) |
                    ((static_cast<uint8_t>(clockpol) & 0b1) << USART_CR2_CPOL_Pos) |
                    ((static_cast<uint8_t>(clockphase) & 0b1) << USART_CR2_CPHA_Pos) | 
                    ((static_cast<uint8_t>(lastbitclock) & 0b1) << USART_CR2_LBCL_Pos)
                );
            }

            void disableCk() const {
                reg::clear(std::ref(usartHandle_->CR2),
                    (0b1 << USART_CR2_CLKEN_Pos) |
                    (0b1 << USART_CR2_CPOL_Pos) |
                    (0b1 << USART_CR2_CPHA_Pos) | 
                    (0b1 << USART_CR2_LBCL_Pos)
                );
            }

            void clearInterruptFlag(usart::interrupt interrupt) const {
                reg::set(std::ref(usartHandle_->ICR), static_cast<uint32_t>(interrupt));
            }

            bool getInterruptFlag(usart::interrupt interrupt) const {
                if(interrupt == usart::interrupt::txbeforeguard) return static_cast<bool>(reg::read(std::ref(usartHandle_->ISR), USART_ISR_TCBGT)); 
                if(interrupt == usart::interrupt::txempty) return static_cast<bool>(reg::read(std::ref(usartHandle_->ISR), USART_ISR_TXFE)); 

                return static_cast<bool>(reg::read(std::ref(usartHandle_->ISR), static_cast<uint32_t>(interrupt)));
            }

            bool getStatusFlag(usart::flag flag) const {
                return static_cast<bool>(reg::read(std::ref(usartHandle_->ISR), static_cast<uint32_t>(flag)));
            }


    };

}

#endif