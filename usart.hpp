#ifndef USART_H
#define USART_H

#include <cstdint>
#include <cstddef>
//#include <cmath>
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
    
    enum class wordlength : std::uint8_t{
        eightbit  = 0b00,
        ninebit   = 0b01,
        sevenbit  = 0b10
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
        rxenable        = USART_ISR_REACK 
    };

    /*constexpr std::uint32_t baudToRaw(const unsigned int periphclock, const unsigned int baudrate, const usart::oversampling oversamp = usart::oversampling::times16){
        return std::round((((oversamp ==  usart::oversampling::times16) ? 1.0 : 2.0) * static_cast<double>(periphclock)) / static_cast<double>(baudrate)); 
    }
*/
    template<peripheral Peripheral>
    class usart{
        private:
            USART_TypeDef * const usartHandle_ = reinterpret_cast<USART_TypeDef *>(static_cast<std::uint32_t>(Peripheral));
        public:
            constexpr usart(std::uint32_t rawbaud, bool fifomode, wordlength wordlength = wordlength::eightbit, ){

            }
    };

}

#endif