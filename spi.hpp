#ifndef SPI_H
#define SPI_H

#include "stm32h753xx.h"
#include <cstdint>

namespace spi{

    enum class peripheral : uint32_t {
        spi1 = SPI1_BASE,
        spi2 = SPI2_BASE,
        spi3 = SPI3_BASE,
        spi4 = SPI4_BASE,
        spi5 = SPI5_BASE,
        spi6 = SPI6_BASE
    };

    enum class role {
        slave   = 0b0, ///< SPI is configured as slave
        master  = 0b1  ///< SPI is configured as master
    };

    enum class mode {
        fullduplex  = 0b00, ///< SPI is configured in full duplex mode
        txsimplex   = 0b01, ///< SPI is configured in transmitter simplex mode
        rxsimplex   = 0b10, ///< SPI is configured in receiver simplex mode
        halfduplex  = 0b11  ///< SPI is configured in half duplex mode
    };

    enum class direction {
        receiver    = 0b0, ///< SPI half-dupex direction is receiver
        transmitter = 0b1  ///< SPI half-dupex direction is transmitter
    };


    enum class protocol {
        motorola    = 0b000, ///< SPI is configured to use the motorola protocol
        ti          = 0b001  ///< SPI is configured to use the TI protocol
    };

    enum class clockpol {
        idlelow     = 0b0, ///< The SPI clock is low in idle
        idlehigh    = 0b1  ///< The SPI clock is high in idle
    };

    enum class ssorigin {
        sspad       = 0b0, 
        ssbit       = 0b1  
    };

    enum class ssbehavior {
        endoftransfer       = 0b0, ///< SS is kept at active level till data transfer is completed, it becomes inactive with EOT flag
        interleaved         = 0b1  ///< SPI data frames are interleaved with SS non active pulses when dataIdleness > 1
    };

    enum class afbehavior {
        nocontrol           = 0b0, ///< The peripheral takes no control of GPIOs while it is disabled
        keepcontrol         = 0b1  ///< The peripheral keeps always control of all associated GPIOs
    };

    enum class clockphase {
        firsttransition     = 0b0, ///< The first clock transition is the first data capture edge
        secondtransition    = 0b1  ///< The second clock transition is the first data capture edge
    };

    enum class bitorder {
        msbfirst    = 0b0, ///< MSB is transmitted first
        lsbfirst    = 0b1  ///< LSB is transmitted first
    };

    enum class sspol {
        activelow   = 0b0, ///< The SPI slave select is active low
        activehigh  = 0b1  ///< The SPI slave select is active high
    };
    
    enum class underrunbehav {
        pattern             = 0b00, ///< Send out pattern defined by SPI_UDRDR
        repeatlastrx        = 0b01, ///< Repeat lastly received data
        repeatlasttx        = 0b10  ///< Repeat lastly transmitted data
    };

    enum class underrundet {
        beginframe          = 0b00, ///< Underrun detected at beginning of the data frame
        endframe            = 0b01, ///< Underrun detected at end of the data frame
        beginslaveselect    = 0b10  ///< Underrun detected at beginning of the slave select signal
    };

    enum class crc {
        allzero     = 0b0, ///< All zero calculation pattern 
        allone      = 0b1  ///< All one calculation patern
    };

    enum class masterdivider {
        div2        = 0b000, ///< The spi master clock is divided by 2
        div4        = 0b001, ///< The spi master clock is divided by 4
        div8        = 0b010, ///< The spi master clock is divided by 8
        div16       = 0b011, ///< The spi master clock is divided by 16
        div32       = 0b100, ///< The spi master clock is divided by 32
        div64       = 0b101, ///< The spi master clock is divided by 64
        div128      = 0b110, ///< The spi master clock is divided by 128
        div256      = 0b111  ///< The spi master clock is divided by 256
    };

    enum class interrupt : uint32_t {
        endoftransfer    = SPI_SR_EOT,   ///< End of transfer
        txfilled         = SPI_SR_TXTF,  ///< Transmission transfer filled
        underrun         = SPI_SR_UDR,   ///< Underrun
        overrun          = SPI_SR_OVR,   ///< Overrun
        crcerror         = SPI_SR_CRCE,  ///< CRC error
        tiferror         = SPI_SR_TIFRE, ///< TI frame format error
        modefault        = SPI_SR_MODF,  ///< Mode fault
        additionalreload = SPI_SR_TSERF, ///< Additional number of data
        suspend          = SPI_SR_SUSP,   ///< SPI suspended
        rxpacket         = SPI_SR_RXP,   ///< RX packet is available
        txpacket         = SPI_SR_TXP,   ///< TX packet space is available
        duplexpacket     = SPI_SR_DXP,   ///< RX packet was received and TX packet space is available
        txcomplete       = SPI_SR_TXC   ///< RX packet is available
    };

    template<peripheral Peripheral>
        class spi{
            private:
                SPI_TypeDef * const spiHandle_ = reinterpret_cast<SPI_TypeDef *>(static_cast<std::uint32_t>(Peripheral));
            public:
                spi(role role, mode mode, uint8_t framesize, masterdivider masterdivider = masterdivider::div2, protocol protocol = protocol::motorola, bitorder bitorder = bitorder::msbfirst, 
                clockpol clockpol = clockpol::idlelow, clockphase clockphase = clockphase::firsttransition, ssorigin ssorigin = ssorigin::sspad, sspol sspol = sspol::activelow, bool ssoutputenable = false, 
                ssbehavior ssbehavior = ssbehavior::endoftransfer , uint8_t fifotreshold = 1, bool enabletxdma = false, bool enablerxdma = false, uint8_t ssidleness = 0, uint8_t dataidleness = 0,
                bool ioswap = false, afbehavior afbehavior = afbehavior::nocontrol, bool crcenable = false, uint8_t crcsize = 4, bool crcfullsize = false, crc crcrx = crc::allzero,
                crc crctx = crc::allzero, bool enablesuspend = false, underrundet underrundet = underrundet::beginframe, underrunbehav underrunbehav = underrunbehav::pattern){
                    
                    bool enableSS_ = false;

                    if(role == role::master){
                        ssorigin = ssorigin::ssbit;
                        enableSS_ = true;
                    }

                    reg::write(std::ref(spiHandle_->CR1),
                        ((static_cast<uint8_t>(enableSS_) & 0b1) << SPI_CR1_SSI_Pos) |
                        ((static_cast<uint8_t>(enablesuspend) & 0b1) << SPI_CR1_MASRX_Pos) |
                        ((static_cast<uint8_t>(crcfullsize) & 0b1) << SPI_CR1_CRC33_17_Pos) |
                        ((static_cast<uint8_t>(crcrx) & 0b1) << SPI_CR1_RCRCINI_Pos) |
                        ((static_cast<uint8_t>(crctx) & 0b1) << SPI_CR1_TCRCINI_Pos)
                    );

                    //static_assert(frameSize_ > 3 && frameSize_ < 33, "The number of bits has to be between 4 and 32!");
                    //static_assert(crcSize_ > 3 && crcSize_ < 31, "The CRC size has to be between 4 and 32 bits!");
                    //static_assert(frameSize_ < 17, "The FIFO treshold has to be less than 17!");

                    reg::write(std::ref(spiHandle_->CFG1),
                        ((static_cast<uint8_t>(framesize - 1) & 0b11111) << SPI_CFG1_DSIZE_Pos) |
                        ((static_cast<uint8_t>(fifotreshold - 1) & 0b1111) << SPI_CFG1_FTHLV_Pos) |
                        ((static_cast<uint8_t>(underrunbehav) & 0b11) << SPI_CFG1_UDRCFG_Pos) |
                        ((static_cast<uint8_t>(underrundet) & 0b11) << SPI_CFG1_UDRDET_Pos) |
                        ((static_cast<uint8_t>(enablerxdma) & 0b1) << SPI_CFG1_RXDMAEN_Pos) |
                        ((static_cast<uint8_t>(enabletxdma) & 0b1) << SPI_CFG1_TXDMAEN_Pos) |
                        ((static_cast<uint8_t>(crcsize - 1) & 0b11111) << SPI_CFG1_CRCSIZE_Pos) |
                        ((static_cast<uint8_t>(crcenable) & 0b1) << SPI_CFG1_CRCEN_Pos) |
                        ((static_cast<uint8_t>(masterdivider) & 0b111) << SPI_CFG1_MBR_Pos) 
                    );

                    //static_assert(!((ssIdleness_ || dataIdleness_) && protocol_ == protocol::ti), "The SS and DATA idleness is not supported in TI mode!");
                    //static_assert(ssIdleness_ < 16, "The SS idleness less than 16 clock periods!");
                    //static_assert(dataIdleness_ < 16, "The DATA idleness less than 16 clock periods!");

                    reg::write(std::ref(spiHandle_->CFG2),
                        ((static_cast<uint8_t>(ssidleness) & 0b1111) << SPI_CFG2_MSSI_Pos) |
                        ((static_cast<uint8_t>(dataidleness) & 0b1111) << SPI_CFG2_MIDI_Pos) |
                        ((static_cast<uint8_t>(ioswap) & 0b1) << SPI_CFG2_IOSWP_Pos) |
                        ((static_cast<uint8_t>(mode) & 0b11) << SPI_CFG2_COMM_Pos) |
                        ((static_cast<uint8_t>(protocol) & 0b111) << SPI_CFG2_SP_Pos) |
                        ((static_cast<uint8_t>(role) & 0b1) << SPI_CFG2_MASTER_Pos) |
                        ((static_cast<uint8_t>(bitorder) & 0b1) << SPI_CFG2_LSBFRST_Pos) |
                        ((static_cast<uint8_t>(clockphase) & 0b1) << SPI_CFG2_CPHA_Pos) |
                        ((static_cast<uint8_t>(clockpol) & 0b1) << SPI_CFG2_CPOL_Pos) |
                        ((static_cast<uint8_t>(ssorigin) & 0b1) << SPI_CFG2_SSM_Pos) |
                        ((static_cast<uint8_t>(sspol) & 0b1) << SPI_CFG2_SSIOP_Pos) |
                        ((static_cast<uint8_t>(ssoutputenable) & 0b1) << SPI_CFG2_SSOE_Pos) |
                        ((static_cast<uint8_t>(ssbehavior) & 0b1) << SPI_CFG2_SSOM_Pos) |
                        ((static_cast<uint8_t>(afbehavior) & 0b1) << SPI_CFG2_AFCNTR_Pos) 
                    );
                }

                void enable(){
                    reg::set(std::ref(spiHandle_->CR1), SPI_CR1_SPE);
                }

                void disable(){
                    reg::clear(std::ref(spiHandle_->CR1), SPI_CR1_SPE);
                }

                void startTransfer(){
                    reg::set(std::ref(spiHandle_->CR1), SPI_CR1_CSTART);
                }

                void requestSuspend(){
                    reg::set(std::ref(spiHandle_->CR1), SPI_CR1_CSUSP);
                }

                void setDirection(direction direction){
                    if(direction == direction::receiver){
                        reg::clear(std::ref(spiHandle_->CR1), SPI_CR1_HDDIR);
                    }else reg::set(std::ref(spiHandle_->CR1), SPI_CR1_HDDIR);
                }

                void setInternalSs(){
                    reg::set(std::ref(spiHandle_->CR1), SPI_CR1_SSI);
                }

                void clearInternalSs(){
                    reg::clear(std::ref(spiHandle_->CR1), SPI_CR1_SSI);
                }

                void enableCrc(){
                    reg::set(std::ref(spiHandle_->CFG1), SPI_CFG1_CRCEN);
                }

                void disableCrc(){
                    reg::clear(std::ref(spiHandle_->CFG1), SPI_CFG1_CRCEN);
                }

                void setCrc(uint32_t crc){
                    reg::write(std::ref(spiHandle_->CRCPOLY), crc); 
                }

                void setTxCrc(uint32_t crc){
                    reg::write(std::ref(spiHandle_->TXCRC), crc); 
                }
                
                void setRxCrc(uint32_t crc){
                    reg::write(std::ref(spiHandle_->RXCRC), crc); 
                }

                void lockIo(){
                    reg::set(std::ref(spiHandle_->CR1), SPI_CR1_IOLOCK);
                }

                void setNumberOfData(uint16_t number){
                    reg::change(std::ref(spiHandle_->CR2), 0xFFFF, number, SPI_CR2_TSIZE_Pos);
                }

                void setNumberOfDataExtension(uint16_t number){
                    reg::change(std::ref(spiHandle_->CR2), 0xFFFF, number, SPI_CR2_TSER_Pos);
                }

                void enableRxDma(){
                    reg::set(std::ref(spiHandle_->CFG1), SPI_CFG1_RXDMAEN);
                }

                void disableRxDma(){
                    reg::clear(std::ref(spiHandle_->CFG1), SPI_CFG1_RXDMAEN);
                }

                void enableTxDma(){
                    reg::set(std::ref(spiHandle_->CFG1), SPI_CFG1_TXDMAEN);
                }

                void disableTxDma(){
                    reg::clear(std::ref(spiHandle_->CFG1), SPI_CFG1_TXDMAEN);
                }

                void enableInterrupt(interrupt interrupt){
                    //All these three interrupts are affected by this bit
                    if(interrupt == interrupt::endoftransfer || interrupt == interrupt::suspend || interrupt == interrupt::txcomplete){
                        reg::set(std::ref(spiHandle_->IER), SPI_IER_EOTIE);
                    }else reg::set(std::ref(spiHandle_->IER), static_cast<std::uint32_t>(interrupt));
                    
                }

                void disableInterrupt(interrupt interrupt){
                    //All these three interrupts are affected by this bit
                    if(interrupt == interrupt::endoftransfer || interrupt == interrupt::suspend || interrupt == interrupt::txcomplete){
                        reg::clear(std::ref(spiHandle_->IER), SPI_IER_EOTIE);
                    }else reg::clear(std::ref(spiHandle_->IER), static_cast<std::uint32_t>(interrupt));
                }

                bool getInterruptFlag(interrupt interrupt){
                    return static_cast<bool>(reg::read(std::ref(spiHandle_->SR), static_cast<std::uint32_t>(interrupt)));
                }

                void clearInterruptFlag(interrupt interrupt){
                    reg::read(std::ref(spiHandle_->IFCR), static_cast<std::uint32_t>(interrupt));
                }

                bool rxFifoNotEmpty(){
                    return static_cast<bool>(reg::read(std::ref(spiHandle_->SR), SPI_SR_RXWNE));
                }

                std::uint8_t rxFifoFramesAvailable(){
                    return reg::read(std::ref(spiHandle_->SR), 0x03, SPI_SR_RXPLVL_Pos);
                }

                std::uint16_t remainingNumberOfData(){
                    return reg::read(std::ref(spiHandle_->SR), 0xFFFF, SPI_SR_CTSIZE_Pos);
                }

                void setUnderrunPattern(uint32_t pattern){
                    reg::write(std::ref(spiHandle_->UDRDR), pattern);
                }

    };

}

#endif
