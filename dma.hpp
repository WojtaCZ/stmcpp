#ifndef DMA_H
#define DMA_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include "register.hpp"
#include "stm32h753xx.h"

namespace dma{
    enum class peripheral : uint32_t {
        dma1 = DMA1_BASE,
        dma2 = DMA2_BASE
    };

    enum class stream {
        stream0 = 0x010UL,
        stream1 = 0x028UL,
        stream2 = 0x040UL,
        stream3 = 0x058UL,
        stream4 = 0x070UL,
        stream5 = 0x088UL,
        stream6 = 0x0A0UL,
        stream7 = 0x0B8UL,
    };

    enum class priority{
        low = 0b00,
        medium = 0b01,
        high = 0b10,
        veryhigh = 0b11
    };

    enum class mode{
        periph2mem = 0b00,
        mem2periph = 0b01,
        mem2mem = 0b10,
    };

    enum class datasize{
        byte = 0b00,
        halfword = 0b01,
        word = 0b10
    };

    enum class burstsize{
        single = 0b00,
        incremental4 = 0b01,
        incremental8 = 0b10,
        incremental16 = 0b11
    };

    enum class flowcontroller{
        dma = 0b0,
        peripheral = 0b1,
    };

    enum class pincoffset{
        psize = 0b0,
        word = 0b1,
    };

    enum class targetmem{
        mem0 = 0b0,
        mem1 = 0b1,
    };

    enum class interrupt : uint32_t{
        transferComplete        = 0x0020,
        transferHalfComplete    = 0x0010,
        transferError           = 0x0008,
        directModeError         = 0x0004,
        fifoError               = 0x0001
    };

    enum class fifostat{
        empty = 0b100,
        quarter = 0b000,
        half = 0b001,
        threequarter = 0b010, 
        almostfull = 0b011,
        full = 0b101
    };

    enum class fifotreshold{
        quarter = 0b00,
        half = 0b01,
        threequarter = 0b10, 
        full = 0b11
    };

    template<peripheral Peripheral, stream Stream, mode mode_, datasize psize_, bool pincrement_, datasize msize_, bool mincrement_, uint16_t numofdata_,
             priority priority_ = priority::low, bool circular_ = false, pincoffset pincoffset_ = pincoffset::psize, bool doublebuffer_ = false, bool bufferedtransfers_ = false, flowcontroller flowcontroller_ = flowcontroller::dma, burstsize pburst_ = burstsize::single, burstsize mburst_ = burstsize::single>
    class dma{
        private:
            DMA_Stream_TypeDef * const streamHandle_ = reinterpret_cast<DMA_Stream_TypeDef *>(static_cast<std::uint32_t>(Peripheral) + static_cast<std::uint32_t>(Stream));
        public:
            dma(uint32_t paddress_, uint32_t m0address_, uint32_t m1address_ = 0){
                reg::write(std::ref(streamHandle_->CR),
                    //Interrupts are not enabled here and the channel is not yet being enabled
                    ((static_cast<std::uint8_t>(flowcontroller_) & 0b1) << DMA_SxCR_PFCTRL_Pos) |
                    ((static_cast<std::uint8_t>(mode_) & 0b11) << DMA_SxCR_DIR_Pos) | 
                    ((static_cast<std::uint8_t>(circular_) & 0b1) << DMA_SxCR_CIRC_Pos) |
                    ((static_cast<std::uint8_t>(pincrement_) & 0b1) << DMA_SxCR_PINC_Pos) |
                    ((static_cast<std::uint8_t>(mincrement_) & 0b1) << DMA_SxCR_MINC_Pos) |
                    ((static_cast<std::uint8_t>(psize_) & 0b11) << DMA_SxCR_PSIZE_Pos) | 
                    ((static_cast<std::uint8_t>(msize_) & 0b11) << DMA_SxCR_MSIZE_Pos) | 
                    ((static_cast<std::uint8_t>(pincoffset_) & 0b1) << DMA_SxCR_PINC_Pos) |
                    ((static_cast<std::uint8_t>(priority_) & 0b11) << DMA_SxCR_PL_Pos) | 
                    ((static_cast<std::uint8_t>(doublebuffer_) & 0b1) << DMA_SxCR_DBM_Pos) |
                    //Current target is not set here
                    ((static_cast<std::uint8_t>(bufferedtransfers_) & 0b1) << DMA_SxCR_TRBUFF_Pos) |
                    ((static_cast<std::uint8_t>(pburst_) & 0b11) << DMA_SxCR_PBURST_Pos) | 
                    ((static_cast<std::uint8_t>(mburst_) & 0b11) << DMA_SxCR_MBURST_Pos)  
                );

                reg::write(std::ref(streamHandle_->PAR), paddress_);
                reg::write(std::ref(streamHandle_->M0AR), m0address_);
                reg::write(std::ref(streamHandle_->M1AR), m1address_);
                reg::write(std::ref(streamHandle_->NDTR), numofdata_);
            }

            void enable(){
                reg::set(std::ref(streamHandle_->CR), DMA_SxCR_EN);
            }

            void disable(){
                reg::clear(std::ref(streamHandle_->CR), DMA_SxCR_EN);
            }

            void setPeriphAddress(uint32_t add_){

            }

            void setTargetMemory(targetmem mem_){
                reg::change(std::ref(streamHandle_->CR), 0x01, static_cast<std::uint8_t>(mem_), DMA_SxCR_CT_Pos);
            }

            void enableInterrupt(const std::vector<interrupt> int_){
                uint8_t mask_ = 0;

                for(auto i : int_){
                    if(i == interrupt::fifoError){
                        //If the fifo interrupt should be enabled, set it right away
                        reg::set(std::ref(streamHandle_->CR), DMA_SxFCR_FEIE);
                    }else{
                        switch(int_){
                        case interrupt::transferHalfComplete:
                            mask_ |= DMA_SxCR_HTIE;
                            break;
                        case interrupt::transferComplete:
                            mask_ |= DMA_SxCR_TCIE;
                            break;
                        case interrupt::transferError:
                            mask_ |= DMA_SxCR_TEIE;
                            break;
                        case interrupt::directModeError:
                            mask_ |= DMA_SxCR_DMEIE;
                            break;
                        
                        default:
                            break;
                        }
                    }
                }

                //Apply the mask to the control register
                reg::set(std::ref(streamHandle_->CR), mask_);
            }

            void enableInterrupt(interrupt int_){
                if(int_ == interrupt::fifoError){
                    //If the fifo interrupt should be enabled, set it
                    reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, FCR)>(int_);
                }else{
                    //If the interrupt in the CR should be set, set it
                     switch(int_){
                        case interrupt::transferHalfComplete:
                            reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_HTIE_Pos);
                            break;
                        case interrupt::transferComplete:
                            reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_TCIE_Pos);
                            break;
                        case interrupt::transferError:
                            reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_TEIE_Pos);
                            break;
                        case interrupt::directModeError:
                            reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_DMEIE_Pos);
                            break;
                        
                        default:
                            break;
                        }
                }
            }

            void disableInterrupt(std::vector<interrupt> int_){
                uint8_t mask_ = 0;

                for(auto i : int_){
                    if(i == interrupt::fifoError){
                        //If the fifo interrupt should be enabled, set it right away
                        reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, FCR)>(0b1 << DMA_SxFCR_FEIE_Pos);
                    }else{
                        switch(int_){
                        case interrupt::transferHalfComplete:
                            mask_ |= (0b1 << DMA_SxCR_HTIE_Pos);
                            break;
                        case interrupt::transferComplete:
                            mask_ |= (0b1 << DMA_SxCR_TCIE_Pos);
                            break;
                        case interrupt::transferError:
                            mask_ |= (0b1 << DMA_SxCR_TEIE_Pos);
                            break;
                        case interrupt::directModeError:
                            mask_ |= (0b1 << DMA_SxCR_DMEIE_Pos);
                            break;
                        
                        default:
                            break;
                        }
                    }
                }

                //Apply the mask to the control register
                reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(mask_);
            }

            void disableInterrupt(interrupt int_){
                if(int_ == interrupt::fifoError){
                    //If the fifo interrupt should be enabled, set it
                    reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, FCR)>(static_cast<uint32_t>(int_));
                }else{
                    //If the interrupt in the CR should be set, set it
                     switch(int_){
                        case interrupt::transferHalfComplete:
                            reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_HTIE_Pos);
                            break;
                        case interrupt::transferComplete:
                            reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_TCIE_Pos);
                            break;
                        case interrupt::transferError:
                            reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_TEIE_Pos);
                            break;
                        case interrupt::directModeError:
                            reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, CR)>(0b1 << DMA_SxCR_DMEIE_Pos);
                            break;
                        
                        default:
                            break;
                        }
                }
            }


            void clearInterruptFlag(interrupt int_){
                switch (stream_){
                    case stream::stream0:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LIFCR)>(static_cast<uint32_t>(int_));
                        break;
                    case stream::stream1:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LIFCR)>(static_cast<uint32_t>(int_) << 5);
                        break;
                    case stream::stream2:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LIFCR)>(static_cast<uint32_t>(int_) << 10);
                        break;
                    case stream::stream3:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LIFCR)>(static_cast<uint32_t>(int_) << 15);
                        break;
                    case stream::stream4:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HIFCR)>(static_cast<uint32_t>(int_));
                        break;
                    case stream::stream5:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HIFCR)>(static_cast<uint32_t>(int_) << 5);
                        break;
                    case stream::stream6:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HIFCR)>(static_cast<uint32_t>(int_) << 10);
                        break;
                    case stream::stream7:
                        reg::write<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HIFCR)>(static_cast<uint32_t>(int_) << 15);
                        break;
                    
                    default:
                        break;
                }
            }

            bool getInterruptFlag(interrupt int_){
                switch (stream_){
                    case stream::stream0:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LISR)>(static_cast<uint32_t>(int_)));
                        break;
                    case stream::stream1:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LISR)>(static_cast<uint32_t>(int_) << 5));
                        break;
                    case stream::stream2:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LISR)>(static_cast<uint32_t>(int_) << 10));
                        break;
                    case stream::stream3:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, LISR)>(static_cast<uint32_t>(int_) << 15));
                        break;
                    case stream::stream4:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HISR)>(static_cast<uint32_t>(int_)));
                        break;
                    case stream::stream5:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HISR)>(static_cast<uint32_t>(int_) << 5));
                        break;
                    case stream::stream6:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HISR)>(static_cast<uint32_t>(int_) << 10));
                        break;
                    case stream::stream7:
                        return static_cast<bool>(reg::read<static_cast<uint32_t>(peripheral_) + offsetof(DMA_TypeDef, HISR)>(static_cast<uint32_t>(int_) << 15));
                        break;
                    
                    default:
                        break;
                }
            }

            void setFifoTreshold(fifotreshold ft_){
                reg::clear<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, FCR)>(0b11 << DMA_SxFCR_FTH_Pos);
                reg::set<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, FCR)>(static_cast<std::uint8_t>(ft_) << DMA_SxFCR_FTH_Pos);
            }

            fifostat getFifoStatus(){
                return static_cast<fifostat>(reg::read<static_cast<uint32_t>(peripheral_) + static_cast<uint32_t>(stream_) + offsetof(DMA_Stream_TypeDef, FCR)>(0b111 << DMA_SxFCR_FS_Pos));
            }

    };

}



#endif
