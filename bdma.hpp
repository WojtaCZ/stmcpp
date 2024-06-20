#ifndef BDMA_H
#define BDMA_H

#include <cstdint>
#include <cstddef>
#include "register.hpp"
#include "stm32h753xx.h"

namespace bdma{
    enum class peripheral : std::uint32_t {
        dma1 = BDMA_BASE
    };

    enum class channel : std::uint32_t {
        channel0 = 0x008UL,
        channel1 = 0x01CUL,
        channel2 = 0x030UL,
        channel3 = 0x044UL,
        channel4 = 0x058UL,
        channel5 = 0x06CUL,
        channel6 = 0x080UL,
        channel7 = 0x094UL,
    };

    enum class priority : std::uint8_t {
        low = 0b00,
        medium = 0b01,
        high = 0b10,
        veryhigh = 0b11
    };

    enum class mode : std::uint8_t {
        periph2mem = 0b00,
        mem2periph = 0b01,
        mem2mem = 0b10,
    };

    enum class datasize : std::uint8_t {
        byte = 0b00,
        halfword = 0b01,
        word = 0b10
    };

    enum class pincoffset : std::uint8_t {
        psize = 0b0,
        word = 0b1,
    };

    enum class targetmem : std::uint8_t {
        mem0 = 0b0,
        mem1 = 0b1,
    };

    enum class interrupt : std::uint8_t{
        global                  = 0x01, 
        transferComplete        = 0x02,
        transferHalfComplete    = 0x04,
        transferError           = 0x08
    };


    template<peripheral Peripheral, channel Channel>
    class bdma{
        private:
            BDMA_Channel_TypeDef * const channelHandle_ = reinterpret_cast<DMA_Stream_TypeDef *>(static_cast<std::uint32_t>(Peripheral) + static_cast<std::uint32_t>(Channel));
            BDMA_TypeDef * const bdmaHandle_ = reinterpret_cast<DMA_TypeDef *>(static_cast<std::uint32_t>(Peripheral));
        public:
            bdma(mode mode, datasize psize, bool pincrement, std::uint32_t paddress, datasize msize, bool mincrement, uint32_t m0address, uint32_t m1address, uint16_t numofdata,
                 priority priority = priority::low, bool circular = false, pincoffset pincoffset = pincoffset::psize, bool doublebuffer = false>
            ){
                reg::write(std::ref(channelHandle_->CCR),
                    //Interrupts are not enabled here and the channel is not yet being enabled
                    ((static_cast<uint8_t>(mode) & 0b01) << BDMA_CCR_DIR_Pos) |
                    ((static_cast<uint8_t>(circular) & 0b1) << BDMA_CCR_CIRC_Pos) |
                    ((static_cast<uint8_t>(pincrement) & 0b1) << BDMA_CCR_PINC_Pos) |
                    ((static_cast<uint8_t>(mincrement) & 0b1) << BDMA_CCR_MINC_Pos) |
                    ((static_cast<uint8_t>(psize) & 0b11) << BDMA_CCR_PSIZE_Pos) | 
                    ((static_cast<uint8_t>(msize) & 0b11) << BDMA_CCR_MSIZE_Pos) | 
                    ((static_cast<uint8_t>(priority) & 0b11) << BDMA_CCR_PL_Pos) | 
                    ((static_cast<uint8_t>(mode) & 0b10) << BDMA_CCR_MEM2MEM_Pos - 1) | 
                    ((static_cast<uint8_t>(doublebuffer) & 0b1) << BDMA_CCR_DBM_Pos) 
                    //Current target is not set here
                );
                reg::write(std::ref(channelHandle_->CPAR), paddress);
                reg::write(std::ref(channelHandle_->CM0AR), m0address);
                reg::write(std::ref(channelHandle_->CM1AR), m1address);
                reg::write(std::ref(channelHandle_->CNDTR), numofdata);
            }

            void enable(){
                reg::set(std::ref(channelHandle_->CCR), BDMA_CCR_EN);
            }

            void disable(){
                reg::clear(std::ref(channelHandle_->CCR), BDMA_CCR_EN);
            }

            void setTargetMemory(targetmem memory){
                reg::change(std::ref(channelHandle_->CCR), 0x01, static_cast<std::uint8_t>(memory), BDMA_CCR_CT_Pos);
            }

            void enableInterrupt(std::vector<interrupt> interrupts){

                std::uint8_t mask_ = 0;

                for(auto i : interrupts){
                    //Merge the interrupts into single mask
                    switch(i){
                    case interrupt::transferHalfComplete:
                        mask_ |= BDMA_CCR_HTIE;
                        break;
                    case interrupt::transferComplete:
                        mask_ |= BDMA_CCR_TCIE;
                        break;
                    case interrupt::transferError:
                        mask_ |= BDMA_CCR_TEIE;

                        
                    default:
                        break;
                    }
                    
                }
                
                //Apply the mask to the control register
                reg::set(std::ref(channelHandle_->CCR), mask_);
            }

            void enableInterrupt(interrupt interrupt){
                switch(interrupt){
                case interrupt::transferHalfComplete:
                    reg::set(std::ref(channelHandle_->CCR), BDMA_CCR_HTIE);
                    break;
                case interrupt::transferComplete:
                    reg::set(std::ref(channelHandle_->CCR), BDMA_CCR_TCIE);
                    break;
                case interrupt::transferError:
                    reg::set(std::ref(channelHandle_->CCR), BDMA_CCR_TEIE); 
                default:
                    break;
                }
            }

            void disableInterrupt(std::vector<interrupt> interrupts){

                std::uint8_t mask_ = 0;

                for(auto i : interrupts){
                    //Merge the interrupts into single mask
                    switch(i){
                    case interrupt::transferHalfComplete:
                        mask_ |= BDMA_CCR_HTIE;
                        break;
                    case interrupt::transferComplete:
                        mask_ |= BDMA_CCR_TCIE;
                        break;
                    case interrupt::transferError:
                        mask_ |= BDMA_CCR_TEIE;

                    default:
                        break;
                    }
                    
                }
                
                //Apply the mask to the control register
                reg::set(std::ref(channelHandle_->CCR), mask_);
            }

            void disableInterrupt(interrupt interrupt){
                switch(interrupt){
                case interrupt::transferHalfComplete:
                    reg::clear(std::ref(channelHandle_->CCR), BDMA_CCR_HTIE);
                    break;
                case interrupt::transferComplete:
                    reg::clear(std::ref(channelHandle_->CCR), BDMA_CCR_TCIE);
                    break;
                case interrupt::transferError:
                    reg::clear(std::ref(channelHandle_->CCR), BDMA_CCR_TEIE); 
                default:
                    break;
                }
            }

            void clearInterruptFlag(interrupt interrupt){
                switch (Channel){
                    case channel::channel0:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 0);
                    case channel::channel1:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 4);
                    case channel::channel2:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 8);
                    case channel::channel3:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 12);
                    case channel::channel4:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 16);
                    case channel::channel5:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 20);
                    case channel::channel6:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 24);
                    case channel::channel7:
                        reg::set(std::ref(bdmaHandle_->IFCR), interrupt, 28);
                    
                    default:
                        break;
                }
            }

            bool getInterruptFlag(interrupt interrupt){
                switch (Channel){
                    case channel::channel0:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 0));
                    case channel::channel1:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 4));
                    case channel::channel2:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 8));
                    case channel::channel3:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 12));
                    case channel::channel4:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 16));
                    case channel::channel5:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 20));
                    case channel::channel6:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 24));
                    case channel::channel7:
                        return static_cast<bool>(reg::set(std::ref(bdmaHandle_->ISR), interrupt, 28));
                    
                    default:
                        break;
                }
            }

    };

}



#endif
