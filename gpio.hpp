#ifndef GPIO_H
#define GPIO_H

#include <cstdint>
#include <cstddef>
#include "stm32h753xx.h"

#include "register.hpp"

namespace gpio{
    enum class port : std::uintptr_t {
        porta = GPIOA_BASE,
        portb = GPIOB_BASE,
        portc = GPIOC_BASE,
        portd = GPIOD_BASE,
        
        #ifdef GPIOE_BASE
        porte = GPIOE_BASE,
        #endif

        #ifdef GPIOF_BASE
        portf = GPIOF_BASE,
        #endif

        #ifdef GPIOG_BASE
        portg = GPIOG_BASE,
        #endif

        #ifdef GPIOH_BASE
        porth = GPIOH_BASE,
        #endif

        #ifdef GPIOI_BASE
        porti = GPIOI_BASE,
        #endif

        #ifdef GPIOJ_BASE
        portj = GPIOJ_BASE,
        #endif

        #ifdef GPIOK_BASE
        portk = GPIOK_BASE,
        #endif

        #ifdef GPIOZ_BASE
        portz = GPIOZ_BASE
        #endif
    };

    enum class mode : std::uint8_t{
        input     = 0b00000000,
        output    = 0b00000001,
        af0       = 0b00000010,
        af1       = 0b00010010,
        af2       = 0b00100010,
        af3       = 0b00110010,
        af4       = 0b01000010,
        af5       = 0b01010010,
        af6       = 0b01100010,
        af7       = 0b01110010,
        af8       = 0b10000010,
        af9       = 0b10010010,
        af10      = 0b10100010,
        af11      = 0b10110010,
        af12      = 0b11000010,
        af13      = 0b11010010,
        af14      = 0b11100010,
        af15      = 0b11110010,
        analog    = 0b00000011
    };

    enum class speed : std::uint8_t{
        low       = 0b00,
        medium    = 0b01,
        high      = 0b10,
        veryhigh  = 0b11
    };

    enum class pull : std::uint8_t{
        nopull    = 0b00,
        pullup    = 0b01,
        pulldown  = 0b10
    };

    enum class otype : std::uint8_t {
        pushpull  = 0b0,
        opendrain = 0b1
    };

    namespace interrupt {
        enum class edge {
            rising,
            falling,
            both
        };
    }


    template<gpio::port Port, std::uint8_t Pin>
    class pin{
        private:
            GPIO_TypeDef * const gpioHandle_ = reinterpret_cast<GPIO_TypeDef *>(Port);

        public:
            constexpr pin(gpio::mode mode = gpio::mode::input, gpio::otype otype = gpio::otype::pushpull, gpio::pull pull = gpio::pull::nopull, gpio::speed speed = gpio::speed::low) {
                static_assert(Pin < 16, "The pin number cannot be greater than 15!");
               
                setMode(mode);
                setSpeed(speed);
                setPull(pull);
                setOutputType(otype);
            };

            void set() const {
                reg::write(std::ref(gpioHandle_->BSRR), 0x0001, Pin);
            }

            void clear() const {
                reg::write(std::ref(gpioHandle_->BSRR), 0x0100, Pin);
            }

            void write(bool state) const {
                state ? set() : clear();
            }

            void toggle() const {
                reg::toggle(std::ref(gpioHandle_->ODR), 0x0001, Pin);
            }

            bool read() const {
                //The internal reg library mask bitshift is intentionally not used to save on bitshift operations
                return static_cast<bool>(reg::read(std::ref(gpioHandle_->IDR), 0x0001 << Pin));
            }

            void setMode(gpio::mode mode) const {
                reg::change(std::ref(gpioHandle_->MODER), 0x03, static_cast<std::uint8_t>(mode), Pin * 2);

                constexpr auto afrIndex_ = (Pin < 8) ? 0 : 1; 
                constexpr auto afrShift_ = (Pin % 8) * 4;
                //Upper half of the mode byte holds the AF information
                reg::change(std::ref(gpioHandle_->AFR[afrIndex_]), 0x0F, static_cast<std::uint8_t>(mode) >> 4, afrShift_);
            }

            void setSpeed(gpio::speed speed) const {
                reg::change(std::ref(gpioHandle_->OSPEEDR), 0x03, static_cast<std::uint8_t>(speed), Pin * 2);
            }

            void setPull(gpio::pull pull) const {
                reg::change(std::ref(gpioHandle_->PUPDR), 0x03, static_cast<std::uint8_t>(pull), Pin * 2);
            }

            void setOutputType(gpio::otype otype) const {
               reg::change(std::ref(gpioHandle_->OTYPER), 0x01, static_cast<std::uint8_t>(otype), Pin);
            }

            void enableInterrupt(gpio::interrupt::edge edge) const {
                
                constexpr auto extiIndex_ = static_cast<unsigned int>(Pin / 4);
                constexpr auto extiShift_ = (Pin % 4) * 4;
                constexpr auto extiPort_ = (static_cast<uint32_t>(Port) - GPIOA_BASE) / 0x0400UL;

                reg::change(std::ref(SYSCFG->EXTICR[extiIndex_]), 0x0F, extiPort_, extiShift_);

                setInterruptEdge(edge);

                reg::set(std::ref(EXTI->IMR1), 0x01, Pin);
            }

            void disableInterrupt() const {
                reg::clear(std::ref(EXTI->IMR1), 0x01, Pin);
                reg::clear(std::ref(EXTI->RTSR1), 0x01, Pin);
                reg::clear(std::ref(EXTI->FTSR1), 0x01, Pin);
            }

            void clearInterruptFlag() const {
                reg::set(std::ref(EXTI->PR1), 0x01, Pin);
            }

            void setInterruptEdge(gpio::interrupt::edge edge) const {
                if(edge == gpio::interrupt::edge::rising){
                    reg::change(std::ref(EXTI->RTSR1), 0x01, 0x01, Pin);
                }else if(edge == gpio::interrupt::edge::falling){
                    reg::change(std::ref(EXTI->FTSR1), 0x01, 0x01, Pin);
                }else if(edge == gpio::interrupt::edge::both){
                    reg::change(std::ref(EXTI->RTSR1), 0x01, 0x01, Pin);
                    reg::change(std::ref(EXTI->FTSR1), 0x01, 0x01, Pin);
                }
            }
    };    
}

#endif
