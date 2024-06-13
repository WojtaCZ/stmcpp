#ifndef CLOCK_H
#define CLOCK_H

#include <cstdint>
#include <tuple>

#include "register.hpp"
#include "stm32h753xx.h"

#define STM32H7

namespace clock{
    
    void init();
        
    namespace domain {

        enum class d1cpre : uint8_t {
            div1    = 0b0000,
            div2    = 0b1000,
            div4    = 0b1001,
            div8    = 0b1010,
            div16   = 0b1011,
            div64   = 0b1100,
            div128  = 0b1101,
            div256  = 0b1110,
            div512  = 0b1111,
        };

        enum class d1ppre : uint8_t {
            div1    = 0b000,
            div2    = 0b100,
            div4    = 0b101,
            div8    = 0b110,
            div16   = 0b111,
        };

        enum class hpre : uint8_t {
            div1    = 0b0000,
            div2    = 0b1000,
            div4    = 0b1001,
            div8    = 0b1010,
            div16   = 0b1011,
            div64   = 0b1100,
            div128  = 0b1101,
            div256  = 0b1110,
            div512  = 0b1111,
        };

        enum class d2ppre1 : uint8_t {
            div1    = 0b000,
            div2    = 0b100,
            div4    = 0b101,
            div8    = 0b110,
            div16   = 0b111,
        };

        enum class d2ppre2 : uint8_t {
            div1    = 0b000,
            div2    = 0b100,
            div4    = 0b101,
            div8    = 0b110,
            div16   = 0b111,
        };

        enum class d3ppre : uint8_t {
            div1    = 0b000,
            div2    = 0b100,
            div4    = 0b101,
            div8    = 0b110,
            div16   = 0b111,
        };

        enum class source : uint8_t {
            hsi     = 0b000,
            csi     = 0b001,
            hse     = 0b010,
            pll1    = 0b011
        };

        class domain{
            public:
                constexpr domain(clock::domain::d1cpre d1cpre, clock::domain::d1ppre d1ppre, clock::domain::hpre hpre, clock::domain::d2ppre1 d2ppre1, clock::domain::d2ppre2 d2ppre2, clock::domain::d3ppre d3ppre, clock::domain::source source = clock::domain::source::hsi){
                        reg::write(std::ref(RCC->D1CFGR), 
                            static_cast<uint8_t>(d1cpre) << RCC_D1CFGR_D1CPRE_Pos |
                            static_cast<uint8_t>(d1ppre) << RCC_D1CFGR_D1PPRE_Pos |
                            static_cast<uint8_t>(hpre)   << RCC_D1CFGR_HPRE_Pos 
                        );

                        reg::write(std::ref(RCC->D2CFGR), 
                            static_cast<uint8_t>(d2ppre1) << RCC_D2CFGR_D2PPRE1_Pos |
                            static_cast<uint8_t>(d2ppre2) << RCC_D2CFGR_D2PPRE2_Pos 
                        );

                        reg::write(std::ref(RCC->D3CFGR), static_cast<uint8_t>(d3ppre) << RCC_D3CFGR_D3PPRE_Pos);

                        reg::change(std::ref(RCC->CFGR), 0x03, static_cast<uint8_t>(source), RCC_CFGR_SW_Pos);
                }

                clock::domain::source sourceStatus() const {
                    return static_cast<clock::domain::source>(reg::read(std::ref(RCC->CFGR), 0x07, RCC_CFGR_SWS_Pos));
                }
        };
    
    }

    namespace pll{

        #ifdef STM32H7
            //Ranges for divisor values
            constexpr std::pair<unsigned int, unsigned int> range_div_m = {1u,  63u};
            constexpr std::pair<unsigned int, unsigned int> range_div_n = {4u, 512u};
            constexpr std::pair<unsigned int, unsigned int> range_div_p = {2u, 128u};
            constexpr std::pair<unsigned int, unsigned int> range_div_q = {1u, 128u};
            constexpr std::pair<unsigned int, unsigned int> range_div_r = {1u, 128u};

            static_assert(range_div_m.first < range_div_m.second, "Bottom range bound must be smaller than the top one!");
            static_assert(range_div_n.first < range_div_n.second, "Bottom range bound must be smaller than the top one!");
            static_assert(range_div_p.first < range_div_p.second, "Bottom range bound must be smaller than the top one!");
            static_assert(range_div_q.first < range_div_q.second, "Bottom range bound must be smaller than the top one!");
            static_assert(range_div_r.first < range_div_r.second, "Bottom range bound must be smaller than the top one!");


        #endif

        enum class peripheral : uint32_t {
            pll1 = offsetof(RCC_TypeDef, PLL1DIVR),
            pll2 = offsetof(RCC_TypeDef, PLL2DIVR),
            pll3 = offsetof(RCC_TypeDef, PLL3DIVR)
        };

        enum class inputrange : uint8_t {
            f1_2MHz     = 0b00,
            f2_4MHz     = 0b01,
            f4_8MHz     = 0b10,
            f8_16MHz    = 0b11
        };

        enum class vcorange : uint8_t {
            f192_960MHz     = 0b0,
            f150_420MHz     = 0b1,
        };

        enum class clksource : uint8_t {
            hsi,
            csi,
            hse,
        };

        template<peripheral Peripheral, unsigned int M, unsigned int N, unsigned int P, unsigned int Q, unsigned int R, unsigned int Fraction = 0>
        class pll{
            private:
                constexpr auto getPllIdx_() const {
                    return ((static_cast<unsigned int>(Peripheral) - offsetof(RCC_TypeDef, PLL1DIVR)) / 8);
                }
            public:
                constexpr pll(clock::pll::inputrange inputrange = clock::pll::inputrange::f1_2MHz, clock::pll::vcorange vcorange = clock::pll::vcorange::f192_960MHz) {
                    static_assert(M >= clock::pll::range_div_m.first && M <= clock::pll::range_div_m.second, "The M divider is out of range.");
                    static_assert(N >= clock::pll::range_div_n.first && N <= clock::pll::range_div_n.second, "The N divider is out of range.");
                    static_assert(P >= clock::pll::range_div_p.first && P <= clock::pll::range_div_p.second, "The P divider is out of range.");
                    static_assert(Q >= clock::pll::range_div_q.first && Q <= clock::pll::range_div_q.second, "The Q divider is out of range.");
                    static_assert(R >= clock::pll::range_div_r.first && R <= clock::pll::range_div_r.second, "The R divider is out of range.");

                    volatile uint32_t * const pllCfgrAdd_ = reinterpret_cast<volatile uint32_t *>(RCC_BASE + static_cast<std::uint32_t>(Peripheral));

                    reg::write(std::ref(*pllCfgrAdd_), 
                        N << RCC_PLL1DIVR_N1_Pos |
                        P << RCC_PLL1DIVR_P1_Pos |
                        Q << RCC_PLL1DIVR_Q1_Pos |
                        R << RCC_PLL1DIVR_R1_Pos 
                    );

                    volatile uint32_t * const  pllFracAdd_ = reinterpret_cast<volatile uint32_t *>(RCC_BASE + static_cast<std::uint32_t>(Peripheral) + 0x004);

                    reg::write(std::ref(*pllFracAdd_), Fraction, 3);

                    reg::change(std::ref(RCC->PLLCKSELR), 0x3F, M, (8 * getPllIdx_()) + 4);

                    reg::change(std::ref(RCC->PLLCFGR), 0x0E,
                        (
                            static_cast<uint8_t>(inputrange) << RCC_PLLCFGR_PLL1RGE_Pos |
                            static_cast<uint8_t>(vcorange) << RCC_PLLCFGR_PLL1VCOSEL_Pos 
                        ), 4 * getPllIdx_());
                };

                void enable() const{
                    reg::set(std::ref(RCC->CR), 0x01, (2 * getPllIdx_()) + RCC_CR_PLL1ON_Pos);
                }

                void disable() const{
                    reg::set(std::ref(RCC->CR), 0x01, (2 * getPllIdx_()) + RCC_CR_PLL1ON_Pos);
                }

                bool isLocked() const{
                    return static_cast<bool>(reg::read(std::ref(RCC->CR), 0x01, (2 * getPllIdx_()) + RCC_CR_PLL1RDY_Pos));
                }
                
        };    
    }    
}

#endif
