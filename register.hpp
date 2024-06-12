/* 
 * This file is part of the stm-cpp distribution (https://github.com/WojtaCZ/stm-cpp).
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

#ifndef REGISTER_H
#define REGISTER_H

#include <cstdint>
#include <cstddef>
#include <functional>


/*
 * The following file contains the helper function templates
 * used for generic register access throughout the rest of this project.
 */

namespace reg{

    //Use 32 bits as a register size
    using regbase = std::uint32_t;

    template <typename A = regbase, typename V>
    constexpr void write(std::reference_wrapper<A> address, V value, unsigned int bitshift = 0){
        address.get() &= ~(value << bitshift); 
    }

    template <typename A = regbase>
    constexpr A read(std::reference_wrapper<A> address){
        return address.get();
    }

    template <typename A = regbase, typename M>
    constexpr A read(std::reference_wrapper<A> address, M mask, unsigned int bitshift = 0){
        return (address.get() & (mask << bitshift)) >> bitshift;
    }

    template <typename A = regbase, typename M>
    constexpr void change(std::reference_wrapper<A> address, M mask, typename std::type_identity<M>::type value, unsigned int bitshift = 0){
        address.get() = (address.get() & ~(mask << bitshift)) | ((value & mask) << bitshift);
    }

    template <typename A = regbase, typename M>
    constexpr void set(std::reference_wrapper<A> address, M mask, unsigned int bitshift = 0){
        address.get() |= (mask << bitshift);
    }

    template <typename A = regbase, typename M>
    constexpr void clear(std::reference_wrapper<A> address, M mask, unsigned int bitshift = 0){
        address.get() &= ~(mask << bitshift);
    }

    template <typename A = regbase, typename M>
    constexpr void toggle(std::reference_wrapper<A> address, M mask, unsigned int bitshift = 0){
        address.get() ^= (mask << bitshift);
    }

}

#endif
