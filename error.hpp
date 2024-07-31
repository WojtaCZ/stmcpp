/* 
 * This file is part of the stmcpp distribution (https://github.com/WojtaCZ/stmcpp).
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

#ifndef STMCPP_FAULT_H
#define STMCPP_FAULT_H

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace stmcpp::error {

    // Don't forget to define the globalFaultHandler!!!
    extern "C" void globalFaultHandler(std::uint32_t hash, std::uint32_t code);

    // Hash multiplier is set experimentally
    constexpr unsigned int HASH_MULTIPLIER = 37;

    constexpr std::uint32_t moduleHash(const char * str) {
        std::uint32_t hash_ = 0;

        for (; *str != '\0'; str++){
            hash_ = HASH_MULTIPLIER * hash_ + *str;
        }
        return hash_;
    }

    // This class serves as a wrapper to allow for compile time hash calculation
    template<size_t N>
    struct StringLiteral {
        char value[N];

        constexpr StringLiteral(const char (&str)[N]) {
            std::copy_n(str, N, value);
        }
    };

    template<typename ErrorEnum, StringLiteral ModuleName>
    class handler {

        static_assert(std::is_enum<ErrorEnum>::value, "The ErrorEnum parameter must be an enum!");

        private:
            // Hash function to get the module hash from its name
            template<size_t N>
            static constexpr std::uint32_t hashFunction (StringLiteral<N> s) {
                std::uint32_t hash_ = 0;

                for (unsigned int i = 0; i < sizeof(s.value) - 1; i++) {
                    hash_ = HASH_MULTIPLIER * hash_ + s.value[i];
                }
                return hash_;
            }

            // Some statuses
            ErrorEnum lastSoftError_ = static_cast<ErrorEnum>(0);
            bool hasSoftError_ = false;
            static constexpr std::uint32_t moduleHash_ = handler::hashFunction(ModuleName);


        public:
            constexpr void softThrow(ErrorEnum error) {
                lastSoftError_ = error;
                hasSoftError_ = true;
            }

            constexpr bool hasSoftError() {
                return hasSoftError_;
            }

            constexpr ErrorEnum getSoftError() {
                return lastSoftError_;
            }

            constexpr void clearSoftError() {
                hasSoftError_ = false;
            }

            constexpr void hardThrow(ErrorEnum error) {
                globalFaultHandler(moduleHash_, static_cast<std::uint32_t>(error));
            }

            constexpr std::uint32_t getHash() {
                return moduleHash_;
            }
    };    
} 

#endif