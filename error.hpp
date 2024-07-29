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

#ifndef STMCPP_FAULT_H
#define STMCPP_FAULT_H

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace stmcpp::error {

    enum class code {
        i2c_nack,
        i2c_bus,
        i2c_arbitration,
        i2c_overrun,
        i2c_timeout,
        i2c_too_large_payload,
        i2c_other,

        systick_used_uninitialized
    };

    
    extern "C" void globalFaultHandler(code errorCode);

    class handler final {
        private:
            static inline code lastSoftError_;
            static inline bool hasSoftError_ = false;
        public:
            handler() = delete;
            handler(const handler &) = delete;
            handler(handler &&) = delete;

            static void hardThrow(code errorCode) {
                globalFaultHandler(errorCode);
            }

            static void softThrow(code errorCode) {
                lastSoftError_ = errorCode;
                hasSoftError_ = true;
            }

            static bool hasSoftError() {
                return hasSoftError_;
            }

            static code getSoftError() {
                return lastSoftError_;
            }

            static void clearSoftError() {
                hasSoftError_ = false;
            }
    };      
} 


#endif