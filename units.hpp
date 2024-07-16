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

#ifndef STMCPP_UNITS_H
#define STMCPP_UNITS_H

#include <cstdint>
#include <cstddef>
#include <cmath>
#include "register.hpp"
#include "stm32h753xx.h"

#include <cstdint>
#include <limits>

#include <type_traits>
#include <cstdint>

namespace stmcpp::units {

	template<typename StorageType, typename ConcreteType>
	struct Unit {
        using StorageType_ = StorageType;
        StorageType_ raw_;

        constexpr ConcreteType operator+ (ConcreteType rhs) const { 
            return { static_cast<StorageType_>(raw_ + rhs.raw_) }; 
        }

        constexpr ConcreteType& operator+= (ConcreteType rhs) { 
            raw_ += rhs.raw_;
            return static_cast<ConcreteType&>(*this);
        }
        
        constexpr ConcreteType operator- () const { 
            return { static_cast<StorageType_>(-raw_) };
        }        

        constexpr ConcreteType operator- (ConcreteType rhs) const {
            return { static_cast<StorageType_>(raw_ - rhs.raw_) };
        }

        constexpr ConcreteType& operator-= (ConcreteType rhs) { 
            raw_ -= rhs.raw_;
            return static_cast<ConcreteType&>(*this);
        }

        template<typename Scalar>
        constexpr friend ConcreteType operator* (Scalar s, Unit q) {
            return { static_cast<StorageType_>(q.raw_ * s) };
        }

        template<typename Scalar>
        constexpr friend ConcreteType operator* (Unit q, Scalar s) {
            return { static_cast<StorageType_>(q.raw_ * s) };
        }

        template<typename Scalar> 
        constexpr ConcreteType& operator*=(Scalar s) {
            raw_ *= s;
            return static_cast<ConcreteType&>(*this);
        }

        template<typename Scalar>
        constexpr std::enable_if_t<std::is_arithmetic_v<Scalar>, ConcreteType> operator/ (Scalar s) const {
            return { static_cast<StorageType_>(raw_ / s) };
        }
        
        constexpr float operator/ (Unit q) const {
            return raw_ / static_cast<float>(q.raw_);
        }

        template<typename Scalar>
        constexpr ConcreteType& operator/=(Scalar s) {
            raw_ /= s;
            return static_cast<ConcreteType&>(*this);
        }

        constexpr friend bool operator<     (ConcreteType lhs, ConcreteType rhs) { return lhs.raw_ < rhs.raw_; }
        constexpr friend bool operator>     (ConcreteType lhs, ConcreteType rhs) { return lhs.raw_ > rhs.raw_; }
        constexpr friend bool operator==    (ConcreteType lhs, ConcreteType rhs) { return lhs.raw_ == rhs.raw_; }
        constexpr friend bool operator!=    (ConcreteType lhs, ConcreteType rhs) { return lhs.raw_ != rhs.raw_; }
        constexpr friend bool operator>=    (ConcreteType lhs, ConcreteType rhs) { return lhs.raw_ >= rhs.raw_; }
        constexpr friend bool operator<=    (ConcreteType lhs, ConcreteType rhs) { return lhs.raw_ <= rhs.raw_; }
	};

    /*
        Current unit (stored in uA or pA)
    */

    // Change the storage type based on the range required
    #ifdef STMCPP_UNITS_VOLTAGE_HIGHRANGE
        using VoltageStorage = std::int64_t; // Allow for Megavolt to Picovolt range (value stored in picovolts)
        static constexpr auto voltageScaleFactor_ = 1'000'000;
    #else
        using VoltageStorage = std::int32_t; // Allow for Kilovolt to Microvolt range (value stored in microvolts)
        static constexpr auto voltageScaleFactor_ = 1;
    #endif

	struct Voltage : public Unit<VoltageStorage, Voltage> {
		template<typename T>
		constexpr static Voltage fromVolts(T volts) {
            return { static_cast<StorageType_>(volts * (1'000'000ll * voltageScaleFactor_)) };
        }

        template<typename T>
		constexpr static Voltage fromMilliVolts(T millivolts) {
            return { static_cast<StorageType_>(millivolts * (1'000 * voltageScaleFactor_)) };
        }

        template<typename T>
		constexpr static Voltage fromMicroVolts(T microvolts) {
            return { static_cast<StorageType_>(microvolts * voltageScaleFactor_) };
        }

        constexpr float toMicroVolts() const {
            return raw_ / (1.0f * voltageScaleFactor_);
        }

		constexpr float toMilliVolts() const {
            return raw_ / (1'000.0f * voltageScaleFactor_);
        }

		constexpr float toVolts() const {
            return raw_ / (1'000'000.0f * voltageScaleFactor_);
        }

        #ifdef STMCPP_UNITS_VOLTAGE_HIGHRANGE
            template<typename T>
            constexpr static Voltage fromKiloVolts(T kilovolts) {
                return { static_cast<StorageType_>(kilovolts * (1'000'000'000ll * voltageScaleFactor_)) };
            }

            template<typename T>
            constexpr static Voltage fromNanoVolts(T nanovolts) {
                return { static_cast<StorageType_>(nanovolts * (voltageScaleFactor_ / 1'000)) };
            }

            template<typename T>
            constexpr static Voltage fromPicoVolts(T picovolts) {
                return { static_cast<StorageType_>(picovolts *  (voltageScaleFactor_ / 1'000'000)) };
            }

            constexpr float toKiloVolts() const {
                return raw_ / (1'000'000'000.0f * voltageScaleFactor_);
            }

            constexpr float toNanoVolts() const {
                return raw_ / (1.0f * (voltageScaleFactor_ / 1'000ll));
            }

            constexpr StorageType_ toPicoVolts() const {
                return raw_;
            }
        #endif

	};

    constexpr Voltage operator""_V(unsigned long long voltage) { return Voltage::fromVolts(voltage); }
    constexpr Voltage operator""_mV(unsigned long long voltage) { return Voltage::fromMilliVolts(voltage); }
    constexpr Voltage operator""_uV(unsigned long long voltage) { return Voltage::fromMicroVolts(voltage); }
    
    static_assert((1_V + 111_mV) - 1111000_uV == 0_V, "Voltage units check.");

    #ifdef STMCPP_UNITS_VOLTAGE_HIGHRANGE
        constexpr Voltage operator""_kV(unsigned long long voltage) { return Voltage::fromKiloVolts(voltage); }
        constexpr Voltage operator""_nV(unsigned long long voltage) { return Voltage::fromNanoVolts(voltage); }
        constexpr Voltage operator""_pV(unsigned long long voltage) { return Voltage::fromPicoVolts(voltage); }

        static_assert((123_kV + 456_V + 789_mV + 123_uV + 456_nV) - 123456789123456000_pV == 0_V, "Voltage units check.");
    #endif

    /*
        Current unit (stored in uA or pA)
    */

    // Change the storage type based on the range required
    #ifdef STMCPP_UNITS_CURRENT_HIGHRANGE
        using CurrentStorage = std::int64_t; // Allow for Megaampere to Picoampere range (value stored in picoampere)
        static constexpr auto currentScaleFactor_ = 1'000'000;
    #else
        using CurrentStorage = std::int32_t; // Allow for Kiloampere to Microampere range (value stored in microampere)
        static constexpr auto currentScaleFactor_ = 1;
    #endif

	struct Current : public Unit<CurrentStorage, Current> {
		template<typename T>
		constexpr static Current fromAmperes(T amperes) {
            return { static_cast<StorageType_>(amperes * (1'000'000ll * currentScaleFactor_)) };
        }

        template<typename T>
		constexpr static Current fromMilliAmperes(T milliamperes) {
            return { static_cast<StorageType_>(milliamperes * (1'000 * currentScaleFactor_)) };
        }

        template<typename T>
		constexpr static Current fromMicroAmperes(T microamperes) {
            return { static_cast<StorageType_>(microamperes * currentScaleFactor_) };
        }

        constexpr float toMicroAmperes() const {
            return raw_ / (1.0f * currentScaleFactor_);
        }

		constexpr float toMilliAmperes() const {
            return raw_ / (1'000.0f * currentScaleFactor_);
        }

		constexpr float toAmperes() const {
            return raw_ / (1'000'000.0f * currentScaleFactor_);
        }

        #ifdef STMCPP_UNITS_CURRENT_HIGHRANGE
            template<typename T>
            constexpr static Current fromKiloAmperes(T kiloamperes) {
                return { static_cast<StorageType_>(kiloamperes * (1'000'000'000ll * currentScaleFactor_)) };
            }

            template<typename T>
            constexpr static Current fromNanoAmperes(T nanoamperes) {
                return { static_cast<StorageType_>(nanoamperes * (currentScaleFactor_ / 1'000)) };
            }

            template<typename T>
            constexpr static Current fromPicoAmperes(T picoamperes) {
                return { static_cast<StorageType_>(picoamperes *  (currentScaleFactor_ / 1'000'000)) };
            }

            constexpr float toKiloAmperes() const {
                return raw_ / (1'000'000'000.0f * currentScaleFactor_);
            }

            constexpr float toNanoAmperes() const {
                return raw_ / (1.0f * (currentScaleFactor_ / 1'000ll));
            }

            constexpr StorageType_ toPicoAmperes() const {
                return raw_;
            }
        #endif

	};

    constexpr Current operator""_A(unsigned long long current) { return Current::fromAmperes(current); }
    constexpr Current operator""_mA(unsigned long long current) { return Current::fromMilliAmperes(current); }
    constexpr Current operator""_uA(unsigned long long current) { return Current::fromMicroAmperes(current); }
    
    static_assert((123_A + 456_mA) - 123456000_uA == 0_A, "Current units check.");

    #ifdef STMCPP_UNITS_CURRENT_HIGHRANGE
        constexpr Current operator""_kA(unsigned long long voltage) { return Current::fromKiloAmperes(current); }
        constexpr Current operator""_nA(unsigned long long voltage) { return Current::fromNanoAmperes(current); }
        constexpr Current operator""_pA(unsigned long long voltage) { return Current::fromPicoAmperes(current); }

        static_assert((123_kA + 456_A + 789_mA + 123_uA + 456_nA) - 123456789123456000_pA == 0_A, "Current units check.");
    #endif

    /*
        Duration unit (stored in us or ps)
    */

    // Change the storage type based on the range required
    #ifdef STMCPP_UNITS_DURATION_HIGHRANGE
        using DurationStorage = std::uint64_t; // Allow for higher range: ~213days @ 1ps resolution
        static constexpr auto durationScaleFactor_ = 1'000'000;
    #else
        using DurationStorage = std::uint32_t; // Allow for a range of ~35minutes @ 1us resolution
        static constexpr auto durationScaleFactor_ = 1;
    #endif

	struct Duration : Unit<DurationStorage, Duration> {
        constexpr static Duration fromMicroSeconds(unsigned microseconds) {
            return { microseconds * durationScaleFactor_};
        }

		constexpr static Duration fromMilliSeconds(unsigned milliseconds) {
            return { milliseconds * (1'000 * durationScaleFactor_) };
        }

		constexpr static Duration fromSeconds(unsigned seconds) {
            return { seconds * (1'000'000 * durationScaleFactor_) };
        }

        constexpr float toMicroSeconds() const {
            return raw_ / (1.0f * durationScaleFactor_);
        }

		constexpr float toMilliSeconds() const {
            return raw_ / (1'000.0f * durationScaleFactor_);
        }

		constexpr float toSeconds() const {
            return raw_ / (1'000'000.0f * durationScaleFactor_);
        }

        #ifdef STMCPP_UNITS_DURATION_HIGHRANGE
            constexpr static Duration fromNanoSeconds(unsigned nanoseconds) {
                return { microseconds * (durationScaleFactor_ / 1'000)};
            }

            constexpr static Duration fromPicoSeconds(unsigned picoseconds) {
                return { microseconds * (durationScaleFactor_ / 1'000)};
            }

            constexpr float toDays() const {
                return raw_ / (24.0f * 60.0f * 60.0f * durationScaleFactor_);
            }

            constexpr float toHours() const {
                return raw_ / (60.0f * 60.0f * durationScaleFactor_);
            }

            constexpr float toMinutes() const {
                return raw_ / (60.0f * durationScaleFactor_);
            }

            constexpr float toNanoSeconds() const {
                return raw_ / (1.0f * (voltageScaleFactor_ / 1'000ll));
            }

            constexpr DurationStorage toPicoSeconds() const {
                return raw_;
            }
        #endif
	};

    constexpr Duration operator""_us(unsigned long long duration) { return Duration::fromMicroSeconds(duration); }
    constexpr Duration operator""_ms(unsigned long long duration) { return Duration::fromMilliSeconds(duration); }
    constexpr Duration operator""_s(unsigned long long duration) { return Duration::fromSeconds(duration); }

    static_assert((123_s + 456_ms) - 123456000_us == 0_s, "Duration units check.");

    #ifdef STMCPP_UNITS_DURATION_HIGHRANGE
        constexpr Duration operator""_ns(unsigned long long duration) { return Duration::fromNanoSeconds(duration); }
        constexpr Duration operator""_ps(unsigned long long duration) { return Duration::fromPicoSeconds(duration); }

        static_assert((123_s + 456_ms + 789_us + 123_ns) - 123456789123000_ps == 0_s, "Duration units check.");
    #endif

    /*
        Frequency unit (stored in hertz)
    */
    struct Frequency : Unit<std::uint32_t, Frequency> {
		template<typename T>
		constexpr static Frequency fromHertz(T hertz) {
            return { static_cast<StorageType_>(hertz) };
        }

		template<typename T>
		constexpr static Frequency fromKiloHertz(T kilohertz) {
            return { static_cast<StorageType_>(kilohertz * 1'000) };
        }

		template<typename T>
		constexpr static Frequency fromMegaHertz(T megahertz) {
            return { static_cast<StorageType_>(megahertz * 1'000'000) };
        }

        template<typename T>
		constexpr static Frequency fromGigaHertz(T gigahertz) {
            return { static_cast<StorageType_>(gigahertz * 1'000'000'000) };
        }

		constexpr StorageType_ toHertz() const { 
            return raw_;
        }

        constexpr float toKiloHertz() const { 
            return raw_ / 1'000.0f;
        }

        constexpr float toMegaHertz() const { 
            return raw_ / 1'000'000.0f;
        }

		constexpr Duration period() const { 
            return Duration::fromMicroSeconds(1'000'000 / raw_);
        }
	};

    constexpr Frequency operator""_Hz(unsigned long long freq) { return Frequency::fromHertz(freq); }
    constexpr Frequency operator""_kHz(unsigned long long freq) { return Frequency::fromKiloHertz(freq); }
    constexpr Frequency operator""_MHz(unsigned long long freq) { return Frequency::fromMegaHertz(freq); }

    static_assert((123_MHz + 456_kHz) - 123456000_Hz == 0_Hz, "Frequency units check.");
}

#endif