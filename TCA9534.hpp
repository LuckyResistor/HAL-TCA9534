#pragma once
//
// (c)2020 by Lucky Resistor. See LICENSE for details.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//


#include "hal-common/BitTools.hpp"
#include "hal-common/Flags.hpp"
#include "hal-common/StatusTools.hpp"
#include "hal-common/WireMasterChipRegister.hpp"

#include <functional>


namespace lr {


/// A very simple but efficient driver for the TCA9534 chip.
///
/// It uses a three byte cache to prevent unnecessary operations on the I2C bus.
///
class TCA9534
{
public:
    /// The call status for this class
    ///
    using Status = CallStatus;

    /// The call status with a byte result.
    ///
    using ByteStatus = StatusResult<uint8_t>;

    /// The chip address.
    ///
    enum Address : uint8_t {
        Address0 = 0b111000,
        Address1 = 0b111001,
        Address2 = 0b111010,
        Address3 = 0b111011,
        Address4 = 0b111100,
        Address5 = 0b111101,
        Address6 = 0b111110,
        Address7 = 0b111111,
    };

    /// The port.
    ///
    enum Port : uint8_t {
        Port0 = 0,
        Port1 = 1,
        Port2 = 2,
        Port3 = 3,
        Port4 = 4,
        Port5 = 5,
        Port6 = 6,
        Port7 = 7,
    };

    /// The mode.
    ///
    enum Mode : uint8_t {
        Input = 0,
        HighZ = Input,
        Output_High = 1,
        Output_Low = 2,
    };

    /// Update function.
    ///
    /// @param originalValue The original from the cache.
    /// @return The new value after applied update operation.
    ///
    using UpdateFn = std::function<uint8_t(uint8_t originalValue)>;

public:
    /// Create an instance of this driver.
    ///
    /// @param bus The I2C bus where the chip is attached.
    /// @param address The address of the chip.
    ///
    TCA9534(WireMaster *bus, Address address = Address0);

    /// Initialize the driver and the chip.
    ///
    /// - Sets all ports into Hi-Z state
    /// - Sets inversion to zero.
    /// - Sets all outputs to zero.
    ///
    /// @return The status of this operation. Can fail if the chip is not reachable.
    ///
    Status initialize();

    /// Set the port mode.
    ///
    /// This combines configuration and output register in one call.
    ///
    /// @param port The port to change.
    /// @param mode The mode for the port.
    /// @return The status of this operation.
    ///
    Status setMode(Port port, Mode mode);

    /// Get the input mask.
    ///
    /// @return The result with the input mask bits.
    ///
    ByteStatus getInputs();

public: // Low-Level API
    /// The registers of the chip.
    ///
    enum class Register : uint8_t {
        InputPort = 0x00,
        OutputPort = 0x01,
        PolarityInversion = 0x02,
        Configuration = 0x03,
    };

    /// Get the bit from a given port.
    ///
    static constexpr uint8_t bitFromPort(Port port) {
        return oneBit8(static_cast<uint8_t>(port));
    }

    /// Access the bus directly.
    ///
    inline const WireMasterRegisterChip<Register>& bus() {
        return _bus;
    }

public: // Low-Level Caching functions.
    /// Set a cache entry.
    ///
    /// @param reg The register to update. Only `OutputPort`, `PolarityInversion` and `Configuration`.
    /// @param value The new value for the cache.
    ///
    inline void setCache(Register reg, int8_t value) {
        _cache[static_cast<uint8_t>(reg)-1] = value;
    }

    /// Get a cache entry.
    ///
    /// @param reg The register to update. Only `OutputPort`, `PolarityInversion` and `Configuration`.
    /// @return The current value in the cache.
    ///
    inline uint8_t getCache(Register reg) const {
        return _cache[static_cast<uint8_t>(reg)-1];
    }

    /// Update function to change bits using the cache.
    ///
    /// @param reg The register to update. Only `OutputPort`, `PolarityInversion` and `Configuration`.
    /// @param updateFn The function to update the bits.
    /// @return The status of the operation.
    ///
    inline Status updateBits(Register reg, UpdateFn updateFn) {
        uint8_t originalValue = getCache(reg);
        uint8_t newValue = updateFn(originalValue);
        if (newValue != originalValue) {
            setCache(reg, newValue);
            if (hasError(_bus.writeRegister(reg, newValue))) {
                return Status::Error;
            }
        }
        return Status::Success;
    };

    /// Set bits using the cache.
    ///
    /// @param reg The register to update. Only `OutputPort`, `PolarityInversion` and `Configuration`.
    /// @param value The bits to set.
    /// @return The status of the operation.
    ///
    Status setBits(Register reg, int8_t value);

    /// Clear bits using the cache.
    ///
    /// @param reg The register to update. Only `OutputPort`, `PolarityInversion` and `Configuration`.
    /// @param value The bits to clear.
    /// @return The status of the operation.
    ///
    Status clearBits(Register reg, int8_t value);

private:
    const WireMasterRegisterChip<Register> _bus; ///< The bus for the communication.
    int8_t _cache[3]; ///< The cache with the states of the last 3 registers.
};


}