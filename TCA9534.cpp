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
#include "TCA9534.hpp"


namespace lr {


TCA9534::TCA9534(WireMaster *bus, Address address)
    : _bus(bus, static_cast<uint8_t>(address))
{
}


TCA9534::Status TCA9534::initialize()
{
    const uint8_t data[3] = {0xff, 0x00, 0xff};
    if (hasError(_bus.writeRegisterData(Register::OutputPort, data, 3))) {
        return Status::Error;
    }
    setCache(Register::OutputPort, 0xff);
    setCache(Register::PolarityInversion, 0x00);
    setCache(Register::Configuration, 0xff);
    return Status::Success;
}


TCA9534::Status TCA9534::setMode(Port port, Mode mode)
{
    switch (mode) {
    case Input:
        setBits(Register::Configuration, bitFromPort(port));
        break;
    case Output_High:
        setBits(Register::OutputPort, bitFromPort(port));
        clearBits(Register::Configuration, bitFromPort(port));
        break;
    case Output_Low:
        clearBits(Register::OutputPort, bitFromPort(port));
        clearBits(Register::Configuration, bitFromPort(port));
        break;
    default:
        break;
    }
    return Status::Success;
}


TCA9534::ByteStatus TCA9534::getInputs()
{
    uint8_t value;
    if (hasError(_bus.readRegister(Register::InputPort, value))) {
        return ByteStatus::error();
    }
    return ByteStatus::success(value);
}


TCA9534::Status TCA9534::setBits(Register reg, int8_t value)
{
    return updateBits(reg, [value](uint8_t originalValue) -> uint8_t {
        return originalValue | value;
    });
}


TCA9534::Status TCA9534::clearBits(Register reg, int8_t value)
{
    return updateBits(reg, [value](uint8_t originalValue) -> uint8_t {
        return originalValue & (~value);
    });
}


}

