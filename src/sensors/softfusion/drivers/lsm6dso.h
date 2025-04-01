/*
        SlimeVR Code is placed under the MIT license
        Copyright (c) 2025 Shade_Emry & SlimeVR Contributors

        Permission is hereby granted, free of charge, to any person obtaining a copy
        of this software and associated documentation files (the "Software"), to deal
        in the Software without restriction, including without limitation the rights
        to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
        copies of the Software, and to permit persons to whom the Software is
        furnished to do so, subject to the following conditions:

        The above copyright notice and this permission notice shall be included in
        all copies or substantial portions of the Software.

        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
        IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
        FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
        AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
        LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
        OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
        THE SOFTWARE.
*/

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "lsm6ds-common.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver config: accelerometer ±8g, gyroscope ±1000 dps; gyro ODR = 416 Hz, accel ODR = 104 Hz
// (Using ±8g instead of ±16g gives finer resolution since typical VR movements stay <8g)

template <typename I2CImpl>
struct LSM6DSO : LSM6DSOutputHandler<I2CImpl> {
        static constexpr uint8_t Address = 0x6a;
        static constexpr auto Name = "LSM6DSO";
        static constexpr auto Type = SensorTypeID::LSM6DSO;

        static constexpr float GyrFreq = 416;
        static constexpr float AccFreq = 104;
        static constexpr float MagFreq = 120;
        static constexpr float TempFreq = 52;

        static constexpr float GyrTs = 1.0 / GyrFreq;
        static constexpr float AccTs = 1.0 / AccFreq;
        static constexpr float MagTs = 1.0 / MagFreq;
        static constexpr float TempTs = 1.0 / TempFreq;

        static constexpr float GyroSensitivity = 1000 / 35.0f;   // 35 mdps/LSB at ±1000 dps FS
        static constexpr float AccelSensitivity = 1000 / 0.244f; // 0.244 mg/LSB at ±8g FS

        static constexpr float TemperatureBias = 25.0f;
        static constexpr float TemperatureSensitivity = 256.0f;

        static constexpr float TemperatureZROChange = 10.0;

		static constexpr VQFParams SensorVQFParams{
			.motionBiasEstEnabled = true,
			.biasSigmaInit = 0.3f,         // Ultra-fast convergence for dynamic full-body motions
			.biasClip = 5.0f,              // Expanded range for extreme limb movements
			.restThGyr = 0.3f,             // Precision rest detection (0.3°/s)
			.restThAcc = 0.08f,            // Ultra-sensitive accelerometer stillness threshold
			
		};
        using LSM6DSOutputHandler<I2CImpl>::i2c;

        struct Regs {
                struct WhoAmI {
                        static constexpr uint8_t reg = 0x0f;
                        static constexpr uint8_t value = 0x6c;
                };
                struct Ctrl1XL {
                        static constexpr uint8_t reg = 0x10;
                        static constexpr uint8_t value = 0b01001100;  // Accel @104Hz, ±8g full-scale
                };
                struct Ctrl2GY {
                        static constexpr uint8_t reg = 0x11;
                        static constexpr uint8_t value = 0b01101000;  // Gyro @416Hz, ±1000 dps full-scale
                };
                struct Ctrl3C {
                        static constexpr uint8_t reg = 0x12;
                        static constexpr uint8_t valueSwReset = 1;
                        static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU=1 (block data update), IF_INC=1 (auto-increment registers)
                };
                struct FifoCtrl3BDR {
                        static constexpr uint8_t reg = 0x09;
                        static constexpr uint8_t value
                                = (0b0110 << 4) | (0b0100);  // FIFO batch rates: gyro=417Hz, accel=104Hz (match sensor ODRs)
                };
                struct FifoCtrl4Mode {
                        static constexpr uint8_t reg = 0x0a;
                        static constexpr uint8_t value = 0b110110;    // FIFO mode = continuous; temperature batch rate = 52Hz
                };

                static constexpr uint8_t FifoStatus = 0x3a; // FIFO status register (flags/level)
                static constexpr uint8_t FifoData   = 0x78; // FIFO data output register (read sensor data stream)
        };

        LSM6DSO(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
                : LSM6DSOutputHandler<I2CImpl>(i2c, logger) {}

        bool initialize() {
                // Perform sensor reset and configuration sequence
                i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
                delay(20); // small delay for reset to take effect
                // Set accelerometer ODR and full-scale (104 Hz, ±8g)
                i2c.writeReg(Regs::Ctrl1XL::reg, Regs::Ctrl1XL::value);
                // Set gyroscope ODR and full-scale (416 Hz, ±1000 dps)
                i2c.writeReg(Regs::Ctrl2GY::reg, Regs::Ctrl2GY::value);
                // Configure common control (BDU=1 to lock data until read, IF_INC=1 to auto-increment register addresses)
                i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
                // Configure FIFO: set gyro/accel batching rates and FIFO operating mode
                i2c.writeReg(Regs::FifoCtrl3BDR::reg, Regs::FifoCtrl3BDR::value);
                i2c.writeReg(Regs::FifoCtrl4Mode::reg, Regs::FifoCtrl4Mode::value);
                return true;
        }

        template <typename AccelCall, typename GyroCall, typename TempCall>
        void bulkRead(
                AccelCall&& processAccelSample,
                GyroCall&& processGyroSample,
                TempCall&& processTempSample
        ) {
                // Use base class bulkRead implementation to retrieve all queued samples from FIFO
                LSM6DSOutputHandler<I2CImpl>::
                        template bulkRead<AccelCall, GyroCall, TempCall, Regs>(
                                processAccelSample,
                                processGyroSample,
                                processTempSample,
                                GyrTs,
                                AccTs,
                                TempTs
                        );
        }
};

// Future enhancements to consider:
// - Support dynamic range switching (auto-ranging) if needed for extreme motions
// - Allow configurable ODR or power modes via firmware settings
// - Verify if disabling FIFO when not in use could save power (currently using continuous FIFO to stream data)

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
