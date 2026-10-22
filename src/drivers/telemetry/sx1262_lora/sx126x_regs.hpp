#pragma once
#include <cstdint>

namespace sx126x
{

enum class OpCode : uint8_t {
	GetStatus               = 0xC0,
	WriteRegister            = 0x0D,
	ReadRegister             = 0x1D,
	WriteBuffer              = 0x0E,
	ReadBuffer               = 0x1E,
	SetSleep                 = 0x84,
	SetStandby               = 0x80,
	SetFs                    = 0xC1,
	SetTx                    = 0x83,
	SetRx                    = 0x82,
	SetPacketType            = 0x8A,
	GetPacketType            = 0x11,
	SetRfFrequency           = 0x86,
	SetTxParams              = 0x8E,
	SetPaConfig              = 0x95,
	SetBufferBaseAddress     = 0x8F,
	SetModulationParams      = 0x8B,
	SetPacketParams          = 0x8C,
	GetRxBufferStatus        = 0x13,
	GetPacketStatus          = 0x14,
	SetDioIrqParams          = 0x08,
	GetIrqStatus             = 0x12,
	ClearIrqStatus           = 0x02,
	Calibrate                = 0x89,
	CalibrateImage           = 0x98,
	SetRegulatorMode         = 0x96,
	GetDeviceErrors          = 0x17,
	ClearDeviceErrors        = 0x07,
};

namespace irq
{
static constexpr uint16_t TxDone        = 1u << 0;
static constexpr uint16_t RxDone        = 1u << 1;
static constexpr uint16_t HeaderErr     = 1u << 5;
static constexpr uint16_t CrcErr        = 1u << 6;
static constexpr uint16_t Timeout       = 1u << 9;
static constexpr uint16_t All           = 0xFFFF;
} // namespace irq

// STANDBY_RC vs STANDBY_XOSC for SetStandby param
static constexpr uint8_t STDBY_RC   = 0x00;
static constexpr uint8_t STDBY_XOSC = 0x01;

// SetRegulatorMode
static constexpr uint8_t REGULATOR_LDO_ONLY = 0x00;
static constexpr uint8_t REGULATOR_DC_DC    = 0x01; // check E22 module's actual regulator config before using

static constexpr uint8_t PACKET_TYPE_LORA = 0x01;

// LoRa spreading factor raw values for SetModulationParams
enum LoRaSF : uint8_t { SF7 = 7, SF8 = 8, SF9 = 9, SF10 = 10 };

// LoRa bandwidth raw values for SetModulationParams
enum LoRaBW : uint8_t { BW125 = 0x04, BW250 = 0x05, BW500 = 0x06 };

// LoRa coding rate raw values for SetModulationParams
enum LoRaCR : uint8_t { CR4_5 = 1, CR4_6 = 2, CR4_7 = 3, CR4_8 = 4 };

/**
 * Converts an RF frequency in Hz to the 32-bit register value the SX126x expects.
 * freq_reg = round(freq_hz * 2^25 / 32e6)
 */
static inline uint32_t freq_to_reg(uint32_t freq_hz)
{
	return (uint32_t)(((uint64_t)freq_hz << 25) / 32000000ULL);
}

} // namespace sx126x
