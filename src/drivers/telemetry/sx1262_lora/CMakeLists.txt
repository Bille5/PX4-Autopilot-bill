px4_add_module(
	MODULE drivers__telemetry__sx1262_lora
	MAIN sx1262_lora
	COMPILE_FLAGS
	SRCS
		SX1262.cpp
		LoraLink.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
	)
