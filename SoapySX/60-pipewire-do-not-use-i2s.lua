-- Configuration file to prevent PipeWire from trying to use
-- the I2S ALSA device on Raspberry Pi as a normal sound card.
-- This should be placed at /usr/share/wireplumber/main.lua.d/
-- or ~/.config/wireplumber/main.lua.d/
--
-- Based on https://blog.zenlinux.com/2022/08/how-to-disable-audio-devices-in-pipewire-wireplumber/

table.insert(alsa_monitor.rules, {
	matches = {{
		{ "device.name", "equals", "alsa_card.platform-soc_sound" },
	}},
	apply_properties = {
		["device.disabled"] = true,
	},
})
