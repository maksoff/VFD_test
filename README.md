# About this project
This project is used for test of some ideas, like VFD display, FRAM memory, and nRF24L01+ modules.
To ease the work with nRF modules, additional nRFF project will be implemented.

# nRFF project
Idea of this project to combine some ideas and create lightweight **secure** *mesh* (or, dynamic-tree) network, with MQTT-like interface.

## Targets:
* Security (with modern SDR packets without any protection can be easily decoded)
	+ `nonce` to prevent replay attack
	+ XXTEA block coding - lightweight and secure for short packets
* Message-based (like MQTT)
	+ eliminated the need of knowing the exact module address & co
	+ network can be easily extended with new modules
	+ 12 bit message id (+4 bit for message counter)
* Mesh network (more preicesily: dynamic-tree)
	+ dynamically configurable network
		- one server, for managing network and message distributions
		- static addresses for fixed devices for manual network configuration
		- dynamic addresses for flexible network configurations, which can be stored for faster connection
		- dynamic addresses for moving devices, which are never stored
## Limitations
* 24 bytes pro message. Can be solved by multi-packet transmision
* 12 bit message ID

## TODOs
* uMQTT
* XXTEA + nonce
* Network communication