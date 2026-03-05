<a name="readme-top"></a>

<h1 align="center">
  <br> STM32_MDIO_Master <br>
</h1>

<div align="center">

[![Orel138 - STM32_MDIO_Master](https://img.shields.io/static/v1?label=Orel138&message=STM32_MDIO_Master&color=blue&logo=github)](https://github.com/Orel138/STM32_MDIO_Master "Go to GitHub repo")
[![stars - STM32_MDIO_Master](https://img.shields.io/github/stars/Orel138/STM32_MDIO_Master?style=social)](https://github.com/Orel138/STM32_MDIO_Master)
[![forks - STM32_MDIO_Master](https://img.shields.io/github/forks/Orel138/STM32_MDIO_Master?style=social)](https://github.com/Orel138/STM32_MDIO_Master)

[![Open in Visual Studio Code](https://img.shields.io/static/v1?logo=visualstudiocode&label=&message=Open%20in%20Visual%20Studio%20Code&labelColor=2c2c32&color=007acc&logoColor=007acc)](https://open.vscode.dev/Orel138/STM32_MDIO_Master)
[![license](https://custom-icon-badges.demolab.com/github/license/Orel138/STM32_MDIO_Master?logo=law&logoColor=white)](https://github.com/Orel138/STM32_MDIO_Master/blob/main/LICENSE "license MIT")
[![issues](https://custom-icon-badges.demolab.com/github/issues-raw/Orel138/STM32_MDIO_Master?logo=issue)](https://github.com/Orel138/STM32_MDIO_Master/issues "issues")

[![STM32](https://img.shields.io/badge/STM32-message?style=flat&logo=stmicroelectronics&color=%2303234B)](https://st.com "STM32")
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-message?style=flat&logo=freertos&color=%23000000)](https://freertos.org/ "FreeRTOS")

</div>

<div align="center">
  <h4>
    <a href="#about">About</a> |
    <a href="#key-goals">Key Goals</a> |
    <a href="#architecture-overview">Architecture</a> |
    <a href="#requirements">Requirements</a> |
    <a href="#installation">Installation</a> |
    <a href="#usage">Usage</a> |
    <a href="#references">References</a> |
    <a href="#license">License</a>
  </h4>
</div>

<div align="center">
  <sub>Built by
  <a href="https://orel138.github.io">Orel138</a> and
  <a href="https://github.com/orel138/STM32_MDIO_Master/graphs/contributors">contributors</a>
</div>
<br>

## About

**STM32 MDIO Master Example** is an implementation example of an MDIO Master device using STM32 microcontrollers with the FreeRTOS kernel. This project demonstrates how to configure and use the MDIO peripheral in master mode to communicate with PHY devices, allowing read/write access to PHY registers via the MDIO protocol.

The implementation is based on STM32Cube HAL and integrates with FreeRTOS for task management, logging, and CLI interface. It supports read/write operations on PHY registers and handles interrupts for efficient data exchange.

## Table of Contents

- [About](#about)
- [Key Goals](#key-goals)
- [Architecture Overview](#architecture-overview)
- [Project Structure](#project-structure)
- [Design Principles](#design-principles)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [References](#references)
- [Contributing](#contributing)
- [License](#license)

## Key Goals

- Demonstrate MDIO Master functionality on STM32 devices
- Provide a FreeRTOS-integrated example with logging and CLI
- Ensure compatibility with STM32Cube HAL and BSP
- Offer a foundation for PHY device communication in embedded systems

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Architecture Overview

### High-Level Architecture

```
STM32 MDIO Peripheral (Master Mode)
        │
        ▼
FreeRTOS Tasks (Heartbeat, CLI, Logging)
```

### Project Structure

```
.
├── Drivers/
│   ├── CMSIS/               # ARM Cortex-M CMSIS headers
│   └── STM32H7xx_HAL_Driver/ # STM32 HAL drivers
│
├── Middlewares/
│   └── Third_Party/
│       └── FreeRTOS/        # FreeRTOS kernel
│
├── Projects/
│   ├── Common/
│   │   ├── cli/             # Command Line Interface utilities
│   │   └── config/          # Common configuration files
│   └── NUCLEO-H723ZG/       # Board-specific projects
│       └── Applications/
│           └── ETH_MDIO_Master/  # Main application
│
└── README.md                # This file
```

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Design Principles

- HAL-based peripheral initialization
- FreeRTOS task-based architecture
- Interrupt-driven MDIO operations
- PHY register access management
- Logging and CLI for debugging and interaction

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Requirements

### Hardware

- NUCLEO-H723ZG or NUCLEO-H743ZI
- MDIO slave device (e.g., PHY device) : see https://github.com/Orel138/STM32_MDIO_Slave

### Software

- STM32CubeIDE or compatible toolchain
- FreeRTOS kernel
- STM32Cube HAL for H7 series

### Development Environment

- Git
- STM32CubeIDE or VS Code with STM32 extension

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Installation

Clone the repository:

```bash
git clone https://github.com/Orel138/STM32_MDIO_Master.git
```

Enter the project directory:

```bash
cd STM32_MDIO_Master
```

Open the project in STM32CubeIDE:

- Import the project from `Projects/NUCLEO-H723ZG/Applications/ETH_MDIO_Master`
- Build and flash to the target board

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Usage

### Running the Example

1. Flash the firmware to the STM32 Nucleo board
2. Connect an MDIO slave device (e.g., PHY device)
3. Use the UART CLI for interaction and monitoring
4. Observe LED heartbeat and MDIO transactions

### CLI Commands

The project includes a CLI for system interaction. Available commands include:

- `ps` - List running tasks
- `kill` - Signal tasks
- `heapstat` - Display heap statistics
- `uptime` - Show system uptime
- `reset` - Reboot the system
- `mdio` - Perform MDIO Master operations (write/read PHY registers)

### MDIO Operations

The MDIO master can communicate with PHY devices containing up to 32 registers. Write operations send data to PHY registers, and read operations retrieve register values. Interrupts handle transaction completion efficiently.

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## References

- [STM32Cube MCU Overall Offer](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer)
- [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
- [STM32 MDIO Peripheral Documentation](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743-753-and-stm32h750-value-line-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
- [MDIO Protocol Specification](https://en.wikipedia.org/wiki/Management_Data_Input/Output)

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## Contributing

Contributions are welcome.

To contribute:

1. Fork the repository
2. Create a feature branch
3. Commit your changes with clear messages
4. Open a pull request

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

## License

This project is released under the [MIT License](LICENSE).

© [Orel138](https://github.com/Orel138)

<p align="right"><a href="#readme-top">~~~~~ back to top ~~~~~</a></p>

> [!TIP]
> If you find this project useful, consider giving it a ⭐.
> It is the simplest way to show support and helps the project grow.
