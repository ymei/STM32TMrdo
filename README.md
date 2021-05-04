# STM32TMrdo
Topmetal control and readout using STM32 and its internal ADC

# STM32H43ZITx NUCLEO-LQFP144
  - USB VCP (on OTG port) could be opened with `python -m serial.tools.miniterm /dev/cu.usbmodemFD131 115200`
  - PB1 (on CN12) is ADC12_INP5, which is a fast channel.
  - D-Cache could cause [DMA issues](https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices)
  - [Examples](https://github.com/STMicroelectronics/STM32CubeH7) are very valuable.
## ADC
  - t_conv = 4.5 CLK (8bit) ~ 8.5 CLK (16bit), 1 CLK for every 2 bits.
  - t_samp >= 1.5 CLK.
  - 16-bit mode max sample rate is 3.6 Msps.
  - ADC1 and ADC2 are in AHB1/D2 (domain 2).
  - Fadc = 75MHz / a select factor.  Max is 36MHz (BOOST=1), 20MHz (BOOST=0).

# STM32 ARM MCU firmware
  - `STM32CubeMX` is used to configure the pin function/clock and setup the basic software skeleton.
    - Make sure SYS->Debug = Serial Wire is selected.  Otherwise future flash writing will be disabled.
    - Choose `Makefile` under `Toolchain/IDE` for GNU-RM compatible skeleton.
    - Choose HAL set all free pins as analog and Enable full assert.
    - Make minimal modification to the generated code.  Place the majority of user code in separate files one level up to the generated code directory.
    - It seems asking the software to update the already generated files is not reliable.  Better delete all generated files and re-generate.
    - After copying the files over, run `cleanCube.sh` to clean up the file permissions and add additional information into `Makefile`.  New `makefile` is generated and `make` will pick up the new one automatically.  Modify `cleanCube.sh` accordingly when new `.c` files are added.
    - A new `main.c` is generated as well.  This way, no manual intervention to any of the STM32CubeMX generated files is needed.
## Useful commands
  - `readelf -a xxx.elf` or `objdump -h xxx.elf` to see the built image size.  Add up the size of all the loadable sections, such as `.text` and `.data`.  Look for the `LOAD` flag in the output from `objdump`.  Ignore non-loadable sections such as `.comment`, `.debug` and `.bss`.
## USB CDC ACM
  - (https://damogranlabs.com/2018/02/stm32-usb-cdc/).
## Toolchain
  - [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
  - [GNU](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
    - Download and put into `~/Applications/arm`.
    - `~/Applications/arm/env.sh`:
    ```
    #!/bin/sh

    ARMROOT=$HOME/Applications/arm
    export PATH=$ARMROOT/bin:$ARMROOT/arm-none-eabi/bin:$PATH
    export LD_LIBRARY_PATH=$ARMROOT/lib:$LD_LIBRARY_PATH
    export MANPATH=$ARMROOT/share/man:$MANPATH
    ```
    - `arm-none-eabi-objdump -d build/main.o` to see the assembly code generated by `gcc`.
  - [stlink stm32 discovery line linux programmer](https://github.com/texane/stlink)
    - Install under `~/Applications/arm/`
    ```
    $ mkdir build && cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/arm ..
    $ make -j5
    $ make DESTDIR=~/Applications install
    ```
    - `st-util`, `st-flash`, `st-info` to download image to/manipulate STM32 MCU.
  - [In-Application Programming (IAP)](https://www.st.com/en/embedded-software/x-cube-iap-usart.html)
  - [Open source ARM Cortex-M microcontroller library libopencm3](https://github.com/libopencm3/libopencm3)
  - [J-Link Debug Probes](https://www.segger.com/products/debug-probes/j-link/)
  - [Black Magic Probe: In application debugger for ARM Cortex microcontrollers](https://github.com/blacksphere/blackmagic)
  - STM32CubeProgrammer
    - Write firmware then start running:  `~/Applications/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI -c port=SWD -w build/STM32H743ZITxCubeMX.hex -s`
    - For NUCLEO (-H7 at least), remember to hold down reset (black button), start the command, then release the button.
      - Enable Debug (SWD) in CubeMX project to solve this problem.
