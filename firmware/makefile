PROJECT           := bb-8
DEVICES           := LPC1768
GCC4MBED_DIR      := ../gcc4mbed
NO_FLOAT_SCANF    := 1

# Allow firmware to be debugged wirelessly with MRI via esp-link.
GCC4MBED_TYPE     := Checked
MRI_BREAK_ON_INIT := 0
MRI_UART          := MRI_UART_MBED_P13_P14 MRI_UART_BAUD=460800

include $(GCC4MBED_DIR)/build/gcc4mbed.mk
