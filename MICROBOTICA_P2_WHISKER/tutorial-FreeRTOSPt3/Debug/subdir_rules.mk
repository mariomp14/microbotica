################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --include_path="C:/Users/Usuario/Desktop/MICROBOTICA_PRACTICAS/microbotica/MICROBOTICA_P2_WHISKER/tutorial-FreeRTOSPt3" --include_path="C:/Users/Usuario/Desktop/MICROBOTICA_PRACTICAS/microbotica/MICROBOTICA_P2_WHISKER/tutorial-FreeRTOSPt3/FreeRTOS/Source/include" --include_path="C:/Users/Usuario/Desktop/MICROBOTICA_PRACTICAS/microbotica/MICROBOTICA_P2_WHISKER/tutorial-FreeRTOSPt3/FreeRTOS/Source/portable/CCS/ARM_CM4F" --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --define=UART_BUFFERED --define=WANT_CMDLINE_HISTORY --define=WANT_FREERTOS_SUPPORT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


