CROSS   ?= riscv64-unknown-elf-
CC      := $(CROSS)gcc
LD      := $(CROSS)ld
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
SIZE    := $(CROSS)size

# SoC: AN7552, AN7581, AN7583
SOC     ?= AN7583
# WiFi: MT7916, MT7991, MT7992, MT7993, MT7996, NOWIFI
WIFI    ?= MT7996
# 1 logs every WiFi mailbox command, from inside the mailbox ISR
MAILTRACE ?= 0
# 0 stages host tx frames but never writes the WiFi tx ring
NPUTX ?= 1
# 1 reports the datapath counters every two seconds
NPUDBG ?= 1

ARCH    := -march=rv32imc_zicsr_zifencei -mabi=ilp32
CFLAGS  := $(ARCH) -Os -ffunction-sections -fdata-sections \
           -fno-builtin -ffreestanding -nostdlib \
           -Wall -Wno-unused-function \
           -D$(SOC) -D$(WIFI) $(if $(filter 1,$(MAILTRACE)),-DNPU_MAIL_TRACE) \
           $(if $(filter 0,$(NPUTX)),-DEAGLE_NO_TX_PUSH) \
           $(if $(filter 1,$(NPUDBG)),-DNPU_DATAPATH_DBG)
ASFLAGS := $(ARCH) -D$(SOC) -D$(WIFI)
LIBGCC  := $(shell $(CC) $(ARCH) -print-libgcc-file-name)
LDFLAGS := -m elf32lriscv -T link.ld -nostdlib --gc-sections --relax \
           --print-memory-usage

BUILD   := build/$(SOC)_$(WIFI)
ELF     := $(BUILD)/firmware.elf
BIN     := $(BUILD)/npu_rv32.bin
DATA    := $(BUILD)/npu_data.bin
MAP     := $(BUILD)/firmware.map
DIS     := $(BUILD)/firmware.dis

SRCS_S  := crt0.S
SRCS_C  := npu_main.c npu_printf.c npu_wifi.c npu_tunnel.c
OBJS    := $(patsubst %.S,$(BUILD)/%.o,$(SRCS_S)) \
           $(patsubst %.c,$(BUILD)/%.o,$(SRCS_C))
FLAGS   := $(BUILD)/.flags

.PHONY: all clean disasm

all: $(BIN) $(DATA)

$(BUILD):
	mkdir -p $(BUILD)

# rebuild when the build flags change, not just the sources
$(BUILD)/.flags: FORCE | $(BUILD)
	@echo '$(CFLAGS)' > $@.tmp; cmp -s $@.tmp $@ || mv $@.tmp $@; rm -f $@.tmp
FORCE:

$(BUILD)/%.o: %.S npu_config.h $(FLAGS) | $(BUILD)
	$(CC) $(ASFLAGS) -c -o $@ $<

$(BUILD)/%.o: %.c npu_config.h npu_regs.h npu_types.h npu_internal.h $(FLAGS) | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(ELF): $(OBJS) link.ld
	$(LD) $(LDFLAGS) -Map=$(MAP) -o $@ $(OBJS) $(LIBGCC)

$(BIN): $(ELF)
	$(OBJCOPY) -O binary -j .text -j .rodata $< $@

$(DATA): $(ELF)
	$(OBJCOPY) -O binary -j .data $< $@

disasm: $(ELF)
	$(OBJDUMP) -d -M no-aliases $< > $(DIS)

clean:
	rm -rf build/

# Build all 11 variants
VARIANTS := AN7552_MT7916 AN7552_MT7991 AN7552_MT7993 \
            AN7581_MT7916 AN7581_MT7992 AN7581_MT7996 \
            AN7583_MT7916 AN7583_MT7992 AN7583_MT7993 \
            AN7583_MT7996 AN7583_NOWIFI

.PHONY: all-variants $(VARIANTS)

all-variants: $(VARIANTS)

$(VARIANTS):
	$(MAKE) SOC=$(word 1,$(subst _, ,$@)) WIFI=$(word 2,$(subst _, ,$@))
