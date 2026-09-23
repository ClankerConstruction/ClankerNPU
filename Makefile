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
NPUDBG ?= 0
# 1 forwards reordered frames to the wired side instead of the host,
# which needs a configured PPE
HWFAST ?= 0

ARCH    := -march=rv32imc_zicsr_zifencei -mabi=ilp32
CFLAGS  := $(ARCH) -Os -ffunction-sections -fdata-sections \
           -fno-builtin -ffreestanding -nostdlib \
           -Wall -Wno-unused-function \
           -D$(SOC) -D$(WIFI) $(if $(filter 1,$(MAILTRACE)),-DNPU_MAIL_TRACE) \
           $(if $(filter 0,$(NPUTX)),-DEAGLE_NO_TX_PUSH) \
           $(if $(filter 1,$(NPUDBG)),-DNPU_DATAPATH_DBG) \
           $(if $(filter 1,$(HWFAST)),-DEAGLE_HW_FASTPATH)
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
# npu_globals.c must stay first: it is the only source of .data, so its
# order fixes the layout of npu_data.bin.
SRCS_C  := npu_globals.c \
           npu_main.c npu_util.c npu_mutex.c npu_plic.c npu_timer.c \
           npu_mbox.c npu_sram.c npu_bridge.c npu_printf.c \
           npu_dba.c npu_tr471.c \
           npu_wifi.c npu_wifi_bufid.c npu_wifi_fwd.c npu_wifi_ba.c \
           npu_wifi_rx.c npu_wifi_init.c \
           npu_wifi_kite.c npu_wifi_eagle.c npu_wifi_eagle_dp.c \
           npu_tdma.c npu_hostadpt.c \
           npu_tunnel.c npu_ppe.c npu_l4s.c
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

$(BUILD)/%.o: %.c npu_config.h npu_regs.h npu_types.h npu_internal.h \
		 npu_wifi.h $(FLAGS) | $(BUILD)
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
