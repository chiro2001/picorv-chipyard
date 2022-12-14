bootrom_img = bootrom.rv32.img bootrom.rv32.simple.img start.rv32.img
bootrom_dump = bootrom.rv32.dump bootrom.rv32.simple.dump start.rv32.dump

CROSS_COMPILE := riscv-none-embed-
GCC=$(CROSS_COMPILE)gcc
CFLAGS_RV32=-mabi=ilp32 -march=rv32im
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

START_FILES := irq.c print.c

default: img dump

img: $(bootrom_img)

dump: $(bootrom_dump)

%.img: %.elf
	$(OBJCOPY) -O binary --change-addresses=-0x10000 $< $@

start.rv32.elf: start.S linker.ld
	$(GCC) $(CFLAGS_RV32) -Tlinker.ld start.S $(START_FILES) -nostdlib -static -o start.rv32.elf

%.rv32.elf: %.S linker.ld
	$(GCC) $(CFLAGS_RV32) -Tlinker.ld $< -nostdlib -static -o $@

%.rv32.simple.elf: %_simple.S linker.ld
	$(GCC) $(CFLAGS_RV32) -Tlinker.ld bootrom_simple.S -nostdlib -static -o $@

%.dump: %.elf
	$(OBJDUMP) -d $< > $@

clean:
	rm -f *.elf *.dump *.img
