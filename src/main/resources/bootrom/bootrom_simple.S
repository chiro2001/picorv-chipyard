#define BOOTADDR_REG 0x4000

// boot all cores (only hart 0) and jump to main program execution
.section .text.start, "ax", @progbits
.globl _start
_start:
  li a1, 0x2000000 // base address of clint
  addi a2, a1, 4
  li a3, 1

  sw zero, 0(a1)      // clear the interrupt
  li a0, BOOTADDR_REG // program reset vector
#if __riscv_xlen == 32
  lw a0, 0(a0)        // get boot address from bootaddr_reg SCR
#else
  ld a0, 0(a0)        // get boot address from bootaddr_reg SCR
#endif
  # csrw mepc, a0       // return from interrupt to start of user program
  # csrr a0, mhartid    // hartid for next level bootloader
  la a1, _dtb         // dtb address for next level bootloader
  # li a2, 0x80         // set mstatus MPIE to 0
  # csrc mstatus, a2
  # mret // no mret in rv32i
  jalr x0, a0, 0

.align 3
_dtb:
