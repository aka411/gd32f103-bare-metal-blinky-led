/* Linker script for GD32F103C8T6 */


ENTRY(Reset_Handler) /* Define the entry point of the program*/


MEMORY
{
 FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
 RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 20K
}


SECTIONS
{
/* ---------------------------------------------------- */
 /* .isr_vector section (Interrupt Vector Table)        */
 /* ---------------------------------------------------- */
 /* This section must be placed at the very beginning of Flash. */
 .isr_vector :
 {
   . = ALIGN(4); /* Align to 4-byte boundary */
   KEEP (*(.isr_vector)) /* Keep this section even if not explicitly referenced */
 } > FLASH /* Resides in Flash */




 /* ---------------------------------------------------- */
  /* .text section (Program Code)                        */
  /* ---------------------------------------------------- */
  /* This section contains all executable code. */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* All .text sections from all object files */
    *(.text.*)         /* All .text.something sections (e.g., from C++ or specific functions) */

    *(.rodata)         /* Read-only data (string literals, const variables) */
    *(.rodata.*)

    *(.init)
    *(.init_array)/* C++ constructors */
    *(.init_array.*)

    *(.fini)           /* C++ destructors */

  } > FLASH      /* Resides in Flash */





  /* Symbol marking the end of the initialized code and read-only data */
  /* This is also the load address of the .data section */
  _etext = .;


  _sidata =  _etext ;
  /* ---------------------------------------------------- */
  /* .data section (Initialized Data)                    */
  /* ---------------------------------------------------- */
  /* This section contains initialized global and static variables.
   * It resides in RAM (run address), but its initial values are
   * loaded from Flash (load address) by the startup code.
   */
  .data : AT (_etext)/* The load address is immediately after _etext (in Flash) */
  {
    . = ALIGN(4); /* Align to 4-byte boundary in RAM */
    _sdata = .;   /* Symbol for start of .data in RAM (run address) */
    *(.data)      /* All .data sections from all object files */
    *(.data.*)
    . = ALIGN(4);
    _edata = .;   /* Symbol for end of .data in RAM (run address) */
  } > RAM         /* Resides in RAM (run address) */

  /* ---------------------------------------------------- */
  /* .bss section (Uninitialized Data)                   */
  /* ---------------------------------------------------- */
  /* This section contains uninitialized global and static variables.
   * It resides in RAM and is zeroed out by the startup code.
   * It does not have a load address in Flash.
   */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .; /* Symbol for start of .bss in RAM */
    *(.bss)    /* All .bss sections from all object files */
    *(.bss.*)
    *(COMMON)  /* Common uninitialized data (e.g., from old C style) */
    . = ALIGN(4);
    _ebss = .; /* Symbol for end of .bss in RAM */
  } > RAM      /* Resides in RAM */

  /* ---------------------------------------------------- */
  /* .stack section (Stack)                              */
  /* ---------------------------------------------------- */
  /* The stack typically starts at the very end of RAM and grows downwards.
   * Its size is usually defined by a symbol for the startup code.
   */

  .stack :
  {
    . = ALIGN(8); /* Stack often requires 8-byte alignment for performance (ABI compliance) */
    _estack = ORIGIN(RAM) + LENGTH(RAM); /* Symbol for the initial top of stack */
  } > RAM



  /* Symbol for the end of RAM, used for stack/heap boundary checks */
  _end = .; /* Marks the end of all allocated RAM sections (excluding stack) */



}
