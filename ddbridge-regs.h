// $Id: DD-DVBBridgeV1.h 246 2010-07-14 16:36:39Z manfred $

// Register Definitions


// --------------------------------------------------------------------------
// SPI Controller

#define SPI_CONTROL     0x10
#define SPI_DATA        0x14

// --------------------------------------------------------------------------

// Interrupt controller
// How many MSI's are available depends on HW (Min 2 max 8)
// How many are usable also depends on Host platform

#define INTERRUPT_BASE   (0x40)

#define INTERRUPT_ENABLE (INTERRUPT_BASE + 0x00)
#define MSI0_ENABLE      (INTERRUPT_BASE + 0x00)
#define MSI1_ENABLE      (INTERRUPT_BASE + 0x04)
#define MSI2_ENABLE      (INTERRUPT_BASE + 0x08)
#define MSI3_ENABLE      (INTERRUPT_BASE + 0x0C)
#define MSI4_ENABLE      (INTERRUPT_BASE + 0x10)
#define MSI5_ENABLE      (INTERRUPT_BASE + 0x14)
#define MSI6_ENABLE      (INTERRUPT_BASE + 0x18)
#define MSI7_ENABLE      (INTERRUPT_BASE + 0x1C)

#define INTERRUPT_STATUS (INTERRUPT_BASE + 0x20)
#define INTERRUPT_ACK    (INTERRUPT_BASE + 0x20)

#define INTMASK_I2C0     (0x00000001)
#define INTMASK_I2C1     (0x00000002)
#define INTMASK_I2C2     (0x00000004)
#define INTMASK_I2C3     (0x00000008)

// --------------------------------------------------------------------------
// I2C Master Controller

#define I2C_BASE        (0x80)  // Byte offset

#define I2C_COMMAND     (0x00)
#define I2C_TIMING      (0x04)
#define I2C_TASKLENGTH  (0x08)     // High read, low write
#define I2C_TASKADDRESS (0x0C)     // High read, low write

#define I2C_MONITOR     (0x1C)

#define I2C_BASE_0      (I2C_BASE + 0x00)
#define I2C_BASE_1      (I2C_BASE + 0x20)
#define I2C_BASE_2      (I2C_BASE + 0x40)
#define I2C_BASE_3      (I2C_BASE + 0x60)

#define I2C_TASKMEM_BASE    (0x1000)    // Byte offset
#define I2C_TASKMEM_SIZE    (0x1000)

#define I2C_SPEED_400   (0x04030404)
#define I2C_SPEED_200   (0x09080909)
#define I2C_SPEED_154   (0x0C0B0C0C)
#define I2C_SPEED_100   (0x13121312)
#define I2C_SPEED_77    (0x19181919)
#define I2C_SPEED_50    (0x27262727)


// --------------------------------------------------------------------------
// DMA  Controller

#define DMA_BASE_WRITE        (0x100)
#define DMA_BASE_READ         (0x140)

#define DMA_CONTROL     (0x00)                  // 64
#define DMA_ERROR       (0x04)                  // 65 ( only read instance )

#define DMA_DIAG_CONTROL                (0x1C)  // 71
#define DMA_DIAG_PACKETCOUNTER_LOW      (0x20)  // 72
#define DMA_DIAG_PACKETCOUNTER_HIGH     (0x24)  // 73  
#define DMA_DIAG_TIMECOUNTER_LOW        (0x28)  // 74  
#define DMA_DIAG_TIMECOUNTER_HIGH       (0x2C)  // 75  
#define DMA_DIAG_RECHECKCOUNTER         (0x30)  // 76  ( Split completions on read )
#define DMA_DIAG_WAITTIMEOUTINIT        (0x34)  // 77  
#define DMA_DIAG_WAITOVERFLOWCOUNTER    (0x38)  // 78 
#define DMA_DIAG_WAITCOUNTER            (0x3C)  // 79  

// --------------------------------------------------------------------------
// DMA  Buffer

#define TS_INPUT_BASE       (0x200)
#define TS_INPUT_CONTROL(i)         (TS_INPUT_BASE + (i) * 16 + 0x00) 

#define TS_OUTPUT_BASE       (0x280)
#define TS_OUTPUT_CONTROL(i)         (TS_OUTPUT_BASE + (i) * 16 + 0x00) 

#define DMA_BUFFER_BASE     (0x300)

#define DMA_BUFFER_CONTROL(i)       (DMA_BUFFER_BASE + (i) * 16 + 0x00) 
#define DMA_BUFFER_ACK(i)           (DMA_BUFFER_BASE + (i) * 16 + 0x04)
#define DMA_BUFFER_CURRENT(i)       (DMA_BUFFER_BASE + (i) * 16 + 0x08)
#define DMA_BUFFER_SIZE(i)          (DMA_BUFFER_BASE + (i) * 16 + 0x0c)

#define DMA_BASE_ADDRESS_TABLE  (0x2000)
#define DMA_BASE_ADDRESS_TABLE_ENTRIES (512)

