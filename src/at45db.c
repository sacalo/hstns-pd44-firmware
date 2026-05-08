
#include <stddef.h>    
#include "xc.h"
#include "p33Fxxxx.h"
#include "define.h"

extern void delay_ms(uint16_t ms);
extern void uart2_transmit_frame(void);
extern s16 voutSetpoint;
extern int16_t cal_a_gain, cal_a_offset;
extern int16_t Iref;
extern uint16_t ioutAdcRaw;
extern uint16_t ocp_latch_threshold;
extern uint16_t ocp_hard_threshold;
extern uint16_t ocp_latch_delay;
extern uint16_t ocp_foldback_delay;
extern uint16_t ovp_threshold_normal;
extern uint16_t ovp_threshold_mode4;
extern uint16_t ovp_freq_ctrl_min;

/* reads can bypass the buffers */
#define OP_READ_CONTINUOUS	0xE8
#define OP_READ_PAGE		0xD2

/* group B requests can run even while status reports "busy" */
#define OP_READ_STATUS		0xD7	/* group B */

#define OP_DISABLE_PROTECT  0x3D

/* move data between host and buffer */
#define OP_READ_BUFFER1		0xD4	/* group B */
#define OP_WRITE_BUFFER1	0x84	/* group B */

/* erasing flash */
#define OP_ERASE_PAGE		0x81

/* move data between buffer and flash */
#define OP_TRANSFER_BUF1	0x53
#define OP_TRANSFER_BUF2	0x55
#define OP_MREAD_BUFFER1	0xD4
#define OP_MREAD_BUFFER2	0xD6
#define OP_MWERASE_BUFFER1	0x83
#define OP_MWERASE_BUFFER2	0x86
#define OP_MWRITE_BUFFER1	0x88	/* sector must be pre-erased */
#define OP_MWRITE_BUFFER2	0x89	/* sector must be pre-erased */

/* write to buffer, then write-erase to flash */
#define OP_PROGRAM_VIA_BUF1	0x82
#define OP_PROGRAM_VIA_BUF2	0x85


/* newer chips report JEDEC manufacturer and device IDs; chip
 * serial number and OTP bits; and per-sector writeprotect.
 */
#define OP_READ_ID		0x9F

uint16_t spi_flash_status;  /* bit7 = RDY/BUSY             */
uint8_t at45db_device_id[8];
uint16_t spi_flash_config_status;
uint16_t spi_flash_fw_status;
uint16_t spi_flash_fw_actual_crc;
uint16_t spi_flash_fw_expected_crc;
uint16_t spi_flash_fw_active_slot;
uint16_t spi_flash_fw_checked_page;
static uint8_t spi_flash_config_buf[256];
static uint8_t spi_flash_fw_buf[256];

#define FLASH_CONFIG_PAGE     6u
#define FLASH_CONFIG_MAGIC    0x5044u
#define FLASH_CONFIG_VERSION  1u
#define FLASH_CONFIG_DATA_WORDS 14u
#define FLASH_CONFIG_CRC_WORD   14u

enum {
    FLASH_CONFIG_OK       = 0x0001u,
    FLASH_CONFIG_BLANK    = 0x0002u,
    FLASH_CONFIG_BAD_CRC  = 0x0004u,
    FLASH_CONFIG_BAD_ID   = 0x0008u,
    FLASH_CONFIG_WRITE_OK = 0x0010u,
    FLASH_CONFIG_WRITE_ERR = 0x0020u
};

/* Firmware update image window. Slot 0 keeps the original 0x0800..0x9FFF
 * external Flash span; slot 1 mirrors the same span as the backup image. */
#define FLASH_FW_SLOT0_START_PAGE 0x0008u
#define FLASH_FW_SLOT1_START_PAGE 0x00A0u
#define FLASH_FW_PAGE_COUNT       0x0098u
#define FLASH_FW_LAST_DATA_BYTES  252u
#define FLASH_FW_META_PAGE0       0x03FEu
#define FLASH_FW_META_PAGE1       0x03FFu
#define FLASH_FW_META_MAGIC       0x4657u
#define FLASH_FW_META_VERSION     1u
#define FLASH_FW_META_DATA_WORDS  7u
#define FLASH_FW_META_CRC_WORD    7u

enum {
    FLASH_FW_OK        = 0x0001u,
    FLASH_FW_NO_IMAGE  = 0x0002u,
    FLASH_FW_BAD_CRC   = 0x0004u,
    FLASH_FW_BAD_META  = 0x0008u,
    FLASH_FW_WRITE_OK  = 0x0010u,
    FLASH_FW_WRITE_ERR = 0x0020u,
    FLASH_FW_SLOT0     = 0x0100u,
    FLASH_FW_SLOT1     = 0x0200u
};

uint16_t crc16(const uint8_t *buf, uint16_t len);
int spi_at45db_page_read(uint8_t *buf, uint16_t len,
                         uint16_t page, uint8_t byte_offset);
int spi_at45db_page_write_safe(const uint8_t *buf, uint16_t page);

static uint16_t crc16_update(uint16_t crc, const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static uint16_t flash_config_get16(uint16_t idx)
{
    uint16_t pos = idx * 2u;
    return (uint16_t)spi_flash_config_buf[pos] |
           ((uint16_t)spi_flash_config_buf[pos + 1u] << 8);
}

static void flash_config_put16(uint16_t idx, uint16_t value)
{
    uint16_t pos = idx * 2u;
    spi_flash_config_buf[pos] = (uint8_t)value;
    spi_flash_config_buf[pos + 1u] = (uint8_t)(value >> 8);
}

static uint16_t flash_config_crc(void)
{
    return crc16(spi_flash_config_buf, FLASH_CONFIG_DATA_WORDS * 2u);
}

static uint16_t flash_fw_slot_flag(uint16_t slot)
{
    return (slot == 0u) ? FLASH_FW_SLOT0 : FLASH_FW_SLOT1;
}

static uint16_t flash_fw_start_page(uint16_t slot)
{
    return (slot == 0u) ? FLASH_FW_SLOT0_START_PAGE : FLASH_FW_SLOT1_START_PAGE;
}

static uint16_t flash_fw_meta_page(uint16_t slot)
{
    return (slot == 0u) ? FLASH_FW_META_PAGE0 : FLASH_FW_META_PAGE1;
}

static uint16_t flash_fw_get16(uint16_t idx)
{
    uint16_t pos = idx * 2u;
    return (uint16_t)spi_flash_fw_buf[pos] |
           ((uint16_t)spi_flash_fw_buf[pos + 1u] << 8);
}

static void flash_fw_put16(uint16_t idx, uint16_t value)
{
    uint16_t pos = idx * 2u;
    spi_flash_fw_buf[pos] = (uint8_t)value;
    spi_flash_fw_buf[pos + 1u] = (uint8_t)(value >> 8);
}

static uint16_t flash_fw_meta_crc(void)
{
    return crc16(spi_flash_fw_buf, FLASH_FW_META_DATA_WORDS * 2u);
}

static void flash_config_clear_buf(void)
{
    for (uint16_t i = 0; i < sizeof(spi_flash_config_buf); i++)
        spi_flash_config_buf[i] = 0xFFu;
}

static void flash_fw_clear_buf(void)
{
    for (uint16_t i = 0; i < sizeof(spi_flash_fw_buf); i++)
        spi_flash_fw_buf[i] = 0xFFu;
}

static void flash_config_pack(void)
{
    flash_config_clear_buf();
    flash_config_put16(0, FLASH_CONFIG_MAGIC);
    flash_config_put16(1, FLASH_CONFIG_VERSION);
    flash_config_put16(2, (uint16_t)voutSetpoint);
    flash_config_put16(3, (uint16_t)cal_a_gain);
    flash_config_put16(4, (uint16_t)cal_a_offset);
    flash_config_put16(5, (uint16_t)Iref);
    flash_config_put16(6, ioutAdcRaw);
    flash_config_put16(7, ocp_latch_threshold);
    flash_config_put16(8, ocp_hard_threshold);
    flash_config_put16(9, ocp_latch_delay);
    flash_config_put16(10, ocp_foldback_delay);
    flash_config_put16(11, ovp_threshold_normal);
    flash_config_put16(12, ovp_threshold_mode4);
    flash_config_put16(13, ovp_freq_ctrl_min);
    flash_config_put16(FLASH_CONFIG_CRC_WORD, flash_config_crc());
}

static void flash_config_apply(void)
{
    voutSetpoint = (s16)flash_config_get16(2);
    cal_a_gain = (int16_t)flash_config_get16(3);
    cal_a_offset = (int16_t)flash_config_get16(4);
    Iref = (int16_t)flash_config_get16(5);
    ioutAdcRaw = flash_config_get16(6);
    ocp_latch_threshold = flash_config_get16(7);
    ocp_hard_threshold = flash_config_get16(8);
    ocp_latch_delay = flash_config_get16(9);
    ocp_foldback_delay = flash_config_get16(10);
    ovp_threshold_normal = flash_config_get16(11);
    ovp_threshold_mode4 = flash_config_get16(12);
    ovp_freq_ctrl_min = flash_config_get16(13);
}

static void flash_fw_pack_meta(uint16_t slot, uint16_t expected_crc)
{
    flash_fw_clear_buf();
    flash_fw_put16(0, FLASH_FW_META_MAGIC);
    flash_fw_put16(1, FLASH_FW_META_VERSION);
    flash_fw_put16(2, slot);
    flash_fw_put16(3, flash_fw_start_page(slot));
    flash_fw_put16(4, FLASH_FW_PAGE_COUNT);
    flash_fw_put16(5, expected_crc);
    flash_fw_put16(6, FLASH_FW_LAST_DATA_BYTES);
    flash_fw_put16(FLASH_FW_META_CRC_WORD, flash_fw_meta_crc());
}

static int flash_fw_read_meta(uint16_t slot, uint16_t *expected_crc)
{
    uint16_t slot_flag = flash_fw_slot_flag(slot);

    if (spi_at45db_page_read(spi_flash_fw_buf, sizeof(spi_flash_fw_buf),
                             flash_fw_meta_page(slot), 0) != 0) {
        spi_flash_fw_status = FLASH_FW_BAD_META | slot_flag;
        return -1;
    }

    if ((spi_flash_fw_buf[0] == 0xFFu) && (spi_flash_fw_buf[1] == 0xFFu)) {
        spi_flash_fw_status = FLASH_FW_NO_IMAGE | slot_flag;
        return 1;
    }

    if ((flash_fw_get16(0) != FLASH_FW_META_MAGIC) ||
        (flash_fw_get16(1) != FLASH_FW_META_VERSION) ||
        (flash_fw_get16(2) != slot) ||
        (flash_fw_get16(3) != flash_fw_start_page(slot)) ||
        (flash_fw_get16(4) != FLASH_FW_PAGE_COUNT) ||
        (flash_fw_get16(6) != FLASH_FW_LAST_DATA_BYTES) ||
        (flash_fw_get16(FLASH_FW_META_CRC_WORD) != flash_fw_meta_crc())) {
        spi_flash_fw_status = FLASH_FW_BAD_META | slot_flag;
        return -1;
    }

    *expected_crc = flash_fw_get16(5);
    return 0;
}

static int flash_fw_crc_slot(uint16_t slot, uint16_t *actual_crc)
{
    uint16_t crc = 0;
    uint16_t start_page = flash_fw_start_page(slot);

    /*
     * 0x425E in the original firmware verifies the staged update image from
     * external Flash address 0x0800 to 0x9FFF and excludes the final 4 bytes.
     * src keeps the same page span for slot 0 and mirrors it for slot 1.
     */
    for (uint16_t page_idx = 0; page_idx < FLASH_FW_PAGE_COUNT; page_idx++) {
        uint16_t page = start_page + page_idx;
        uint16_t len = (page_idx == (FLASH_FW_PAGE_COUNT - 1u)) ?
                       FLASH_FW_LAST_DATA_BYTES : 256u;

        if (spi_at45db_page_read(spi_flash_fw_buf, sizeof(spi_flash_fw_buf),
                                 page, 0) != 0)
            return -1;

        crc = crc16_update(crc, spi_flash_fw_buf, len);
        spi_flash_fw_checked_page = page;
        ClrWdt();
    }

    *actual_crc = crc;
    return 0;
}

static inline void spi_cs_assert(void)
{
    LATGbits.LATG9 = 0;    // CS low = active
}

static inline void spi_cs_deassert(void)
{
    LATGbits.LATG9 = 1;    // CS high = inactive
}

// SPI2 interface for AT45DB021E flash
// dsPIC33F SPI: write to SPI2BUF triggers 8 SCK clocks,
// simultaneously transmits and receives one byte.

static uint8_t spi_xfer(uint8_t data)
{
    if (SPI2STATbits.SPIROV)            // RX overflow occurred
        SPI2STATbits.SPIROV = 0;        // clear to resume

    while (SPI2STATbits.SPITBF);        // wait TX buffer empty
    SPI2BUF = data;                     // trigger 8 SCK clocks
    while (!SPI2STATbits.SPIRBF);       // wait RX complete
    return SPI2BUF;                     // read clears SPIRBF
}

static void spi_write(uint8_t data)
{
    (void)spi_xfer(data);               // discard received byte
}

static uint8_t spi_read(void)
{
    return spi_xfer(0xFF);              // send dummy, return received
}

static int spi_write_then_read(const void *txbuf, unsigned n_tx,
                                void *rxbuf, unsigned n_rx)
{
    const uint8_t *tx = (const uint8_t *)txbuf;
    uint8_t *rx = (uint8_t *)rxbuf;

    spi_cs_assert();

    for (unsigned i = 0; i < n_tx; i++)
        spi_write(tx[i]);

    for (unsigned i = 0; i < n_rx; i++)
        rx[i] = spi_read();

    spi_cs_deassert();

    return 0;
}

static void spi_at45db_read_status(uint16_t *status)
{
    uint8_t cmd = OP_READ_STATUS;
    uint8_t val;

    spi_write_then_read(&cmd, 1, &val, 1);

    *status = (uint16_t)val;
}

static void spi_at45db_wait_ready(void)
{
    uint16_t status;
    do {
        spi_at45db_read_status(&status);
    } while (!(status & 0x80));
}

void spi_at45db_read_id(void)
{
	u8 code = OP_READ_ID;

    // 1Fh 23h 00h 01h 00h
    // JEDEC code: 0001 1111 (1Fh for Adesto)
    // Family code: 001 (AT45Dxxx Family)
    // 2-Mbit DataFlash 
	spi_write_then_read(&code, 1, at45db_device_id, 5);
}

int spi_at45db_page_read(uint8_t *buf, uint16_t len,
                                 uint16_t page, uint8_t byte_offset)
{
    spi_at45db_wait_ready();
    
    uint8_t tx[8] = {
        0xD2,                           // opcode
        (page >> 8) & 0x03,             // A17-A16
        page & 0xFF,                    // A15-A8
        byte_offset,                    // A7-A0
        0x00, 0x00, 0x00, 0x00          // 4 dummy bytes
    };

    return spi_write_then_read(tx, 8, buf, len);
}

// CRC-16/XMODEM
uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    return crc16_update(0, buf, len);
}

// read flash page, send raw bytes via UART
void spi_flash_dump_raw(uint16_t start_page, uint16_t num_pages)
{
    uint8_t buf[256];

    for (uint16_t p = start_page; p < start_page + num_pages; p++) {
        spi_at45db_page_read(buf, 256, p, 0);

        // MCU: send sync header before each page
        while (U2STAbits.UTXBF); U2TXREG = 0xAA;  // sync byte 1
        while (U2STAbits.UTXBF); U2TXREG = 0x55;  // sync byte 2
        
        // then 256 data + 2 CRC
        for (uint16_t i = 0; i < 256; i++) {
            while (U2STAbits.UTXBF);
            U2TXREG = buf[i];
        }
        
        // send 2 bytes CRC (big-endian)
        uint16_t crc = crc16(buf, 256);
        while (U2STAbits.UTXBF);
        U2TXREG = (crc >> 8) & 0xFF;
        while (U2STAbits.UTXBF);
        U2TXREG = crc & 0xFF;
        
        LED_TOGGLE();
        ClrWdt();
    }
}

// Configure AT45DB021E page size (one-time nonvolatile setting)
// mode: 0 = binary 256 bytes, 1 = standard 264 bytes

static int __attribute__((unused)) spi_at45db_config_pagesize(uint8_t mode)
{
    uint8_t tx[4] = {
        0x3D, 0x2A, 0x80,
        (mode == 0) ? 0xA6 : 0xA7
    };

    spi_write_then_read(tx, 4, NULL, 0);

    spi_at45db_wait_ready();            // wait tEP (~20ms)

    return 0;
}

void spi_at45db_page_erase(uint16_t page)
{
    spi_at45db_wait_ready();

    uint8_t tx[4] = {
        OP_ERASE_PAGE,                       // page erase
        (page >> 8) & 0x03,        // A17-A16
        page & 0xFF,                // A15-A8
        0x00                        // don't care
    };

    spi_write_then_read(tx, 4, (void *)0, 0);
}

//static void spi_at45db_page_write(const uint8_t *buf, uint16_t page)
//{
//    spi_at45db_wait_ready();
//
//    // Step 1: write data into Buffer 1 (0x84 + 3 addr + 256 data)
//    uint8_t hdr1[4] = { 0x84, 0x00, 0x00, 0x00 };
//    spi_cs_assert();
//    for (int i = 0; i < 4; i++)
//        spi_write(hdr1[i]);
//    for (int i = 0; i < 256; i++)
//        spi_write(buf[i]);
//    spi_cs_deassert();
//
//    spi_at45db_wait_ready();
//
//    // Step 2: program Buffer 1 to page (0x88 + 3 addr)
//    uint8_t hdr2[4] = {
//        0x88,
//        (page >> 8) & 0x03,
//        page & 0xFF,
//        0x00
//    };
//    spi_write_then_read(hdr2, 4, (void *)0, 0);
//
//    // internal program ~35ms, call wait_ready before next operation
//}

static int spi_at45db_config_binary_pagesize(void)
{
    uint16_t status;

    // check current page size from status register bit 0
    // bit 0 = 1: already binary (256 bytes)
    // bit 0 = 0: standard (264 bytes), need to configure
    spi_at45db_read_status(&status);

    if (status & 0x01)
        return 0;                           // already 256-byte mode

    // wait ready
    spi_at45db_wait_ready();
    
    // configure binary page size (one-time nonvolatile)
    uint8_t tx[4] = { 0x3D, 0x2A, 0x80, 0xA6 };
    spi_write_then_read(tx, 4, (void *)0, 0);

    spi_at45db_wait_ready();                // wait tEP (~20ms)

    // verify
    spi_at45db_read_status(&status);

    return (status & 0x01) ? 0 : -1;       // 0=success, -1=failed
}

// 0x84 + 0x88: buffer then program, NO erase (caller must erase first)
static void spi_at45db_buf_write(const uint8_t *buf, uint16_t page)
{
    spi_at45db_wait_ready();

    // 0x84: write data into Buffer 1
    spi_cs_assert();
    spi_write(0x84);
    spi_write(0x00);
    spi_write(0x00);
    spi_write(0x00);
    for (uint16_t i = 0; i < 256; i++)
        spi_write(buf[i]);
    spi_cs_deassert();

    // 0x88: Buffer 1 to Page Program (no erase)
    uint8_t cmd[4] = {
        0x88,
        (page >> 8) & 0x03,
        page & 0xFF,
        0x00
    };
    spi_write_then_read(cmd, 4, (void *)0, 0);
}


// 0x84 + 0x83: buffer then program with erase
void spi_at45db_buf_program(const uint8_t *buf, uint16_t page)
{
    spi_at45db_wait_ready();

    spi_cs_assert();

    // 0x84: Buffer 1 Write + data
    spi_write(0x84);
    spi_write(0x00);
    spi_write(0x00);
    spi_write(0x00);

    for (uint16_t i = 0; i < 256; i++)
        spi_write(buf[i]);

    spi_cs_deassert();

    // 0x83: Buffer 1 to Page Program with built-in erase
    uint8_t cmd[4] = {
        0x83,
        (page >> 8) & 0x03,
        page & 0xFF,
        0x00
    };
    spi_write_then_read(cmd, 4, (void *)0, 0);

    // internal erase + program ~20ms
    // call wait_ready before next operation
}

int spi_at45db_page_write_safe(const uint8_t *buf, uint16_t page)
{
    uint8_t backup[256];
    uint8_t verify[256];

    // step 1: backup
    spi_at45db_page_read(backup, 256, page, 0);

    // step 2: erase
    spi_at45db_page_erase(page);

    // step 3: write
    spi_at45db_buf_write(buf, page);

    // step 4: verify
    spi_at45db_page_read(verify, 256, page, 0);

    for (uint16_t i = 0; i < 256; i++) {
        if (verify[i] != buf[i]) {
            // restore backup
            spi_at45db_page_erase(page);
            spi_at45db_buf_write(backup, page);
            return -1;
        }
    }

    return 0;
}


int spi_at45db_init(void)
{     
    // ensure binary page size (256 bytes)
    return spi_at45db_config_binary_pagesize();
}


// 0x82: auto erase + program, one step
int spi_at45db_page_program(const uint8_t *buf, uint16_t page)
{
    spi_at45db_wait_ready();

    // 0x82: stream data ? buffer ? auto erase + program
    spi_cs_assert();

    spi_write(0x82);                    // opcode
    spi_write((page >> 8) & 0x03);      // A17-A16
    spi_write(page & 0xFF);             // A15-A8
    spi_write(0x00);                    // byte offset = 0

    for (uint16_t i = 0; i < 256; i++)
        spi_write(buf[i]);

    spi_cs_deassert();                  // triggers erase + program (~20ms)
    
    spi_at45db_wait_ready();

    uint16_t status;
    spi_at45db_read_status(&status);
    if (status & 0x20)
        return -1;

    return 0;
}

int spi_flash_load_config(void)
{
    spi_flash_config_status = 0;

    if (spi_at45db_page_read(spi_flash_config_buf, sizeof(spi_flash_config_buf),
                             FLASH_CONFIG_PAGE, 0) != 0) {
        spi_flash_config_status = FLASH_CONFIG_BAD_ID;
        return -1;
    }

    if ((spi_flash_config_buf[0] == 0xFFu) && (spi_flash_config_buf[1] == 0xFFu)) {
        spi_flash_config_status = FLASH_CONFIG_BLANK;
        return 1;
    }

    if ((flash_config_get16(0) != FLASH_CONFIG_MAGIC) ||
        (flash_config_get16(1) != FLASH_CONFIG_VERSION)) {
        spi_flash_config_status = FLASH_CONFIG_BAD_ID;
        return -1;
    }

    if (flash_config_get16(FLASH_CONFIG_CRC_WORD) != flash_config_crc()) {
        spi_flash_config_status = FLASH_CONFIG_BAD_CRC;
        return -1;
    }

    flash_config_apply();
    spi_flash_config_status = FLASH_CONFIG_OK;
    return 0;
}

int spi_flash_save_config(void)
{
    flash_config_pack();

    if (spi_at45db_page_write_safe(spi_flash_config_buf, FLASH_CONFIG_PAGE) != 0) {
        spi_flash_config_status = FLASH_CONFIG_WRITE_ERR;
        return -1;
    }

    spi_flash_config_status = FLASH_CONFIG_WRITE_OK;
    return 0;
}

int spi_flash_verify_firmware_slot(uint16_t slot)
{
    uint16_t expected_crc = 0;
    uint16_t actual_crc = 0;
    uint16_t slot_flag;
    int meta_status;

    slot &= 1u;
    slot_flag = flash_fw_slot_flag(slot);
    spi_flash_fw_active_slot = slot;
    spi_flash_fw_actual_crc = 0;
    spi_flash_fw_expected_crc = 0;
    spi_flash_fw_checked_page = flash_fw_start_page(slot);

    meta_status = flash_fw_read_meta(slot, &expected_crc);
    if (meta_status != 0)
        return meta_status;

    spi_flash_fw_expected_crc = expected_crc;

    if (flash_fw_crc_slot(slot, &actual_crc) != 0) {
        spi_flash_fw_status = FLASH_FW_BAD_META | slot_flag;
        return -1;
    }

    spi_flash_fw_actual_crc = actual_crc;
    if (actual_crc == expected_crc) {
        spi_flash_fw_status = FLASH_FW_OK | slot_flag;
        return 0;
    }

    spi_flash_fw_status = FLASH_FW_BAD_CRC | slot_flag;
    return -1;
}

int spi_flash_verify_firmware_boot(void)
{
    int slot0_status;
    uint16_t slot0_result;
    uint16_t slot0_actual;
    uint16_t slot0_expected;

    slot0_status = spi_flash_verify_firmware_slot(0);
    if (slot0_status == 0)
        return 0;

    slot0_result = spi_flash_fw_status;
    slot0_actual = spi_flash_fw_actual_crc;
    slot0_expected = spi_flash_fw_expected_crc;

    if (spi_flash_verify_firmware_slot(1) == 0)
        return 0;

    if (slot0_result & (FLASH_FW_BAD_META | FLASH_FW_BAD_CRC)) {
        spi_flash_fw_status = slot0_result;
        spi_flash_fw_active_slot = 0;
        spi_flash_fw_actual_crc = slot0_actual;
        spi_flash_fw_expected_crc = slot0_expected;
    }

    return -1;
}

int spi_flash_save_firmware_metadata(uint16_t slot, uint16_t expected_crc)
{
    slot &= 1u;
    flash_fw_pack_meta(slot, expected_crc);

    if (spi_at45db_page_write_safe(spi_flash_fw_buf, flash_fw_meta_page(slot)) != 0) {
        spi_flash_fw_status = FLASH_FW_WRITE_ERR | flash_fw_slot_flag(slot);
        return -1;
    }

    spi_flash_fw_active_slot = slot;
    spi_flash_fw_expected_crc = expected_crc;
    spi_flash_fw_status = FLASH_FW_WRITE_OK | flash_fw_slot_flag(slot);
    return 0;
}

int spi_flash_save_current_firmware_crc(uint16_t slot)
{
    uint16_t actual_crc;

    slot &= 1u;
    if (flash_fw_crc_slot(slot, &actual_crc) != 0) {
        spi_flash_fw_status = FLASH_FW_BAD_META | flash_fw_slot_flag(slot);
        return -1;
    }

    spi_flash_fw_actual_crc = actual_crc;
    return spi_flash_save_firmware_metadata(slot, actual_crc);
}

uint8_t spi_rx_buf[256];

void spi_flash_test() 
{
  delay_ms(100);
  LED_OFF();
  delay_ms(100);
  LED_ON();
  spi_flash_dump_raw(0, 1024);    // all 1024 pages

  // spi_at45db_page_read(spi_rx_buf, 256, 7, 0);
  // spi_at45db_buf_program(spi_rx_buf, 18);
  //spi_at45db_page_program(spi_rx_buf, 18);
  // spi_at45db_page_write_safe(spi_rx_buf, 18);
  // spi_at45db_page_erase(18);
  LED_OFF();
  delay_ms(100);

  //spi_at45db_init();
  while(1) {
    LED_TOGGLE();
    delay_ms(10);

    ClrWdt();
  }
  
    spi_at45db_read_id();
    spi_at45db_page_read(spi_rx_buf, 0x20, 7, 0);
    uart2_transmit_frame();
  
  
}