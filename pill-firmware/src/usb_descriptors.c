/**
 * USB descriptors — vendor + UAC2 stereo (TinyUSB).
 */
#include "tusb.h"
#include "usb_descriptors.h"
#include "proficio_config.h"
#include <string.h>

#define USB_VID   0x16C0u
#define USB_PID   0x05DCu

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210,   /* 3.17: 2.10 so Windows reads the BOS (WinUSB) */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0101,   /* 3.17: was 0x0100; new value makes Windows look again */
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

enum {
    CONFIG_TOTAL_LEN = TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_AUDIO_PROFICIO_DESC_LEN
};

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    /* stridx 7 = "Multus Control" (was wrongly 6 = Capture — Zadig/Device Manager name) */
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 7, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),
    TUD_AUDIO_PROFICIO_DESCRIPTOR(4, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN, EPNUM_AUDIO_FB)
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

/* 3.17: WinUSB without Zadig. The BOS tells Windows 8.1+ that a Microsoft OS 2.0
 * descriptor set exists; the set puts WinUSB on interface 0 (Multus Control)
 * only. The audio interfaces keep Windows' audio driver. Linux ignores both.
 * A driver picked by hand (Zadig libusb-win32) still wins over WinUSB. */
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

uint8_t const desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, MS_OS_20_VENDOR_CODE)
};

TU_VERIFY_STATIC(sizeof(desc_bos) == BOS_TOTAL_LEN, "BOS length");

uint8_t const *tud_descriptor_bos_cb(void)
{
    return desc_bos;
}

/* Fixed for good: the radio's WinUSB device interface GUID. Never change it. */
uint8_t const desc_ms_os_20[MS_OS_20_DESC_LEN] = {
    /* Set header: length, type, Windows 8.1 (0x06030000), total length */
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    /* Configuration subset header: configuration index 0, subset length */
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    /* Function subset header: first interface 0 (vendor), subset length */
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    /* Compatible ID: WINUSB */
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,

    /* Registry property: DeviceInterfaceGUIDs, REG_MULTI_SZ (7), name 42 bytes */
    U16_TO_U8S_LE(0x0084), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0, 'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0,
    0, 0,
    /* data 80 bytes: the GUID, then two UTF-16 nulls (end of string, end of list) */
    U16_TO_U8S_LE(0x0050),
    '{', 0, '0', 0, '3', 0, '9', 0, '2', 0, 'A', 0, '8', 0, '7', 0, 'C', 0, '-', 0,
    'D', 0, 'D', 0, '9', 0, '5', 0, '-', 0, '4', 0, '3', 0, '3', 0, '2', 0, '-', 0,
    'B', 0, '3', 0, '7', 0, 'A', 0, '-', 0, '7', 0, '6', 0, 'E', 0, 'E', 0, '3', 0,
    '9', 0, 'A', 0, '7', 0, '1', 0, '8', 0, '2', 0, 'C', 0, '}', 0,
    0, 0, 0, 0
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "MS OS 2.0 set length");

static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "Multus",
    "Multus IQ",             /* iProduct — Device Manager / Zadig composite name (3.23: as the PSoC radios, Stew R11-16-0) */
    /* iSerial: fixed. It was the build date/time, so Windows saw every new
     * build as a new device (drivers again, new sound devices, new default).
     * The build is identified by FIRMWARE_VERSION_MINOR instead. */
    "MKII-PTT-F411",
    "Multus IQ Sound",       /* Audio Control — Windows Speakers/Mic parent name */
    "Multus IQ Transmit",    /* TX IQ (host → radio) */
    "Multus IQ Receive",     /* RX IQ (radio → host) */
    "Multus Control"         /* Vendor/control interface (stridx 7) */
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
