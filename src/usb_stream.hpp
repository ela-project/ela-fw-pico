#include "tusb.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"

class USBStream {
   public:
    USBStream(stdio_driver_t *usb_driver) : _usb_driver{*usb_driver} {
    }

    USBStream(const stdio_driver_t &usb_driver) : _usb_driver{usb_driver} {
    }

    void init() const {
        stdio_usb_init();
    }

    int receive_blocking() const {
        return getchar();
    }

    int receive_timeout(const uint32_t &timeout_us) const {
        return getchar_timeout_us(timeout_us);
    }

    int send(const uint8_t &byte) const {
        send(static_cast<const char>(byte));
        return 0;
    }

    int send(const char byte) const {
        send(&byte, 1);
        return 0;
    }

    int send(const uint8_t *buff, size_t count) const {
        send(reinterpret_cast<const char *>(buff), count);
        return 0;
    }

    int send(const char *buff, size_t count) const {
        _usb_driver.out_chars(buff, count);
        return 0;
    }

    bool connected() const {
        return stdio_usb_connected();
    }

    void flush() const {
        tud_cdc_write_flush();
    }

   private:
    const stdio_driver_t &_usb_driver;
};