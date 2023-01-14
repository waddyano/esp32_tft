// IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.

#include <Adafruit_GFX.h> // Core graphics library
#include <fcntl.h>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "wifi.h"

#include <JPEGDEC.h>
#include <HardwareSerial.h>

// #include <Adafruit_TFTLCD.h> // Hardware-specific library

static const char *TAG = "showbmp";

#define MOUNT_POINT "/sdcard"

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

static bool mount_sdcard()
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    //host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    spi_bus_config_t bus_cfg{};
    bus_cfg.mosi_io_num = PIN_NUM_MOSI;
    bus_cfg.miso_io_num = PIN_NUM_MISO;
    bus_cfg.sclk_io_num = PIN_NUM_CLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return false;
    }

    ESP_LOGI(TAG, "SD filesystem mounted");

    return true;
}

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3    // Chip Select goes to Analog 3
#define LCD_CD A2    // Command/Data goes to Analog 2
#define LCD_WR A1    // LCD Write goes to Analog 1
#define LCD_RD A0    // LCD Read goes to Analog 0
#define PIN_SD_CS 10 // Adafruit SD shields and modules: pin 10

// Assign human-readable names to some common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

//PNG pngdec;
JPEGDEC jpegdec;

// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;

#define MAX_BMP 10      // bmp file num
#define FILENAME_LEN 20 // max file name length

const int __Gnbmp_height = 320; // bmp hight
const int __Gnbmp_width = 240;  // bmp width

unsigned char __Gnbmp_image_offset = 0; // offset

/*********************************************/
// This procedure reads a bitmap and draws it to the screen
// its sped up by reading many pixels worth of data at a time
// instead of just one pixel at a time. increading the buffer takes
// more RAM but makes the drawing a little faster. 20 pixels' worth
// is probably a good place

#define BUFFPIXEL 240 // must be a divisor of 240

struct Buffer
{
    static const int NCHUNKS = 16;
    static const int CHUNK_LINES = 320 / NCHUNKS;
    static const int CHUNK = BUFFPIXEL * CHUNK_LINES;

    ~Buffer()
    {
        Free();
    }

    bool Allocate()
    {
        for (int i = 0; i < NCHUNKS; ++i)
        {
            __color[i] = (uint16_t *)malloc(CHUNK * sizeof(uint16_t));
            if (__color[i] == nullptr)
            {
                printf("out of mem for __color[%d of %d] %d\n", i, NCHUNKS, CHUNK * sizeof(uint16_t));
                return false;
            }
        }

        return true;
    }

    void Free()
    {
        for (int i = 0; i < NCHUNKS; ++i)
        {
            free(__color[i]);
            __color[i] = nullptr;
        }
    }

    uint16_t *__color[NCHUNKS] {};
};

void bmpdraw(int fd, int x, int y)
{
    unsigned long start = millis();

    lseek(fd, __Gnbmp_image_offset, SEEK_SET);

    printf("free %zd\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    printf("largest free %zd\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    const int num_lines = 32;
    uint8_t *sdbuffer = (uint8_t *)malloc(BUFFPIXEL * 3 * num_lines); // 3 * pixels to buffer
    if (sdbuffer == nullptr)
    {
        printf("out of mem for sdbuf\n");
    }

    Buffer b;
    if (!b.Allocate())
    {
        free(sdbuffer);
        return;
    }

    printf("free %zd\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    printf("largest free %zd\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    uint32_t px = 0;

    int ichunk = 0;
    for (int i = 0; i < __Gnbmp_height;)
    {
        int n = num_lines;
        if (i + num_lines > __Gnbmp_height)
        {
            n = __Gnbmp_height - i;
        }

        read(fd, sdbuffer, BUFFPIXEL * 3 * n);

        uint16_t buffidx = 0;

        for (int k = 0; k < BUFFPIXEL * n; k++)
        {
            b.__color[ichunk][px] = sdbuffer[buffidx + 2] >> 3;                              // red
            b.__color[ichunk][px] = b.__color[ichunk][px] << 6 | (sdbuffer[buffidx + 1] >> 2); // green
            b.__color[ichunk][px] = b.__color[ichunk][px] << 5 | (sdbuffer[buffidx + 0] >> 3); // blue

            buffidx += 3;
            px++;

            if (px == Buffer::CHUNK)
            {
                px = 0;
                ++ichunk;
            }
        }

        i += n;
    }

    unsigned long t = millis();
    printf("%lu ms to fill\n", t - start);
    start = t;

    int h = 0;
    for (int i = 0; i < Buffer::NCHUNKS; ++i)
    {
        tft.setAddrWindow(0, h, 240, h + Buffer::CHUNK_LINES);
        tft.pushColors(b.__color[i], Buffer::CHUNK, true);
        h += Buffer::CHUNK_LINES;
    }

    t = millis();
    printf("%lu ms to push\n", t - start);

    free(sdbuffer);
    b.Free();
}

/*********************************************/
// These read data from the SD card file and convert them to big endian
// (the data is stored in little endian format!)

// LITTLE ENDIAN!
uint16_t read16(int fd)
{
    uint8_t b[2];
    read(fd, &b, 2);
    return (b[1] << 8) | b[0];
}

// LITTLE ENDIAN!
uint32_t read32(int fd)
{
    uint8_t b[4];
    read(fd, &b, 4);
    return (b[3] << 24) | (b[2] << 16) | (b[1] << 8) | b[0];
}

bool bmpReadHeader(int fd)
{
    // read header
    uint32_t tmp;
    uint8_t bmpDepth;

    if (read16(fd) != 0x4D42)
    {
        // magic bytes missing
        return false;
    }

    // read file size
    tmp = read32(fd);

    // read and ignore creator bytes
    read32(fd);

    __Gnbmp_image_offset = read32(fd);

    // read DIB header
    tmp = read32(fd);

    int bmp_width = read32(fd);
    int bmp_height = read32(fd);

    if (bmp_width != __Gnbmp_width || bmp_height != __Gnbmp_height)
    { // if image is not 320x240, return false
        return false;
    }

    if (read16(fd) != 1)
        return false;

    bmpDepth = read16(fd);

    if (read32(fd) != 0)
    {
        // compression not supported!
        return false;
    }

    return true;
}

static bool has_sdcard = false;
static DIR *dir = nullptr;

void setup(void)
{
    // Serial.begin(9600);
    // vTaskDelay(pdMS_TO_TICKS(5000));
    esp_log_level_set("gpio", ESP_LOG_NONE);
    printf("TFT pictures\n");
    esp_err_t ret;
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = false,
        .dont_mount = false};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    printf("about to register\n");
    ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info("littlefs", &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    printf("littlefs %zd %zd\n", total, used);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
    printf("Using Adafruit 2.4\" TFT Arduino Shield Pinout\n");
#else
    printf("Using Adafruit 2.4\" TFT Breakout Board Pinout\n");
#endif

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //wifi_init_sta("esp_tft", true);

    printf("TFT size is %d x %d\n", tft.width(), tft.height());

    tft.reset();

    uint16_t identifier = tft.readID();


    if (identifier == 0x9325)
    {
        Serial.println(F("Found ILI9325 LCD driver"));
    }
    else if (identifier == 0x9328)
    {
        Serial.println(F("Found ILI9328 LCD driver"));
    }
    else if (identifier == 0x4535)
    {
        Serial.println(F("Found LGDP4535 LCD driver"));
    }
    else if (identifier == 0x7575)
    {
        Serial.println(F("Found HX8347G LCD driver"));
    }
    else if (identifier == 0x9341)
    {
        Serial.println(F("Found ILI9341 LCD driver"));
    }
    else if (identifier == 0x8357)
    {
        Serial.println(F("Found HX8357D LCD driver"));
    }
    else if (identifier == 0x0101)
    {
        identifier = 0x9341;
        Serial.println(F("Found 0x9341 LCD driver"));
    }
    else
    {
        Serial.print(F("Unknown LCD driver chip: "));
        Serial.println(identifier, HEX);
        Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
        Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
        Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
        Serial.println(F("If using the breakout board, it should NOT be #defined!"));
        Serial.println(F("Also if using the breakout, double-check that all wiring"));
        Serial.println(F("matches the tutorial."));
        identifier = 0x9341;
    }

    tft.begin(identifier);
    tft.fillScreen(BLUE);

    has_sdcard = mount_sdcard();
    dir = opendir("/sdcard");
}

static int rot = 0;

static void display(const char *filename)
{
    unsigned long start = millis();
    int fd = open(filename, O_RDONLY);
    if (fd < 0)
    {
        printf("didnt find image %s\n", filename);
        tft.setTextColor(WHITE);
        tft.setTextSize(1);
        tft.println("didnt find BMPimage");
        return;
    }

    if (!bmpReadHeader(fd))
    {
        close(fd);
        printf("bad bmp %s\n", filename);
        tft.setTextColor(WHITE);
        tft.setTextSize(1);
        tft.println("bad bmp");
        return;
    }

    bmpdraw(fd, 0, 0);
    close(fd);
    printf("%s took %lu\n", filename, millis() - start);
}

static void displayPNG(const char *filename)
{
    //pngdec.decode(nullptr, 0);
}

static int jpgfd = -1;

static void * jpgOpen(const char *filename, int32_t *size)
{
    jpgfd = open(filename, O_RDONLY);
    if (jpgfd < 0)
    {
        return 0;
    }

    struct stat st;
    fstat(jpgfd, &st);
    *size = st.st_size;
    return &jpgfd;
}

static void jpgClose(void *handle) 
{
    if (jpgfd >= 0)
    {
        close(jpgfd);
        jpgfd = -1;
    }
}

static int32_t jpgRead(JPEGFILE *handle, uint8_t *buffer, int32_t length)
{
    if (jpgfd >= 0)
    {
        return read(jpgfd, buffer, length);
    }
    return 0;
}

static int32_t jpgSeek(JPEGFILE *handle, int32_t position)
{
    if (jpgfd >= 0)
    {
        return lseek(jpgfd, position, SEEK_SET);
    }
    return 0;
}

static int jpgDraw(JPEGDRAW *pDraw) 
{
    //printf("jpeg draw: x,y=%d,%d, cx,cy = %d,%d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
    tft.setAddrWindow(pDraw->x, pDraw->y, pDraw->x + pDraw->iWidth - 1, pDraw->y + pDraw->iHeight - 1);
    tft.pushColors(pDraw->pPixels, pDraw->iWidth * pDraw->iHeight, true);
    //tft.writeRect(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    return 1;
}

static void displayJPG(const char *filename)
{
    tft.setRotation(3);
    unsigned long start = millis();
    jpegdec.setPixelType(RGB565_BIG_ENDIAN);
    jpegdec.open(filename, jpgOpen, jpgClose, jpgRead, jpgSeek, jpgDraw);
    printf("size %d x %d\n", jpegdec.getWidth(), jpegdec.getHeight());
    jpegdec.decode(0, 0, 0);
    jpegdec.close();
    printf("%s took %lu\n", filename, millis() - start);
    tft.setRotation(0);
}

void loop(void)
{
    if (!has_sdcard)
    {
        tft.setRotation(rot);
        rot ^= 2;
    }

    if (dir != nullptr)
    {
        for (;;)
        {
            dirent *entry = readdir(dir);
            if (entry == nullptr)
            {
                closedir(dir);
                dir = opendir("/sdcard");
                entry = readdir(dir);
            }
            char buf[128];
            strcpy(buf, MOUNT_POINT);
            strcat(buf, "/");
            strcat(buf, entry->d_name);
            if (strstr(entry->d_name, ".bmp") != nullptr)
            {
                display(buf);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else if (strstr(entry->d_name, ".png") != nullptr)
            {
                displayPNG(buf);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else if (strstr(entry->d_name, ".jpg") != nullptr)
            {
                displayJPG(buf);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
    else
    {
        display("/littlefs/bmp.bmp");
    }
}
