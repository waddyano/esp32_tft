idf_component_register(SRCS main.cpp app.cpp 
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES MCUFRIEND_kbv arduino Adafruit-GFX-Library Adafruit_TouchScreen littlefs fatfs JPEGDEC)
