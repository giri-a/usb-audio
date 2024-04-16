idf_component_register(
    SRCS 2mic-1spk.c src/blink_led.c src/usb_descriptors.c
    INCLUDE_DIRS "include")

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
cmake_policy(SET CMP0079 NEW)
#target_link_libraries(${tusb_lib} )
target_link_libraries(${tusb_lib} PRIVATE ${COMPONENT_LIB})
