if(CONFIG_KORZ_JOYSTICK)
  zephyr_library()
  zephyr_library_sources(src/joystick.c)
endif()
