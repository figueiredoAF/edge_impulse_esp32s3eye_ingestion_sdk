if(IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp8266" OR IDF_TARGET STREQUAL "esp32s3")
  set(requires    bus)
  idf_component_register(SRC_DIRS "." 
                         INCLUDE_DIRS "."
                         REQUIRES ${requires})
endif()
