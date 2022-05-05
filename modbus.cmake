# set (MODBUS_DIR ${SRC_DIR}modbus/)
set (MODBUS_INCLUDE_DIRECTORIES
    ${MODBUS_DIR}
)

set (MODBUS_SOURCES
#    MASTER
#    ${MODBUS_DIR}devices/power_meter/power_meter.h
#    ${MODBUS_DIR}devices/power_meter/power_meter.cpp
#    ${MODBUS_DIR}devices/modbus_device.h
#    ${MODBUS_DIR}devices/modbus_device.cpp
#    ${MODBUS_DIR}modbus_master1.h
#    ${MODBUS_DIR}modbus_master1.cpp
#    ${MODBUS_DIR}modbus_master_base.h
#    ${MODBUS_DIR}modbus_master_base.cpp

#    SLAVE
    ${MODBUS_DIR}modbus_slave_base.h
    ${MODBUS_DIR}modbus_slave_base.cpp
    ${MODBUS_DIR}modbus_slave1.h
    ${MODBUS_DIR}modbus_slave1.cpp
    )
