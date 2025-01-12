# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack"
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/1"
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack"
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack/tmp"
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack/src/Serial_Port_Pack+SerialPortPack-stamp"
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack/src"
  "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack/src/Serial_Port_Pack+SerialPortPack-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack/src/Serial_Port_Pack+SerialPortPack-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/tung/UserFolder/XiaomiCloud/Work/Source/ST-Cube-MX/Serial_Pack/MDK-ARM/tmp/Serial_Port_Pack+SerialPortPack/src/Serial_Port_Pack+SerialPortPack-stamp${cfgdir}") # cfgdir has leading slash
endif()
