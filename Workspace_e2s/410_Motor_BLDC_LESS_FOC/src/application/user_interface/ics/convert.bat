set arg1=%1
arm-none-eabi-objdump.exe -W %arg1%.elf>%arg1%.txt
..\src\application\user_interface\ics\ElfMapConverter.exe %arg1%.txt
copy %arg1%.rmap ..\src\application\user_interface\ics\%arg1%_conv.map
