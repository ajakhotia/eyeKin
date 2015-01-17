cd boost_1_57_0
call .\bootstrap.bat
.\b2 toolset=msvc variant=debug,release address-model=32
rmdir /Q /S stage/x86
ren stage\lib x86
.\b2 toolset=msvc variant=debug,release address-model=64
rmdir /Q /S stage/x64
ren stage\lib x64
cd ..\..