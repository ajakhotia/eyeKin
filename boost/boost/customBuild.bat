dir
cd boost_1_57_0
call .\bootstrap.bat

rmdir /q /s stage\lib
rmdir /q /s stage\x86
rmdir /q /s stage\x64

.\b2 toolset=msvc variant=debug,release address-model=32
ren stage\lib x86

.\b2 toolset=msvc variant=debug,release address-model=64
ren stage\lib x64

dir

cd ..

dir