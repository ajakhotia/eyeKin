.\..\protobuf-2.6.1\vsprojects\bin\protoc\x64\Debug\protoc.exe -I=.\messaging\ --cpp_out=.\include\ .\messaging\entity.proto
move /Y .\include\entity.pb.cc .\src\