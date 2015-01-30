.\..\protobuf-2.6.1\vsprojects\bin\protoc\Win32\Debug\protoc.exe -I=.\messaging\ --cpp_out=.\include\ .\messaging\point2D.proto
move /Y .\include\point2D.pb.cc .\src\

.\..\protobuf-2.6.1\vsprojects\bin\protoc\Win32\Debug\protoc.exe -I=.\messaging\ --cpp_out=.\include\ .\messaging\pose2D.proto
move /Y .\include\pose2D.pb.cc .\src\

.\..\protobuf-2.6.1\vsprojects\bin\protoc\Win32\Debug\protoc.exe -I=.\messaging\ --cpp_out=.\include\ .\messaging\entity.proto
move /Y .\include\entity.pb.cc .\src\