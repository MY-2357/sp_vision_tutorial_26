# cmake_minimum_required(VERSION 3.16)

# add_library(io STATIC 
#     hikrobot/hikrobot.cpp    
#     camera.cpp
# )
# target_include_directories(io PUBLIC hikrobot/include)
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#   target_link_directories(io PUBLIC hikrobot/lib/amd64)
# elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#   target_link_directories(io PUBLIC hikrobot/lib/arm64)
# else()
#   message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
# endif()
# target_link_libraries(io MvCameraControl MVSDK usb-1.0)

# 备注：上面的代码编译时CMake总是报错， 原因是缺少mindvision下的libMVSDK.so。我经过仔细排查后，怀疑是我们赛队那里的问题，更新仓库后遗漏了那个文件夹。
# 不过我这里有我们最开始更新前的代码仓库，在这里面我找到了ibMVSDK.so。于是，我复制了一份到现在的仓库中，并修改了CMakeLists.txt文件。

cmake_minimum_required(VERSION 3.16)

add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    camera.cpp
)
target_include_directories(io PUBLIC hikrobot/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC
    hikrobot/lib/amd64
    mindvision/lib/amd64
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC
    hikrobot/lib/arm64
    mindvision/lib/arm64
  )
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()


target_link_libraries(io MvCameraControl MVSDK usb-1.0)
