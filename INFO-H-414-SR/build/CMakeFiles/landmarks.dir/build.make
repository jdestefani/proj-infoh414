# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/deste/workspace/INFO-H-414/INFO-H-414-SR/build

# Include any dependencies generated for this target.
include CMakeFiles/landmarks.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/landmarks.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/landmarks.dir/flags.make

CMakeFiles/landmarks.dir/landmarks.cpp.o: CMakeFiles/landmarks.dir/flags.make
CMakeFiles/landmarks.dir/landmarks.cpp.o: /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src/landmarks.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/deste/workspace/INFO-H-414/INFO-H-414-SR/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/landmarks.dir/landmarks.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/landmarks.dir/landmarks.cpp.o -c /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src/landmarks.cpp

CMakeFiles/landmarks.dir/landmarks.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/landmarks.dir/landmarks.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src/landmarks.cpp > CMakeFiles/landmarks.dir/landmarks.cpp.i

CMakeFiles/landmarks.dir/landmarks.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/landmarks.dir/landmarks.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src/landmarks.cpp -o CMakeFiles/landmarks.dir/landmarks.cpp.s

CMakeFiles/landmarks.dir/landmarks.cpp.o.requires:
.PHONY : CMakeFiles/landmarks.dir/landmarks.cpp.o.requires

CMakeFiles/landmarks.dir/landmarks.cpp.o.provides: CMakeFiles/landmarks.dir/landmarks.cpp.o.requires
	$(MAKE) -f CMakeFiles/landmarks.dir/build.make CMakeFiles/landmarks.dir/landmarks.cpp.o.provides.build
.PHONY : CMakeFiles/landmarks.dir/landmarks.cpp.o.provides

CMakeFiles/landmarks.dir/landmarks.cpp.o.provides.build: CMakeFiles/landmarks.dir/landmarks.cpp.o

# Object files for target landmarks
landmarks_OBJECTS = \
"CMakeFiles/landmarks.dir/landmarks.cpp.o"

# External object files for target landmarks
landmarks_EXTERNAL_OBJECTS =

liblandmarks.so: CMakeFiles/landmarks.dir/landmarks.cpp.o
liblandmarks.so: CMakeFiles/landmarks.dir/build.make
liblandmarks.so: /usr/lib/i386-linux-gnu/liblua5.1.so
liblandmarks.so: /usr/lib/i386-linux-gnu/libm.so
liblandmarks.so: CMakeFiles/landmarks.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library liblandmarks.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/landmarks.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/landmarks.dir/build: liblandmarks.so
.PHONY : CMakeFiles/landmarks.dir/build

CMakeFiles/landmarks.dir/requires: CMakeFiles/landmarks.dir/landmarks.cpp.o.requires
.PHONY : CMakeFiles/landmarks.dir/requires

CMakeFiles/landmarks.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/landmarks.dir/cmake_clean.cmake
.PHONY : CMakeFiles/landmarks.dir/clean

CMakeFiles/landmarks.dir/depend:
	cd /home/deste/workspace/INFO-H-414/INFO-H-414-SR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src /home/deste/workspace/INFO-H-414/INFO-H-414-SR/src /home/deste/workspace/INFO-H-414/INFO-H-414-SR/build /home/deste/workspace/INFO-H-414/INFO-H-414-SR/build /home/deste/workspace/INFO-H-414/INFO-H-414-SR/build/CMakeFiles/landmarks.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/landmarks.dir/depend

