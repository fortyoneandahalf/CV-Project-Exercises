# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build

# Include any dependencies generated for this target.
include CMakeFiles/cuboidExe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cuboidExe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cuboidExe.dir/flags.make

CMakeFiles/cuboidExe.dir/main.cpp.o: CMakeFiles/cuboidExe.dir/flags.make
CMakeFiles/cuboidExe.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cuboidExe.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cuboidExe.dir/main.cpp.o -c /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/main.cpp

CMakeFiles/cuboidExe.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cuboidExe.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/main.cpp > CMakeFiles/cuboidExe.dir/main.cpp.i

CMakeFiles/cuboidExe.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cuboidExe.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/main.cpp -o CMakeFiles/cuboidExe.dir/main.cpp.s

CMakeFiles/cuboidExe.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/cuboidExe.dir/main.cpp.o.requires

CMakeFiles/cuboidExe.dir/main.cpp.o.provides: CMakeFiles/cuboidExe.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/cuboidExe.dir/build.make CMakeFiles/cuboidExe.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/cuboidExe.dir/main.cpp.o.provides

CMakeFiles/cuboidExe.dir/main.cpp.o.provides.build: CMakeFiles/cuboidExe.dir/main.cpp.o


# Object files for target cuboidExe
cuboidExe_OBJECTS = \
"CMakeFiles/cuboidExe.dir/main.cpp.o"

# External object files for target cuboidExe
cuboidExe_EXTERNAL_OBJECTS =

cuboidExe: CMakeFiles/cuboidExe.dir/main.cpp.o
cuboidExe: CMakeFiles/cuboidExe.dir/build.make
cuboidExe: libcuboid.a
cuboidExe: CMakeFiles/cuboidExe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cuboidExe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cuboidExe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cuboidExe.dir/build: cuboidExe

.PHONY : CMakeFiles/cuboidExe.dir/build

CMakeFiles/cuboidExe.dir/requires: CMakeFiles/cuboidExe.dir/main.cpp.o.requires

.PHONY : CMakeFiles/cuboidExe.dir/requires

CMakeFiles/cuboidExe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cuboidExe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cuboidExe.dir/clean

CMakeFiles/cuboidExe.dir/depend:
	cd /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build /informatik2/students/home/5bradfie/CVProj/Exercises/CMakeTest/build/CMakeFiles/cuboidExe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cuboidExe.dir/depend

