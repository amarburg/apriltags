from conans import ConanFile, CMake

class ApriltagsConan(ConanFile):
  name = "apriltags"
  version = "master"
  settings = "os", "compiler", "build_type", "arch"
  generators = "cmake"

  options = {"opencv_dir": "ANY",
             "use_openmp": [True,False],
             "shared": [True,False]}
  default_options = "opencv_dir=''", "shared=True", "use_openmp=False"
  exports = '*'

  def config(self):
    if self.scope.dev and self.scope.build_tests:
      self.requires( "gtest/1.8.0@lasote/stable" )
      self.options["gtest"].shared = False

  def imports(self):
    self.copy("*.dll", dst="bin", src="bin")     # From bin to bin
    self.copy("*.dylib*", dst="bin", src="lib")  # From lib to bin

  def build(self):
    cmake = CMake(self.settings)

    cmake_opts = "-DUSE_CONAN:BOOL=TRUE "
    cmake_opts += "-DUSE_OPENMP:BOOL=%s " % (self.options.use_openmp)

    cmake_opts += "-DOpenCV_DIR:PATH=%s " % (self.options.opencv_dir) if self.options.opencv_dir else ""

    if not self.scope.dev:
        cmake_opts += "-DBUILD_PERF_TESTS:BOOL=FALSE -DBUILD_UNIT_TESTS:BOOL=FALSE "

    self.run('cmake "%s" %s %s' % (self.conanfile_directory,
                                    cmake.command_line, cmake_opts))
    self.run('cmake --build . %s' % (cmake.build_config))


  def package(self):
    self.copy("*.h", dst="")
    if self.options.shared:
        if self.settings.os == "Macos":
            self.copy(pattern="*.dylib", dst="lib", keep_path=False)
        else:
            self.copy(pattern="*.so*", dst="lib", keep_path=False)
    else:
        self.copy(pattern="*.a", dst="lib", src="lib", keep_path=False)

  def package_info(self):
      self.cpp_info.libs = ["apriltags"]
